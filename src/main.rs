#![no_std]
#![no_main]
mod datetime_converter;
mod dcf77_decoder;
mod time_display;

use panic_rtt_target as _;

use chrono::naive::NaiveDateTime;
use core::fmt::Write;
use cortex_m::peripheral::DWT;
use datetime_converter::DCF77DateTimeConverter;
use dcf77_decoder::DCF77Decoder;
use feather_f405::hal as stm32f4xx_hal;
use feather_f405::{hal::prelude::*, pac, setup_clocks};
use ht16k33::{Dimming, Display, HT16K33};
use rtcc::Rtcc;
use rtic::app;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use stm32f4xx_hal::{
    gpio::{gpioa, gpiob, gpioc, Alternate, AlternateOD, Output, PushPull, AF2, AF4},
    i2c::I2c,
    rtc::Rtc,
};
use time_display::{display_error, show_rtc_time};

const DISP_I2C_ADDR: u8 = 0x77;

pub(crate) const TICKS_PER_SECOND: u32 = 10_000;
pub(crate) const MINUTE_MARKER_TIME: u32 = 15_000;
pub(crate) const TRUE_BIT_TIME: u32 = 1500;

type SegmentDisplay =
    HT16K33<I2c<pac::I2C1, (gpiob::PB6<AlternateOD<AF4>>, gpiob::PB7<AlternateOD<AF4>>)>>;

fn sync_rtc(rtc: &mut Rtc, dt: &NaiveDateTime) {
    rtc.set_datetime(dt).expect("To be able to set datetime");
}

pub const CUTOFF_THRESHOLD: u8 = 198;
pub struct TickBuffer {
    pub phase: Option<usize>,
    pub counts: [u8; 1000],
    pub integral: [u32; 1000],
    max_val: u8,
    down_edge: Option<u32>,
    up_edge: Option<u32>,
}

impl Default for TickBuffer {
    fn default() -> Self {
        Self {
            phase: None,
            counts: [0; 1000],
            integral: [0; 1000],
            max_val: 0,
            down_edge: None,
            up_edge: None,
        }
    }
}

impl core::fmt::Display for TickBuffer {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_fmt(format_args!("bins = [ {}", self.counts[0]))?;
        for x in &self.counts[1..] {
            formatter.write_fmt(format_args!(", {}", x))?;
        }
        formatter.write_fmt(format_args!("]; "))?;
        formatter.write_fmt(format_args!("integral = [ {}", self.integral[0]))?;
        for x in &self.integral[1..] {
            formatter.write_fmt(format_args!(", {}", x))?;
        }
        formatter.write_fmt(format_args!("]"))
    }
}

fn increment_vals(slice: &mut [u8], add: i8) {
    for a in slice {
        if add > 0 {
            *a = a.saturating_add(add as u8);
        } else {
            *a = a.saturating_sub(-add as u8);
        }
    }
}

pub enum Edge {
    Down,
    Up,
}

impl TickBuffer {
    /*pub fn add_tick(&mut self, idx: usize, direction: Edge) {
        let span = 10;
        let add = match direction {
            Edge::Down => 1, Edge::Up => -1
        }
        let from = if idx < span { 0 } else { idx - span };
        let to = if idx + span >= self.counts.len() {
            self.counts.len()
        } else {
            idx + span + 1
        };
        increment_vals(&mut self.counts[from..to], add);
        if idx < span {
            let from = self.counts.len() - span + idx;
            increment_vals(&mut self.counts[from..], add);
        } else if self.counts.len() <= idx + span {
            let to = idx + span - self.counts.len();
            increment_vals(&mut self.counts[0..to], add);
        }
        self.max_val = *self.counts.iter().max().unwrap();
        if self.max_val > CUTOFF_THRESHOLD {
            self.reduce_values();
        }
    } */

    pub fn add_tick(&mut self, reg: u32, direction: Edge) {
        match direction {
            Edge::Down => {
                self.down_edge.replace(reg);
                if let Some(up_edge) = self.up_edge {
                    self.add_level(up_edge as usize, reg as usize, -1);
                }
            }
            Edge::Up => {
                self.up_edge.replace(reg);
                if let Some(down_edge) = self.down_edge {
                    self.add_level(down_edge as usize, reg as usize, 1);
                }
            }
        };
        self.max_val = core::cmp::max(self.max_val, self.counts[reg as usize]);
        if self.max_val > CUTOFF_THRESHOLD {
            self.reduce_values();
        }
    }

    fn add_level(&mut self, from: usize, to: usize, add: i8) {
        if from < to {
            increment_vals(&mut self.counts[from..to], add);
        } else {
            increment_vals(&mut self.counts[from..], add);
            increment_vals(&mut self.counts[0..to], add);
        }
    }

    fn reduce_values(&mut self) {
        for x in self.counts.as_mut() {
            *x >>= 1;
        }
        self.max_val >>= 1;
    }

    fn compute_phase(&mut self) {
        const WINDOW_SIZE: usize = 50;
        const STEPS: usize = 2;
        let mut integral: u32 = 0;
        let mut max_index = 0;
        let mut max_val = 0;
        for p in 0..STEPS {
            let idx = p * WINDOW_SIZE;
            integral += self.counts[idx..(idx + WINDOW_SIZE)]
                .iter()
                .map(|x| *x as u32)
                .sum::<u32>();
        }
        for idx in 0..self.counts.len() {
            if integral > max_val {
                max_val = integral;
                max_index = idx;
            }
            self.integral[idx] = integral;
            integral = integral.saturating_sub((self.counts[idx] as u32) * (STEPS as u32));
            for p in 0..STEPS {
                let step = (p + 1) * WINDOW_SIZE;
                integral += self.counts[self.wrap(idx + step)] as u32;
            }
        }
        self.phase.replace(max_index);
    }

    fn wrap(&self, val: usize) -> usize {
        wrap_val(val, self.counts.len())
    }
}

fn wrap_val(val: usize, max: usize) -> usize {
    let mut val = val;
    while (val > max) {
        val -= max;
    }
    val
}

const TIMER_RANGE: u32 = 8_000_u32;
const TIMER_MAX: u32 = TIMER_RANGE << 2;

fn init_timer(timer: pac::TIM3, clocks: &stm32f4xx_hal::rcc::Clocks) -> pac::TIM3 {
    let psc = (((2 * clocks.pclk1().0) / (TIMER_RANGE - 1)) - 1) as u16;

    timer.psc.write(|w| w.psc().bits(psc));
    timer.arr.write(|w| w.arr().bits(TIMER_MAX as u16));

    timer.cr1.modify(|_, w| w.urs().set_bit());
    timer.egr.write(|w| w.ug().set_bit());
    timer.cr1.modify(|_, w| w.urs().clear_bit());

    timer.ccmr1_input().write(|w| {
        w.cc1s().ti1();
        w.cc2s().ti1();
        //w.ic1f().fck_int_n8();
        w
    });

    timer.ccer.write(|w| {
        w.cc1p().set_bit();
        w.cc1np().clear_bit(); //active falling edge
        w.cc2p().clear_bit();
        w.cc2np().clear_bit(); // active raising edge
        w.cc1e().set_bit(); // enable captures
        w.cc2e().set_bit();
        w
    });
    /*timer.smcr.write(|w| {
        w.etp().inverted();
        w.sms().reset_mode();
        w.ts().ti1fp1();
        w
    });*/

    timer.dier.write(|w| w.cc1ie().set_bit().cc2ie().set_bit());
    timer.cr1.modify(|_, w| {
        w.cen().set_bit();
        w.udis().clear_bit();
        w
    });
    timer
}

#[app(device = feather_f405::hal::stm32, monotonic = rtic::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        dcf_pin: gpioa::PA6<Alternate<2>>,
        debug_pin: gpioc::PC6<Output<PushPull>>,
        timer: pac::TIM3,
        dcf77: DCF77Decoder,
        rtc: Rtc,
        segment_display: SegmentDisplay,
        ticks: TickBuffer,
        #[init(None)]
        last_second_marker: Option<u32>,
        #[init(0)]
        count: u8,
        output2: UpChannel,
    }

    #[init(spawn=[])]
    fn init(cx: init::Context) -> init::LateResources {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 512
                    mode: BlockIfFull
                    name: "Up zero"
                }
                1: {
                    size: 5000
                    mode: BlockIfFull
                    name: "Up one"
                }
            }
        };
        set_print_channel(channels.up.0);
        let out2 = channels.up.1;
        let mut core = cx.core;
        let device = cx.device;
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let rcc = device.RCC;
        rcc.apb1enr.write(|w| w.tim3en().set_bit());
        rcc.apb1rstr.write(|w| w.tim3rst().set_bit());
        rcc.apb1rstr.write(|w| w.tim3rst().clear_bit());
        let clocks = setup_clocks(rcc);

        let gpioa = device.GPIOA.split();
        let pin = gpioa.pa6.into_floating_input().into_alternate::<AF2>();

        // Use this pin for debugging decoded signal state with oscilloscope
        let gpioc = device.GPIOC.split();
        let output_pin = gpioc.pc6.into_push_pull_output();
        let gpiob = device.GPIOB.split();
        let scl = gpiob.pb6.into_alternate_open_drain::<AF4>();
        let sda = gpiob.pb7.into_alternate_open_drain::<AF4>();
        let i2c = I2c::new(device.I2C1, (scl, sda), 400.khz(), clocks);
        let mut ht16k33 = HT16K33::new(i2c, DISP_I2C_ADDR);
        ht16k33.initialize().expect("Failed to initialize ht16k33");
        ht16k33
            .set_display(Display::ON)
            .expect("Could not turn on the display!");
        ht16k33
            .set_dimming(Dimming::BRIGHTNESS_MAX)
            .expect("Could not set dimming!");
        display_error(&mut ht16k33, 0);
        ht16k33
            .write_display_buffer()
            .expect("Could not write 7-segment display");
        let mut pwr = device.PWR;
        let rtc = Rtc::new(device.RTC, 255, 127, false, &mut pwr);

        let timer = init_timer(device.TIM3, &clocks);
        rprintln!("Init successful");
        init::LateResources {
            dcf_pin: pin,
            debug_pin: output_pin,
            timer,
            dcf77: DCF77Decoder::new(),
            rtc,
            segment_display: ht16k33,
            ticks: TickBuffer::default(),
            output2: out2,
        }
    }

    #[allow(clippy::empty_loop)]
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {}
    }

    #[task(binds = TIM3, priority=2, resources=[timer, dcf_pin, debug_pin, dcf77, ticks, count, segment_display, rtc, output2, last_second_marker])]
    fn tim3(cx: tim3::Context) {
        let timer: &mut pac::TIM3 = cx.resources.timer;
        let decoder = cx.resources.dcf77;
        let flags = timer.sr.read();
        let fbits = flags.bits();
        let c1 = if flags.cc1if().is_match_() {
            let c1 = timer.ccr1.read().bits() >> 3;
            let count: &mut u8 = cx.resources.count;
            rprintln!("{} - {} : {:016b}", c1, *count, fbits);
            cx.resources.ticks.add_tick(c1 >> 2, Edge::Down);
            *count += 1;
            if *count == 120 {
                *count = 0;
                cx.resources.ticks.compute_phase();
                writeln!(cx.resources.output2, "{}", cx.resources.ticks).unwrap();
            }
            Some(c1)
        } else {
            None
        };
        let mut c2 = if flags.cc2if().is_match_() {
            let c2 = timer.ccr2.read().bits() >> 3;
            cx.resources.ticks.add_tick(c2 >> 2, Edge::Up);
            Some(c2 >> 3)
        } else {
            None
        };
        match (c1, *cx.resources.last_second_marker) {
            (Some(c1), Some(last_second)) => {
                if wrap_val((4_000 + c1 - last_second) as usize, 4_000) > 1800 {
                    decoder.add_minute();
                }
                cx.resources.last_second_marker.replace(c1);
            }
            (Some(c1), None) => {
                cx.resources.last_second_marker.replace(c1);
            }
            _ => {}
        }
        match (c2, *cx.resources.last_second_marker) {
            (Some(c2), Some(last_second)) => {
                let diff = wrap_val((4_000 + c2 - last_second) as usize, 4_000);
                if (90..110).contains(&diff) {
                    decoder.add_second(false);
                } else if (190..210).contains(&diff) {
                    decoder.add_second(true);
                }
            }
            _ => {}
        }

        if let Some(datetime_bits) = decoder.last_bits() {
            decoder.reset_last_bits();
            let converter = DCF77DateTimeConverter::new(datetime_bits);
            match converter.dcf77_decoder() {
                Err(err) => {
                    rprintln!("Decoding error: {:?}", err);
                }
                Ok(dt) => {
                    sync_rtc(cx.resources.rtc, &dt);
                    rprintln!("Good date: {:?}", dt);
                }
            }
        }
        let display = cx.resources.segment_display;
        show_rtc_time(cx.resources.rtc, display);
        timer.sr.modify(|_, w| {
            w.tif().clear_bit();
            w
        });
    }
};
