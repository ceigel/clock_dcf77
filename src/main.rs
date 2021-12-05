#![no_std]
#![no_main]
mod datetime_converter;
mod dcf77_decoder;
mod time_display;

use panic_rtt_target as _;

use chrono::naive::NaiveDateTime;
use cortex_m::peripheral::DWT;
use datetime_converter::DCF77DateTimeConverter;
use dcf77_decoder::DCF77Decoder;
use feather_f405::hal as stm32f4xx_hal;
use feather_f405::{hal::prelude::*, pac, setup_clocks};
use ht16k33::{Dimming, Display, HT16K33};
use rtcc::{Hours, Rtcc};
use rtic::app;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use stm32f4xx_hal::{
    gpio::{gpioa, gpiob, gpioc, Alternate, AlternateOD, Output, PushPull, AF2, AF4},
    i2c::I2c,
    rtc::Rtc,
};
use time_display::{blink_second, display_error, display_time};

const DISP_I2C_ADDR: u8 = 0x77;
const AFTER_PHASE_THRESHOLD: usize = 30;
const BEFORE_PHASE_THRESHOLD: usize = 10;

type SegmentDisplay = HT16K33<I2c<pac::I2C1, (gpiob::PB6<AlternateOD<AF4>>, gpiob::PB7<AlternateOD<AF4>>)>>;

fn sync_rtc(rtc: &mut Rtc, dt: &NaiveDateTime) {
    rtc.set_datetime(dt).expect("To be able to set datetime");
}

pub const CUTOFF_THRESHOLD: u8 = 198;
pub const PHASE_MAX: u8 = 60;

pub struct TickBuffer {
    pub phase: Option<usize>,
    pub counts: [u8; 1000],
    pub integral: [u32; 1000],
    max_val: u8,
    down_edge: Option<u32>,
    up_edge: Option<u32>,
    tick_count: u8,
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
            tick_count: 0,
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
        self.tick_count = self.tick_count.saturating_add(1);
        if self.tick_count == PHASE_MAX {
            self.compute_phase();
            self.tick_count = 0;
        }
        if self.max_val > CUTOFF_THRESHOLD {
            self.reduce_values();
        }
    }

    pub fn down_edge_in_phase(&self, val: u32) -> bool {
        if let Some(phase) = self.phase {
            if (self.wrap(self.counts.len() + (val as usize) - phase) <= AFTER_PHASE_THRESHOLD)
                || (self.wrap(self.counts.len() + phase - (val as usize)) <= BEFORE_PHASE_THRESHOLD)
            {
                return true;
            }
            rprintln!("Down edge not in phase. val: {}, phase: {}", val, phase);
        }
        false
    }

    pub fn compute_bit(&self, val: u32) -> Option<bool> {
        if let Some(phase) = self.phase {
            let zero_phase = self.wrap(self.counts.len() + phase + 100);
            let one_phase = self.wrap(self.counts.len() + phase + 200);
            if (self.wrap(self.counts.len() + (val as usize) - zero_phase) <= AFTER_PHASE_THRESHOLD)
                || (self.wrap(self.counts.len() + zero_phase - (val as usize)) <= BEFORE_PHASE_THRESHOLD)
            {
                return Some(false);
            }
            if (self.wrap(self.counts.len() + (val as usize) - zero_phase) <= AFTER_PHASE_THRESHOLD)
                || (self.wrap(self.counts.len() + zero_phase - (val as usize)) <= BEFORE_PHASE_THRESHOLD)
            {
                return Some(true);
            }
            rprintln!("Bit not in phase. val: {} zero_phase: {}, one_phase: {}", val, zero_phase, one_phase);
        }
        None
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
        rprintln!("Compute phase");
        const WINDOW_SIZE: usize = 50;
        const STEPS: usize = 2;
        let mut integral: u32 = 0;
        let mut max_index = 0;
        let mut max_val = 0;
        for p in 0..STEPS {
            let idx = p * WINDOW_SIZE;
            integral += self.counts[idx..(idx + WINDOW_SIZE)].iter().map(|x| *x as u32).sum::<u32>();
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
    while val >= max {
        val -= max;
    }
    val
}

fn minute_detected(current_edge: u32, last_edge: Option<u32>) -> bool {
    last_edge
        .map(|last_edge| {
            let range: u32 = TIMER_MAX >> 3;
            wrap_val((range + current_edge - last_edge) as usize, range as usize) > 1800
        })
        .unwrap_or(false)
}
const TIMER_RANGE: u32 = 8_000_u32;
const TIMER_MAX: u32 = 32_000_u32;

fn init_timer(timer: pac::TIM3, clocks: &stm32f4xx_hal::rcc::Clocks) -> pac::TIM3 {
    let psc = (((2 * clocks.pclk1().0) / TIMER_RANGE) - 1) as u16;

    timer.psc.write(|w| w.psc().bits(psc));
    timer.arr.write(|w| w.arr().bits((TIMER_MAX - 1) as u16));

    timer.cr1.modify(|_, w| w.urs().set_bit());
    timer.egr.write(|w| w.ug().set_bit());
    timer.cr1.modify(|_, w| w.urs().clear_bit());

    timer.ccmr1_input().write(|w| {
        w.cc1s().ti1();
        w.cc2s().ti1();
        //        w.ic1f().fck_int_n8();
        w
    });

    timer.ccer.write(|w| {
        w.cc1p().set_bit();
        w.cc1np().clear_bit(); //active falling edge
        w.cc2p().clear_bit();
        w.cc2np().clear_bit(); // active raising edge
        w.cc1e().set_bit(); // enable captures
        w.cc2e().set_bit();
        w.cc3e().clear_bit();
        w.cc4e().clear_bit();
        w
    });

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
        #[init(false)]
        tick: bool,
        output2: UpChannel,
        #[init(None)]
        last_sync: Option<NaiveDateTime>,
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
        ht16k33.set_display(Display::ON).expect("Could not turn on the display!");
        ht16k33.set_dimming(Dimming::BRIGHTNESS_MAX).expect("Could not set dimming!");
        display_error(&mut ht16k33, 0);
        ht16k33.write_display_buffer().expect("Could not write 7-segment display");
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

    #[task(resources = [segment_display, tick])]
    fn show_tick(cx: show_tick::Context) {
        let tick = cx.resources.tick;
        *tick = !*tick;
        blink_second(*tick, cx.resources.segment_display);
    }

    #[task(resources = [segment_display, last_sync, rtc])]
    fn show_time(mut cx: show_time::Context) {
        rprintln!("Show time");
        let (h, m) = cx.resources.rtc.lock(|rtc| {
            let h = rtc.get_hours().expect("to read hours");
            let m = rtc.get_minutes().expect("to read minutes");
            (h, m)
        });

        let hours = match h {
            Hours::AM(hours) => hours,
            Hours::PM(hours) => hours,
            Hours::H24(hours) => hours,
        };

        let display = cx.resources.segment_display;
        display_time(display, hours, m, 0);
    }

    #[task(binds = TIM3, priority=2, resources=[timer, dcf_pin, debug_pin, dcf77, ticks,  rtc, output2, last_second_marker, last_sync], spawn=[show_tick, show_time])]
    fn tim3(cx: tim3::Context) {
        let timer: &mut pac::TIM3 = cx.resources.timer;
        let decoder = cx.resources.dcf77;
        let flags = timer.sr.read();
        let fbits = flags.bits();
        let binner = cx.resources.ticks;
        let debug_pin = cx.resources.debug_pin;
        let c1 = flags
            .cc1if()
            .is_match_()
            .then(|| {
                let c1 = timer.ccr1.read().bits() >> 3;
                rprintln!("{} : {:016b}", c1 % 1000, fbits);
                binner.add_tick(c1 % 1000, Edge::Down);
                c1
            })
            .and_then(|c1| binner.down_edge_in_phase(c1 % 1_000).then(|| c1));
        let bit = flags
            .cc2if()
            .is_match_()
            .then(|| {
                let c2 = timer.ccr2.read().bits() >> 3;
                binner.add_tick(c2 % 1000, Edge::Up);
                c2
            })
            .and_then(|c2| binner.compute_bit(c2 % 1000));
        if let Some(c1) = c1 {
            debug_pin.set_low();
            let last_second_marker = cx.resources.last_second_marker;
            if minute_detected(c1, *last_second_marker) {
                decoder.add_minute()
            }
            cx.spawn.show_tick().ok();
            last_second_marker.replace(c1);
        }
        if let Some(bit) = bit {
            debug_pin.set_high();
            decoder.add_second(bit);
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
                    cx.resources.last_sync.replace(dt);
                    cx.spawn.show_time().ok();
                    rprintln!("Good date: {:?}", dt);
                }
            }
        }
        timer.sr.modify(|_, w| {
            w.tif().clear_bit();
            w.uif().clear_bit();
            w
        });
    }
    extern "C" {
        fn DCMI();
    }
};
