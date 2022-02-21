#![no_std]
#![no_main]
mod datetime_converter;
mod dcf77_decoder;
mod time_display;

use panic_rtt_target as _;

use atsamd_hal as hal;
use atsamd_hal::pac;
use chrono::{naive::NaiveDateTime, Datelike, Timelike};
use core::fmt::Write;
use datetime_converter::DCF77DateTimeConverter;
use dcf77_decoder::DCF77Decoder;
use hal::clock::{
    ClockGenId, ClockSource, EicClock, Evsys0Clock, GenericClockController, Tcc0Tcc1Clock,
};
use hal::gpio::v2::pin::{self, AlternateC, Pin, Pins, PullUpInterrupt, PushPullOutput, Alternate, B};
use hal::adc::Adc;
use hal::prelude::*;
use hal::rtc::{ClockMode, Datetime, Rtc};
use hal::sercom::I2CMaster3;
use hal::time::{Hertz, KiloHertz};
use ht16k33::{Dimming, Display, HT16K33};
use pac::Peripherals;
use rtic::app;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use time_display::SegmentDisplayAdapter;

const DISP_I2C_ADDR: u8 = 0x77;
pub const PHASE_MAX: u8 = 60;
const TICKS_PER_SECOND: u32 = 3000;
const MULTIPLIER: u32 = 4;
const TIMER_MAX: u32 = MULTIPLIER * TICKS_PER_SECOND;
const TIMER_RANGE: u32 = MULTIPLIER * 1000;
const AFTER_PHASE_THRESHOLD: usize = 30;
const BEFORE_PHASE_THRESHOLD: usize = 10;
pub const CUTOFF_THRESHOLD: u8 = 198;

fn sync_rtc(rtc: &mut Rtc<ClockMode>, dt: &NaiveDateTime) {
    let time = Datetime {
        seconds: 0,
        minutes: dt.minute() as u8,
        hours: dt.hour() as u8,
        day: dt.day() as u8,
        month: dt.month() as u8,
        year: dt.year() as u8,
    };
    rtc.set_time(time);
}

pub struct TickBuffer {
    pub phase: Option<usize>,
    pub counts: [u8; 1000],
    pub integral: [u32; 1000],
    max_val: u8,
    down_edge: Option<u32>,
    up_edge: Option<u32>,
    pub tick_count: u8,
    pub noise_max: u32,
    pub integral_max: u32,
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
            noise_max: 0,
            integral_max: 0,
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

#[derive(PartialEq, Debug, Copy, Clone)]
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
                || (self.wrap(self.counts.len() + zero_phase - (val as usize))
                    <= BEFORE_PHASE_THRESHOLD)
            {
                return Some(false);
            }
            if (self.wrap(self.counts.len() + (val as usize) - one_phase) <= AFTER_PHASE_THRESHOLD)
                || (self.wrap(self.counts.len() + one_phase - (val as usize))
                    <= BEFORE_PHASE_THRESHOLD)
            {
                return Some(true);
            }
            rprintln!(
                "Bit not in phase. val: {} zero_phase: {}, one_phase: {}",
                val,
                zero_phase,
                one_phase
            );
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
        const WINDOW_SIZE: usize = 100;
        const STEPS: usize = 2;
        let mut integral: u32 = 0;
        let mut max_index = 0;
        self.integral_max = 0;
        for p in 0..STEPS {
            let idx = p * WINDOW_SIZE;
            integral += self.counts[0..(idx + WINDOW_SIZE)]
                .iter()
                .map(|x| *x as u32)
                .sum::<u32>();
        }
        for idx in 0..self.counts.len() {
            if integral > self.integral_max {
                self.integral_max = integral;
                max_index = idx;
            }
            self.integral[idx] = integral;
            integral = integral.saturating_sub((self.counts[idx] as u32) * (STEPS as u32));
            for p in 0..STEPS {
                let step = (p + 1) * WINDOW_SIZE;
                integral += self.counts[self.wrap(idx + step)] as u32;
            }
        }
        let noise_index = max_index + WINDOW_SIZE * STEPS;
        self.noise_max = 0;

        for idx in 0..WINDOW_SIZE {
            for p in 0..STEPS {
                let jdx = noise_index + idx * (p + 1);
                self.noise_max += (self.counts[self.wrap(jdx)] as u32) * ((STEPS - p) as u32);
            }
        }
        rprintln!(
            "Compute phase: {} max_integral {}, noise_max: {}",
            max_index,
            self.integral_max,
            self.noise_max
        );
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
            let range: u32 = TIMER_RANGE;
            wrap_val((range + current_edge - last_edge) as usize, range as usize) > 1800
        })
        .unwrap_or(false)
}

fn init_timer(timer: &mut pac::TCC0, pm: &mut pac::PM, tcc0_clock: &Option<Tcc0Tcc1Clock>) {
    tcc0_clock.as_ref().expect("To have tcc0 clock");
    pm.apbcmask.modify(|_, w| w.tcc0_().set_bit());

    timer.ctrla.write(|w| {
        w.enable().clear_bit();
        w.swrst().set_bit();
        w
    });
    while timer.syncbusy.read().swrst().bit_is_set() || timer.syncbusy.read().enable().bit_is_set()
    {
    }

    timer.per().write(|w| unsafe { w.per().bits(TIMER_MAX) });
    timer.intenclr.write(|w| {
        w.mc1().set_bit();
        w.mc2().set_bit();
        w.mc3().set_bit();
        w.err().set_bit();
        w.ovf().set_bit();
        w
    });

    timer.intenset.write(|w| {
        w.mc0().set_bit();
        w
    });
    timer.ctrla.modify(|_, w| {
        w.prescaler().div64();
        w.cpten0().set_bit();
        w
    });
    timer.evctrl.write(|w| {
        w.mcei0().set_bit();
        w
    });

    timer.ctrla.modify(|_, w| w.enable().set_bit());
    while timer.syncbusy.read().enable().bit_is_set() {}
}

fn init_eic(eic: &mut pac::EIC, pm: &mut pac::PM, eic_clock: &Option<EicClock>) {
    eic_clock.as_ref().expect("To have eic clock");
    pm.apbamask.modify(|_, w| w.eic_().set_bit());
    eic.ctrl.write(|w| {
        w.enable().clear_bit();
        w.swrst().set_bit();
        w
    });
    while eic.status.read().syncbusy().bit_is_set() || eic.ctrl.read().swrst().bit_is_set() {}

    eic.config[0].modify(|_, w| {
        w.filten2().set_bit();
        w.sense2().both();
        w
    });
    //no interrupt
    eic.intenclr.modify(|_, w| w.extint2().set_bit());

    //no wakeup
    eic.wakeup.modify(|_, w| w.wakeupen2().clear_bit());

    //with event
    eic.evctrl.modify(|_, w| w.extinteo2().set_bit());

    eic.ctrl.modify(|_, w| w.enable().set_bit());
    while eic.status.read().syncbusy().bit_is_set() {}
}

fn init_evsys(evsys: &mut pac::EVSYS, pm: &mut pac::PM, evsys_clock: &Option<Evsys0Clock>) {
    evsys_clock.as_ref().expect("To have evsys clock");
    pm.apbcmask.modify(|_, w| w.evsys_().set_bit());
    evsys.ctrl.write(|w| w.swrst().set_bit());
    evsys.user.write(|w| unsafe {
        w.channel().bits(1); //Channel 0 (n+1)
        w.user().bits(0x06); //TCC0 MC1
        w
    });
    evsys.channel.write(|w| {
        unsafe {
            w.channel().bits(0); // Channel 0
            w.evgen().bits(0x0E); // EXTINT2
        }
        w.path().asynchronous();
        w
    });
    while evsys.chstatus.read().usrrdy0().bit_is_clear() {}
}

type Scl = Pin<pin::PA23, AlternateC>;
type Sda = Pin<pin::PA22, AlternateC>;
type I2C = I2CMaster3<Sda, Scl>;
type SegmentDisplay = HT16K33<I2C>;

fn init_i2c(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    sercom3: pac::SERCOM3,
    pm: &mut pac::PM,
    sda: impl Into<Sda>,
    scl: impl Into<Scl>,
) -> I2C {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.sercom3_core(&gclk0).unwrap();
    let baud = baud.into();
    let sda = sda.into();
    let scl = scl.into();
    I2CMaster3::new(clock, baud, sercom3, pm, sda, scl)
}

fn rtt_init_done() {
    rprintln!("Initializing");
}

#[app(device = atsamd_hal::pac,  peripherals = true)]
const APP: () = {
    struct Resources {
        dcf_pin: Pin<pin::PA02, PullUpInterrupt>,
        debug_pin: Pin<pin::PA17, PushPullOutput>,
        eic: pac::EIC,
        evsys: pac::EVSYS,
        timer: pac::TCC0,
        dcf77: DCF77Decoder,
        rtc: Rtc<ClockMode>,
        segment_display: SegmentDisplayAdapter,
        ticks: TickBuffer,
        #[init(None)]
        last_second_marker: Option<u32>,
        #[init(false)]
        tick: bool,
        #[init(None)]
        last_sync: Option<NaiveDateTime>,
        #[init(0)]
        sync_points: u8,
        output2: UpChannel,
        #[init(false)]
        blinking: bool,
        adc: Adc<pac::ADC>,
        adc_pin: Pin<pin::PA07, Alternate<B>>
    }

    #[init(spawn=[])]
    fn init(cx: init::Context) -> init::LateResources {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 512
                    mode: NoBlockSkip
                    name: "Up zero"
                }
                1: {
                    size: 5000
                    mode: NoBlockSkip
                    name: "Up one"
                }
            }
        };
        set_print_channel(channels.up.0);
        let out2 = channels.up.1;
        rtt_init_done();

        let mut device: Peripherals = cx.device;
        let mut clocks = GenericClockController::with_external_32kosc(
            device.GCLK,
            &mut device.PM,
            &mut device.SYSCTRL,
            &mut device.NVMCTRL,
        );


        let timer_clock = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK3, 32, ClockSource::XOSC32K, true)
            .unwrap();
        let rtc_clock = clocks.rtc(&timer_clock).unwrap();
        let rtc = Rtc::clock_mode(device.RTC, rtc_clock.freq(), &mut device.PM);
        unsafe {
            let rtc = (*pac::RTC::ptr()).mode2_mut();
            rtc.intenset.modify(|_, w| w.alarm0().set_bit());
            rtc.mask0.modify(|_, w| w.sel().ss());
        }

        let pins = Pins::new(device.PORT);
        let dcf_pin: Pin<pin::PA02, PullUpInterrupt> = pins.pa02.into();

        // Use this pin for debugging decoded signal state with oscilloscope
        let output_pin = pins.pa17.into_push_pull_output();

        let tc = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK4, 250, ClockSource::DFLL48M, true) // clock divider 64, 3000 counts/s, clock period: 12000
            .expect("To set peripherals clock");
        let timer_clock = clocks.tcc0_tcc1(&tc);
        rprintln!(
            "Timer clock: {}",
            timer_clock.as_ref().map(|tck| tck.freq().0).unwrap_or(0)
        );
        let mut timer = device.TCC0;
        init_timer(&mut timer, &mut device.PM, &timer_clock);
        rprintln!("Timer init done");
        let eic_clock = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK5, 1, ClockSource::XOSC32K, true)
            .unwrap();
        let ec = clocks.eic(&eic_clock);
        rprintln!(
            "EIC clock: {}",
            ec.as_ref().map(|tck| tck.freq().0).unwrap_or(0)
        );
        init_eic(&mut device.EIC, &mut device.PM, &ec);
        rprintln!("EIC init done");
        init_evsys(&mut device.EVSYS, &mut device.PM, &clocks.evsys0(&tc));
        rprintln!("EVSYS init done");

        let scl: Scl = pins.pa23.into();
        let sda: Sda = pins.pa22.into();

        let i2c = init_i2c(
            &mut clocks,
            KiloHertz(400),
            device.SERCOM3,
            &mut device.PM,
            sda,
            scl,
        );

        let ht16k33 = HT16K33::new(i2c, DISP_I2C_ADDR);
        let mut segment_display = SegmentDisplayAdapter::new(ht16k33);
        segment_display.display_error(0);

        let adc = Adc::adc(device.ADC, &mut device.PM, &mut clocks);
        let adc_pin: Pin<pin::PA07, Alternate<B>> = pins.pa07.into();
        cx.read_battery.spawn().unwrap();
        rprintln!("Init successful");
        init::LateResources {
            dcf_pin,
            debug_pin: output_pin,
            timer,
            eic: device.EIC,
            evsys: device.EVSYS,
            dcf77: DCF77Decoder::new(),
            rtc,
            segment_display,
            ticks: TickBuffer::default(),
            output2: out2,
            adc,
            adc_pin
        }
    }

    #[allow(clippy::empty_loop)]
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {}
    }

    #[task(resources = [segment_display])]
    fn show_tick(cx: show_tick::Context, tick: bool) {
        cx.resources.segment_display.blink_second(tick);
    }

    #[task(resources = [segment_display, adc, adc_pin, blinking], schedule=[read_battery])]
    fn read_battery(mut cx: read_battery::Context) {
        const RESOLUTION_BITS:u32 = 10;
        let data: u16 = cx.resources.adc.read(cx.resources.adc_pin).unwrap();
        let max_range = 1 << RESOLUTION_BITS;
        let voltage = ((data as f32) * 3.3) / (max_range as f32);
        rprintln!("Battery level {} - {}", voltage, data);
        let mid = max_range >> 1;
        if data < mid + max_range / 4 && *cx.resources.blinking {
            *cx.resources.blinking = true;
            cx.resources.segment_display.lock(|sd| sd.blink_all(true));
        } 
        if data > mid + max_range / 2 && *cx.resources.blinking {
            *cx.resources.blinking = false;
            cx.resources.segment_display.lock(|sd| sd.blink_all(false));
        }

        cx.schedule.read_battery(48_000_000*60).ok();
    }

    #[task(resources = [segment_display, last_sync, rtc])]
    fn show_time(mut cx: show_time::Context, sync_points: u8) {
        rprintln!("Show time");
        if cx.resources.last_sync.lock(|ls| ls.clone()).is_some() {
            let (h, m) = cx.resources.rtc.lock(|rtc| {
                let h = rtc.current_time().hours;
                let m = rtc.current_time().minutes;
                (h, m)
            });

            let hours = match h {
                24.. => h - 12,
                _ => h,
            };

            cx.resources.segment_display.display_time(hours, m, sync_points);
        } else {
            cx.resources.segment_display.display_error(sync_points);
        }
    }

    #[task(binds = TCC0, priority=8, resources=[timer, dcf_pin, debug_pin, dcf77, rtc, last_sync, output2, sync_points, evsys, ticks, last_second_marker], spawn=[show_tick, show_time])]
    fn tcc0(cx: tcc0::Context) {
        let dcf_pin = cx.resources.dcf_pin;
        let decoder = cx.resources.dcf77;
        let debug_pin = cx.resources.debug_pin;
        let timer: &mut pac::TCC0 = cx.resources.timer;
        let flags = timer.intflag.read();
        let binner = cx.resources.ticks;
        if flags.err().bit_is_set() {
            rprintln!("Error");
            timer.intflag.modify(|_, w| w.err().set_bit());
        }
        if flags.mc0().bit_is_set() {
            let cc0 = timer.cc()[0].read().cc().bits() / 3;
            timer.intflag.modify(|_, w| w.mc0().set_bit());
            let level_low = dcf_pin.is_low().unwrap();
            let edge = if level_low { Edge::Down } else { Edge::Up };

            let tick = cc0 % 1000;
            rprintln!("{} - {}", tick, level_low);
            let should_print = binner.tick_count == PHASE_MAX - 1;
            binner.add_tick(tick, edge);

            if should_print {
                writeln!(cx.resources.output2, "{}", binner).expect("to write data");
            }
            cx.spawn.show_tick(!level_low).ok();
            if level_low && binner.down_edge_in_phase(tick) {
                debug_pin.set_low().unwrap();
                let last_second_marker = cx.resources.last_second_marker;
                if minute_detected(cc0, *last_second_marker) {
                    decoder.add_minute()
                }
                last_second_marker.replace(cc0);
            }
            if edge == Edge::Up {
                if let Some(bit) = binner.compute_bit(tick) {
                    debug_pin.set_high().unwrap();
                    decoder.add_second(bit);
                }
            }
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
                    *cx.resources.sync_points |= 1;
                    cx.spawn.show_time(*cx.resources.sync_points).ok();
                    cx.resources.last_sync.replace(dt);
                    rprintln!("Good date: {:?}", dt);
                }
            }
        }
    }

    #[task(binds = RTC, resources=[sync_points], spawn=[show_time])]
    fn rtc(mut cx: rtc::Context) {
        let int_flags = unsafe {
            let rtc = (*pac::RTC::ptr()).mode2_mut();
            let flag_bits = rtc.intflag.read().bits();
            rtc.intflag.write(|w| w.bits(flag_bits));
            flag_bits
        };
        rprintln!("RTC interrupt {:#x}", int_flags);
        let old_sync_points = cx.resources.sync_points.lock(|sp| {
            let old_sp = *sp;
            *sp = *sp << 1;
            old_sp
        });
        cx.spawn.show_time(old_sync_points).ok();
    }

    extern "C" {
        fn SERCOM5();
    }
};
