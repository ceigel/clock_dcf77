#![no_std]
#![no_main]
mod datetime_converter;
mod dcf77_decoder;
mod time_display;

use panic_rtt_target as _;

use atsamd_hal as hal;
use atsamd_hal::pac;
use chrono::{naive::NaiveDateTime, Datelike, Timelike};
use datetime_converter::DCF77DateTimeConverter;
use dcf77_decoder::DCF77Decoder;
use hal::clock::{ClockGenId, ClockSource, EicClock, Evsys0Clock, GenericClockController, Tcc0Tcc1Clock};
use hal::gpio::v2::pin::{self, AlternateC, Pin, Pins, PullUpInterrupt, PushPullOutput};
use hal::prelude::*;
use hal::rtc::{ClockMode, Datetime, Rtc};
use hal::sercom::I2CMaster3;
use hal::time::{Hertz, KiloHertz};
use ht16k33::{Dimming, Display, HT16K33};
use pac::Peripherals;
use rtic::app;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use time_display::{blink_second, display_error, display_time};

const DISP_I2C_ADDR: u8 = 0x77;

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

pub const PHASE_MAX: u8 = 60;
const TIMER_MAX: u32 = 54545;

fn init_timer(timer: &mut pac::TCC0, pm: &mut pac::PM, tcc0_clock: &Option<Tcc0Tcc1Clock>) {
    tcc0_clock.as_ref().expect("To have tcc0 clock");
    pm.apbcmask.modify(|_, w| w.tcc0_().set_bit());

    timer.ctrla.write(|w| {
        w.enable().clear_bit();
        w.swrst().set_bit();
        w
    });
    while timer.syncbusy.read().swrst().bit_is_set() || timer.syncbusy.read().enable().bit_is_set() {}

    timer.per().write(|w| unsafe { w.per().bits(TIMER_MAX) });
    timer.intenset.write(|w| {
        w.mc0().set_bit();
        w.err().set_bit();
        w.ovf().set_bit();
        w
    });
    timer.ctrla.modify(|_, w| {
        w.prescaler().div8();
        w.cpten0().set_bit();
        w.cpten1().set_bit();
        w
    });
    timer.evctrl.write(|w| {
        w.mcei0().set_bit();
        w.mcei1().set_bit();
        w.evact1().ppw();
        w.tcei1().set_bit();
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
        //        w.filten2().set_bit();
        w.sense2().low();
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
        w.user().bits(0x05); //TCC0 EV0
        w
    });
    evsys.channel.write(|w| {
        unsafe {
            w.channel().bits(0); // Channel 0
            w.evgen().bits(0x0E); // EXTINT2
        }
        w.edgsel().both_edges();
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

pub struct CurrentTimeMeasurements {
    pub cc0: u32,
    pub cc1: u32,
}

impl Default for CurrentTimeMeasurements {
    fn default() -> Self {
        CurrentTimeMeasurements { cc0: 0, cc1: 0 }
    }
}

impl CurrentTimeMeasurements {
    fn valid(&self) -> bool {
        self.cc0 + self.cc1 > 900
    }
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
        segment_display: SegmentDisplay,
        #[init(false)]
        tick: bool,
        output2: UpChannel,
        #[init(None)]
        last_sync: Option<NaiveDateTime>,
        #[init(CurrentTimeMeasurements{cc0:0, cc1:0})]
        current_measurements: CurrentTimeMeasurements,
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
        let out2 = channels.up.1;
        set_print_channel(channels.up.0);
        rtt_init_done();
        let mut device: Peripherals = cx.device;
        let mut clocks = GenericClockController::with_external_32kosc(device.GCLK, &mut device.PM, &mut device.SYSCTRL, &mut device.NVMCTRL);
        let timer_clock = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK3, 32, ClockSource::XOSC32K, true)
            .unwrap();
        let rtc_clock = clocks.rtc(&timer_clock).unwrap();
        let rtc = Rtc::clock_mode(device.RTC, rtc_clock.freq(), &mut device.PM);

        let pins = Pins::new(device.PORT);
        let dcf_pin: Pin<pin::PA02, PullUpInterrupt> = pins.pa02.into();

        // Use this pin for debugging decoded signal state with oscilloscope
        let output_pin = pins.pa17.into_push_pull_output();

        let tc = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK4, 187, ClockSource::DFLL48M, true) // clock divider 8, 32085 counts/s, clock period: 54545
            .expect("To set peripherals clock");
        let timer_clock = clocks.tcc0_tcc1(&tc);
        rprintln!("Timer clock: {}", timer_clock.as_ref().map(|tck| tck.freq().0).unwrap_or(0));
        let mut timer = device.TCC0;
        init_timer(&mut timer, &mut device.PM, &timer_clock);
        rprintln!("Timer init done");
        init_eic(&mut device.EIC, &mut device.PM, &clocks.eic(&tc));
        rprintln!("EIC init done");
        init_evsys(&mut device.EVSYS, &mut device.PM, &clocks.evsys0(&tc));
        rprintln!("EVSYS init done");

        let scl: Scl = pins.pa23.into();
        let sda: Sda = pins.pa22.into();

        let i2c = init_i2c(&mut clocks, KiloHertz(400), device.SERCOM3, &mut device.PM, sda, scl);

        let mut ht16k33 = HT16K33::new(i2c, DISP_I2C_ADDR);
        ht16k33.initialize().expect("Failed to initialize ht16k33");
        ht16k33.set_display(Display::ON).expect("Could not turn on the display!");
        ht16k33.set_dimming(Dimming::BRIGHTNESS_MAX).expect("Could not set dimming!");
        display_error(&mut ht16k33, 0);
        ht16k33.write_display_buffer().expect("Could not write 7-segment display");

        rprintln!("Init successful");
        init::LateResources {
            dcf_pin,
            debug_pin: output_pin,
            timer,
            eic: device.EIC,
            evsys: device.EVSYS,
            dcf77: DCF77Decoder::new(),
            rtc,
            segment_display: ht16k33,
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
            let h = rtc.current_time().hours;
            let m = rtc.current_time().minutes;
            (h, m)
        });

        let hours = match h {
            24.. => h - 12,
            _ => h,
        };

        let display = cx.resources.segment_display;
        display_time(display, hours, m, 0);
    }

    #[task(binds = TCC0, priority=8, resources=[timer, dcf_pin, debug_pin, dcf77, rtc, output2, last_sync, current_measurements], spawn=[show_tick, show_time])]
    fn tcc0(cx: tcc0::Context) {
        let decoder = cx.resources.dcf77;
        let debug_pin = cx.resources.debug_pin;
        let timer: &mut pac::TCC0 = cx.resources.timer;
        let flags = timer.intflag.read();
        let current_measurements = cx.resources.current_measurements;
        if flags.ovf().bit_is_set() {
            rprintln!("Minute marker");
            decoder.add_minute();
            *current_measurements = CurrentTimeMeasurements::default();
            timer.intflag.modify(|_, w| w.ovf().set_bit());
        }
        if flags.err().bit_is_set() {
            rprintln!("Error");
            timer.intflag.modify(|_, w| w.err().set_bit());
        }
        if flags.mc0().bit_is_set() {
            let cc0 = timer.cc()[0].read().cc().bits() >> 5;
            current_measurements.cc0 += cc0;
            cx.spawn.show_tick().ok();
            timer.intflag.modify(|_, w| w.mc0().set_bit());
        }
        if flags.mc1().bit_is_set() {
            let cc1 = timer.cc()[1].read().cc().bits() >> 5;
            current_measurements.cc1 += cc1;
            timer.intflag.modify(|_, w| w.mc1().set_bit());
        }
        if current_measurements.valid() {
            let CurrentTimeMeasurements { cc0, cc1 } = *current_measurements;
            *current_measurements = CurrentTimeMeasurements::default();
            rprintln!("CC0: {}, CC1: {}", cc0, cc1);
            let bit = match cc1 {
                70..=130 => Some(false),
                170..=230 => Some(true),
                _ => None,
            };
            if let Some(bit) = bit {
                debug_pin.set_high().unwrap();
                decoder.add_second(bit);
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
                    cx.resources.last_sync.replace(dt);
                    cx.spawn.show_time().ok();
                    rprintln!("Good date: {:?}", dt);
                }
            }
        }
    }
    extern "C" {
        fn SERCOM5();
    }
};
