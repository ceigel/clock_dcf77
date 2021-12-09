#![no_std]
#![no_main]

use panic_rtt_target as _;

use atsamd_hal as hal;
use atsamd_hal::{pac, prelude::*, time::Hertz};
use hal::clock::GenericClockController;
use hal::gpio::v2::pin::{self, FloatingInterrupt, Pin, Pins, PushPullOutput};
use pac::Peripherals;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

#[app(device = atsamd_hal::pac,  peripherals = true)]
const APP: () = {
    struct Resources {
        dcf_pin: Pin<pin::PA00, FloatingInterrupt>,
        debug_pin: Pin<pin::PA01, PushPullOutput>,
        timer: pac::TC3,
    }
    #[init(spawn=[])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        rprintln!("Initializing");
        let mut device: Peripherals = cx.device;

        let mut clocks = GenericClockController::with_external_32kosc(device.GCLK, &mut device.PM, &mut device.SYSCTRL, &mut device.NVMCTRL);

        let pins = Pins::new(device.PORT);
        let dcf_pin: Pin<pin::PA00, FloatingInterrupt> = pins.pa00.into();

        // Use this pin for debugging decoded signal state with oscilloscope
        let output_pin = pins.pa01.into_push_pull_output();

        let gclk0 = clocks.gclk0();
        let timer_clock = clocks.tcc2_tc3(&gclk0);
        rprintln!("Timer clock: {}", timer_clock.map(|tck| tck.freq().0).unwrap_or(0));
        let timer = device.TC3;

        device.PM.apbcmask.modify(|_, w| w.tc3_().set_bit());

        let count = timer.count32_mut();
        count.ctrla.modify(|_, w| w.enable().clear_bit());
        while count.status.read().syncbusy().bit_is_set() {}

        count.ctrla.write(|w| w.swrst().set_bit());
        while count.status.read().syncbusy().bit_is_set() {}

        while count.ctrla.read().bits() & 1 != 0 {}

        count.ctrlc.modify(|_, w| {
            w.cpten0().set_bit();
            w.cpten1().set_bit();
            w
        });
        count.evctrl.modify(|_, w| {
            w.tcei().set_bit();
            w.evact().pwp();
            //w.tcinv().set_bit();
            w
        });
        count.ctrla.modify(|_, w| {
            w.prescaler().div1024();
            w.runstdby().set_bit();
            w.enable().set_bit();
            w
        });

        rprintln!("Init successful");
        init::LateResources {
            dcf_pin: dcf_pin,
            debug_pin: output_pin,
            timer,
        }
    }

    #[allow(clippy::empty_loop)]
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {}
    }

    /*<S-F6>
    #[task(binds = TIM3, priority=2, resources=[timer, dcf_pin, debug_pin])]
    fn tim3(cx: tim3::Context) {
        let timer: &mut pac::TCC2 = cx.resources.timer;
        let capture = timer.ccr1.read().bits();
        let flags = timer.sr.read().bits();
        timer.sr.modify(|_, w| {
            w.uif().clear_bit();
            w.tif().clear_bit();
            w
        });
        let msg = if capture > 15000 { " minute mark" } else { "" };
        rprintln!("tick {}, flags: {:b}{}", capture, flags, msg);
    }
    */
};
