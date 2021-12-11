#![no_std]
#![no_main]

use panic_rtt_target as _;

use atsamd_hal as hal;
use atsamd_hal::{pac, prelude::*, time::Hertz};
use hal::clock::{EicClock, Evsys0Clock, GenericClockController, Tcc2Tc3Clock};
use hal::gpio::v2::pin::{self, FloatingInterrupt, Pin, Pins, PushPullOutput};
use pac::Peripherals;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

fn init_timer(timer: &mut pac::TC3, pm: &mut pac::PM, tc3_clock: &Option<Tcc2Tc3Clock>) {
    if tc3_clock.is_none() {
        return;
    }

    pm.apbcmask.modify(|_, w| w.tc3_().set_bit());

    let count = timer.count32_mut();

    count.ctrla.write(|w| w.swrst().set_bit());
    while count.status.read().syncbusy().bit_is_set() {}

    while count.ctrla.read().bits() & 1 != 0 {}

    count.evctrl.write(|w| {
        w.tcei().set_bit();
        w.evact().pwp();
        //w.tcinv().set_bit();
        w
    });
    count.ctrla.modify(|_, w| {
        w.prescaler().div1024();
        w.runstdby().clear_bit();
        w.wavegen().nfrq();
        w
    });
    count.ctrlc.write(|w| {
        w.cpten0().set_bit();
        w.cpten1().set_bit();
        w
    });
    count.intenset.write(|w| {
        w.mc0().set_bit();
        w.mc1().set_bit();
        w
    });
    while count.status.read().syncbusy().bit_is_set() {}
    count.ctrla.modify(|_, w| w.enable().set_bit());
}

fn init_eic(eic: &mut pac::EIC, pm: &mut pac::PM, _eic_clock: &Option<EicClock>) {
    pm.apbamask.modify(|_, w| w.eic_().set_bit());
    eic.ctrl.write(|w| w.swrst().set_bit());
    while eic.status.read().syncbusy().bit_is_set() || eic.ctrl.read().swrst().bit_is_set() {}

    eic.config[0].modify(|_, w| {
        w.filten0().set_bit();
        w.sense0().both();
        w
    });
    //no interrupt
    eic.intenclr.modify(|_, w| w.extint0().set_bit());

    //no wakeup
    eic.wakeup.modify(|_, w| w.wakeupen0().clear_bit());

    //with event
    eic.evctrl.modify(|_, w| w.extinteo0().set_bit());
    while eic.status.read().syncbusy().bit_is_set() {}
    eic.ctrl.modify(|_, w| w.enable().set_bit());
}

fn init_evsys(evsys: &mut pac::EVSYS, pm: &mut pac::PM, _eic_clock: &Option<Evsys0Clock>) {
    pm.apbcmask.modify(|_, w| w.evsys_().set_bit());
    evsys.user.write(|w| unsafe {
        w.channel().bits(1); //Channel 0 (n+1)
        w.user().bits(0x12); //TC3
        w
    });
    evsys.channel.write(|w| {
        unsafe {
            w.channel().bits(0); // Channel 0
            w.evgen().bits(0x0C);
        }
        w.edgsel().no_evt_output();
        w.path().asynchronous();
        w
    });

    while evsys.chstatus.read().usrrdy0().bit_is_set() {}
}

fn rtt_init_done() {
    rprintln!("Initializing");
}

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
        rtt_init_done();
        let mut device: Peripherals = cx.device;

        let mut clocks = GenericClockController::with_external_32kosc(device.GCLK, &mut device.PM, &mut device.SYSCTRL, &mut device.NVMCTRL);

        let pins = Pins::new(device.PORT);
        let dcf_pin: Pin<pin::PA00, FloatingInterrupt> = pins.pa00.into();

        // Use this pin for debugging decoded signal state with oscilloscope
        let output_pin = pins.pa01.into_push_pull_output();

        let gclk0 = clocks.gclk0();
        let timer_clock = clocks.tcc2_tc3(&gclk0);
        rprintln!("Timer clock: {}", timer_clock.as_ref().map(|tck| tck.freq().0).unwrap_or(0));
        let mut timer = device.TC3;
        init_timer(&mut timer, &mut device.PM, &timer_clock);
        init_eic(&mut device.EIC, &mut device.PM, &clocks.eic(&gclk0));
        init_evsys(&mut device.EVSYS, &mut device.PM, &clocks.evsys0(&gclk0));
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

    #[task(binds = TC3, priority=2, resources=[timer, dcf_pin, debug_pin])]
    fn tc3(cx: tc3::Context) {
        let timer: &mut pac::TC3 = cx.resources.timer;
        let counter = timer.count32();
        let (mc0, mc1) = (counter.cc[0].read().bits(), counter.cc[1].read().bits());
        let flags = counter.intflag.read();
        if flags.ovf().bit_is_set() {
            rprintln!("Overflow");
        }
        if flags.err().bit_is_set() {
            rprintln!("Error");
        }
        if flags.syncrdy().bit_is_set() {
            rprintln!("Sync ready");
        }
        if flags.mc0().bit_is_set() {
            rprintln!("MC0: {}", mc0);
        }
        if flags.mc1().bit_is_set() {
            rprintln!("MC1: {}", mc1);
        }
        counter.intflag.write(|w| {
            w.mc0().clear_bit();
            w.mc1().clear_bit();
            w
        });
    }
};
