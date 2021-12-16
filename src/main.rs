#![no_std]
#![no_main]

use panic_rtt_target as _;

use atsamd_hal as hal;
use atsamd_hal::pac;
use hal::clock::{ClockGenId, ClockSource, EicClock, Evsys0Clock, GenericClockController, Tcc0Tcc1Clock};
use hal::gpio::v2::pin::{self, Pin, Pins, PullUpInterrupt, PushPullOutput};
use pac::Peripherals;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

fn init_timer(timer: &mut pac::TCC0, pm: &mut pac::PM, tcc0_clock: &Option<Tcc0Tcc1Clock>) {
    tcc0_clock.as_ref().expect("To have tcc0 clock");
    pm.apbcmask.modify(|_, w| w.tcc0_().set_bit());

    timer.ctrla.write(|w| w.enable().clear_bit());
    while timer.syncbusy.read().enable().bit_is_set() {}

    timer.ctrla.write(|w| w.swrst().set_bit());
    while timer.syncbusy.read().swrst().bit_is_set() {}

    timer.ctrlbset.write(|w| {
        // timer up when the direction bit is zero
        w.dir().clear_bit();
        // Periodic
        w.oneshot().clear_bit()
    });

    timer.wave.write(|w| w.wavegen().nfrq());
    timer.per().write(|w| unsafe { w.per().bits(16_000) });
    timer.intenclr.write(|w| {
        w.ovf().set_bit();
        w
    });

    timer.intenset.write(|w| {
        w.mc0().set_bit();
        w.mc1().set_bit();
        w.err().set_bit();
        w
    });
    timer.ctrla.modify(|_, w| {
        w.prescaler().div64();
        w.cpten0().set_bit();
        w.cpten1().set_bit();
        w
    });
    timer.evctrl.write(|w| {
        w.mcei0().set_bit();
        w.mcei1().set_bit();
        w.evact1().pwp();
        w.tcei1().set_bit();
        w.tcinv1().set_bit();
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
        w.filten2().clear_bit();
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
    }
    #[init(spawn=[])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        rtt_init_done();
        let mut device: Peripherals = cx.device;
        let mut clocks = GenericClockController::with_external_32kosc(device.GCLK, &mut device.PM, &mut device.SYSCTRL, &mut device.NVMCTRL);
        let tc = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK3, 187, ClockSource::DFLL48M, true) // clock divider 64, 4000 counts/s, clock period: 16000
            .expect("To set peripherals clock");

        let pins = Pins::new(device.PORT);
        let dcf_pin: Pin<pin::PA02, PullUpInterrupt> = pins.pa02.into();

        // Use this pin for debugging decoded signal state with oscilloscope
        let output_pin = pins.pa17.into_push_pull_output();

        let timer_clock = clocks.tcc0_tcc1(&tc);
        rprintln!("Timer clock: {}", timer_clock.as_ref().map(|tck| tck.freq().0).unwrap_or(0));
        let mut timer = device.TCC0;
        init_timer(&mut timer, &mut device.PM, &timer_clock);
        rprintln!("Timer init done");
        init_eic(&mut device.EIC, &mut device.PM, &clocks.eic(&tc));
        rprintln!("EIC init done");
        init_evsys(&mut device.EVSYS, &mut device.PM, &clocks.evsys0(&tc));
        rprintln!("EVSYS init done");
        rprintln!("Init successful");
        init::LateResources {
            dcf_pin,
            debug_pin: output_pin,
            timer,
            eic: device.EIC,
            evsys: device.EVSYS,
        }
    }

    #[allow(clippy::empty_loop)]
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {}
    }

    #[task(binds = TCC0, priority=8, resources=[timer, dcf_pin, debug_pin])]
    fn tcc0(cx: tcc0::Context) {
        let timer: &mut pac::TCC0 = cx.resources.timer;
        rprintln!("Timer interrupt");
        let flags = timer.intflag.read();
        let cc0 = timer.cc()[0].read().cc().bits();
        let cc1 = timer.cc()[1].read().cc().bits();
        if flags.ovf().bit_is_set() {
            rprintln!("Overflow");
            timer.intflag.modify(|_, w| w.ovf().set_bit());
        }
        if flags.err().bit_is_set() {
            rprintln!("Error");
            timer.intflag.modify(|_, w| w.err().set_bit());
        }
        if flags.mc0().bit_is_set() {
            rprintln!("CC0: {}", cc0);
            timer.intflag.modify(|_, w| w.mc0().set_bit());
        }
        if flags.mc1().bit_is_set() {
            rprintln!("CC1: {}", cc1);
            timer.intflag.modify(|_, w| w.mc1().set_bit());
        }
    }
};
