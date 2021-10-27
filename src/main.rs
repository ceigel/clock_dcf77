#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m::peripheral::DWT;
use feather_f405::hal as stm32f4xx_hal;
use feather_f405::{hal::prelude::*, pac, setup_clocks};
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::gpio::{gpioa, gpioc, Alternate, Output, PushPull, AF2};
use stm32f4xx_hal::time::Hertz;

#[app(device = feather_f405::hal::stm32, monotonic = rtic::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        dcf_pin: gpioa::PA6<Alternate<2>>,
        debug_pin: gpioc::PC6<Output<PushPull>>,
        timer: pac::TIM3,
    }
    #[init(spawn=[])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
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
        let timer = device.TIM3;
        let clock_freq: Hertz = 10.khz().into();
        let psc = ((2 * clocks.pclk1().0) / clock_freq.0 as u32) as u16 - 1;

        timer.psc.write(|w| w.psc().bits(psc));
        timer.arr.write(|w| w.arr().bits(10_000));

        timer.cr1.modify(|_, w| w.urs().set_bit());
        timer.egr.write(|w| w.ug().set_bit());
        timer.cr1.modify(|_, w| w.urs().clear_bit());

        timer.ccmr1_input().write(|w| {
            w.cc1s().ti1();
            w.ic1f().fdts_div32_n8();
            w
        });

        timer.ccer.write(|w| {
            w.cc1p().set_bit();
            w.cc1np().clear_bit();
            w.cc1e().set_bit();
            w
        });
        timer.dier.write(|w| w.cc1ie().set_bit());
        timer.cr1.modify(|_, w| {
            w.cen().set_bit();
            w
        });
        rprintln!("Init successful");
        init::LateResources {
            dcf_pin: pin,
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

    #[task(binds = TIM3, priority=2, resources=[timer, dcf_pin, debug_pin])]
    fn tim3(cx: tim3::Context) {
        let timer: &mut pac::TIM3 = cx.resources.timer;
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
};
