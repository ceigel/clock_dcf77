#![no_std]
#![no_main]

use panic_rtt_target as _;

use core::fmt::{Display, Write};
use cortex_m::peripheral::DWT;
use feather_f405::hal as stm32f4xx_hal;
use feather_f405::{hal::prelude::*, pac, setup_clocks};
use rtic::app;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use stm32f4xx_hal::gpio::{gpioa, gpioc, Alternate, Output, PushPull, AF2};

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

impl Display for TickBuffer {
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
        if val >= self.counts.len() {
            val - self.counts.len()
        } else {
            val
        }
    }
}

fn init_timer(timer: pac::TIM3, clocks: &stm32f4xx_hal::rcc::Clocks) -> pac::TIM3 {
    let psc = ((2 * clocks.pclk1().0) / 7_999_u32) as u16 - 1;

    timer.psc.write(|w| w.psc().bits(psc));
    timer.arr.write(|w| w.arr().bits(7_999));

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
        ticks: TickBuffer,
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
        let timer = init_timer(device.TIM3, &clocks);
        rprintln!("Init successful");
        init::LateResources {
            dcf_pin: pin,
            debug_pin: output_pin,
            timer,
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

    #[task(binds = TIM3, priority=2, resources=[timer, dcf_pin, debug_pin, ticks, count, output2])]
    fn tim3(cx: tim3::Context) {
        let timer: &mut pac::TIM3 = cx.resources.timer;
        let flags = timer.sr.read();
        let fbits = flags.bits();
        let count: &mut u8 = cx.resources.count;
        if flags.cc1if().is_match_() {
            let c1 = timer.ccr1.read().bits();
            rprintln!("{} - {} : {:016b}", c1, *count, fbits);
            cx.resources.ticks.add_tick(c1 >> 3, Edge::Down);
            *count += 1;
            if *count == 60 {
                *count = 0;
                cx.resources.ticks.compute_phase();
                writeln!(cx.resources.output2, "{}", cx.resources.ticks).unwrap();
            }
        } else if flags.cc2if().is_match_() {
            let c2 = timer.ccr2.read().bits();
            cx.resources.ticks.add_tick(c2 >> 3, Edge::Up);
        }
        timer.sr.modify(|_, w| {
            w.uif().clear_bit();
            w.tif().clear_bit();
            w
        });
    }
};
