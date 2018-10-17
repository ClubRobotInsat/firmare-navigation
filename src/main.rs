//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use(entry, exception)] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate stm32f103xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
extern crate stm32f103xx;
extern crate arrayvec;
extern crate embedded_hal;
extern crate nb;
extern crate numtoa;
extern crate pid_control;
extern crate qei;

use cortex_m::Peripherals as CortexPeripherals;

use bluepill_hal::prelude::*; //  Define HAL traits.
use bluepill_hal::qei::Qei;
use bluepill_hal::stm32f103xx as f103;
use bluepill_hal::stm32f103xx::Peripherals;
use bluepill_hal::time::Hertz;
use cortex_m::asm;

use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.

use pid_control::{Controller, DerivativeMode, PIDController};

use qei::QeiManager;

type PIDControlleri = PIDController<i64>;

//  Black Pill starts execution at function main().
entry!(main);

fn main() -> ! {
    let bluepill = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();

    // Config des horloges
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let tim3 = bluepill.TIM3;
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Config du GPIO
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = bluepill.GPIOA.split(&mut rcc.apb2);
    let pa0 = gpioa.pa0; // floating input
    let pa1 = gpioa.pa1; // floating input

    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let pb6 = gpiob.pb6; // floating input
    let pb7 = gpiob.pb7; // floating input
    let left_engine_dir_pb8 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let right_engine_dir_pb9 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

    // Config des QEI
    let qei_right = QeiManager::new(Qei::tim2(
        bluepill.TIM2,
        (pa0, pa1),
        &mut afio.mapr,
        &mut rcc.apb1,
    ));
    let qei_left = QeiManager::new(Qei::tim4(
        bluepill.TIM4,
        (pb6, pb7),
        &mut afio.mapr,
        &mut rcc.apb1,
    ));

    // Config des PWM
    let (mut pwm_right_pb0, mut pwm_left_pb1) = tim3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        Hertz(10000),
        clocks,
        &mut rcc.apb1,
    );
    pwm_left_pb1.enable();
    pwm_right_pb0.enable();

    {
        // K0 = 0.000004
        let mut pid: PIDController<i64> = PIDControlleri::new(1, 100, 100);
        pid.out_min = -65535;
        pid.out_max = 65535;
        pid.i_min = -5;
        pid.i_max = 5;
        pid.set_target(5632);
        pid.d_mode = DerivativeMode::OnError;
    }

    loop {}
}

//  For any hard faults, show a message on the debug console and stop.
exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.
exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
