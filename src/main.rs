//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use(entry, exception)] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate arrayvec;
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate embedded_hal;
extern crate librobot;
extern crate nb;
extern crate numtoa;
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate pid_control;
extern crate qei;
extern crate stm32f103xx;
extern crate stm32f103xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.

use core::fmt::Write;

use cortex_m::Peripherals as CortexPeripherals;

use bluepill_hal::prelude::*; //  Define HAL traits.
use bluepill_hal::qei::Qei;
use bluepill_hal::stm32f103xx as f103;
use bluepill_hal::stm32f103xx::Peripherals;
use bluepill_hal::time::Hertz;

use cortex_m::asm;
use cortex_m_semihosting::hio; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.

use qei::QeiManager;

use librobot::navigation::{Motor, RealWorldPid};
use librobot::units::MilliMeter;

//  Black Pill starts execution at function main().
entry!(main);

fn main() -> ! {
    let bluepill = Peripherals::take().unwrap();
    let _cortex = CortexPeripherals::take().unwrap();
    let mut debug_out = hio::hstdout().unwrap();

    // Config des horloges
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Config du GPIO
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = bluepill.GPIOA.split(&mut rcc.apb2);
    let pa0 = gpioa.pa0; // floating input
    let pa1 = gpioa.pa1; // floating input
    let mut pa9 = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

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
    let (mut pwm_right_pb0, mut pwm_left_pb1) = bluepill.TIM3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        Hertz(10000),
        clocks,
        &mut rcc.apb1,
    );
    pwm_left_pb1.enable();
    pwm_right_pb0.enable();
    let max_duty = pwm_right_pb0.get_max_duty();
    let mut motor_left = Motor::new(pwm_left_pb1, left_engine_dir_pb8);
    let mut motor_right = Motor::new(pwm_right_pb0, right_engine_dir_pb9);

    // Config du PID
    let pos_kp = 1.0;
    let pos_kd = 100.0;
    let orientation_kp = 1.0;
    let orientation_kd = 1.0;

    let mut pid = RealWorldPid::new(
        qei_left,
        qei_right,
        MilliMeter(31),
        MilliMeter(223),
        pos_kp,
        pos_kd,
        orientation_kp,
        orientation_kd,
        max_duty,
    );

    pid.forward(MilliMeter(50));

    let mut io: bool = false;

    loop {
        let (cmd_left, cmd_right) = pid.update();

        //pos_pid.print_qei_state(&mut debug_out);
        //write!(debug_out, "Left : {}, Right : {}\n", cmd_left, cmd_right).unwrap();

        motor_left.apply_command(cmd_left);
        motor_right.apply_command(cmd_right);
        if io {
            io = false;
            pa9.set_low();
        } else {
            io = true;
            pa9.set_high();
        }
    }
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
