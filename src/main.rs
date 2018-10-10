//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use(entry, exception)] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
#[macro_use]
extern crate stm32f103xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
#[macro_use]
extern crate stm32f103xx;
#[macro_use]
extern crate nb;
extern crate embedded_hal;
extern crate pid_control;
extern crate qei;

use cortex_m::Peripherals as CortexPeripherals;

use bluepill_hal::delay::Delay; //  Delay timer.
use bluepill_hal::prelude::*;   //  Define HAL traits.
use bluepill_hal::qei::Qei;
use bluepill_hal::serial::Event::Rxne;
use bluepill_hal::serial::Serial;
use bluepill_hal::stm32f103xx as f103;
use bluepill_hal::stm32f103xx::Peripherals;
use bluepill_hal::time::{Bps, Hertz, KiloHertz};
use cortex_m::asm;

use core::fmt::Write; //  Provides writeln() function for debug console output.
use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.
use cortex_m_semihosting::hio; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

use embedded_hal::Direction;

use f103::Interrupt;

use qei::QeiManager;

//  Black Pill starts execution at function main().
entry!(main);

type QeiPins = (
    bluepill_hal::gpio::gpiob::PB6<bluepill_hal::gpio::Input<bluepill_hal::gpio::Floating>>,
    bluepill_hal::gpio::gpiob::PB7<bluepill_hal::gpio::Input<bluepill_hal::gpio::Floating>>,
);

static mut QEIM: Option<QeiManager<Qei<bluepill_hal::stm32f103xx::TIM4, QeiPins>>> = None;

fn tim2_interrupt() {
    // Clear interrupt pending flag;
    unsafe {
        (*f103::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit());
    };
    unsafe {
        QEIM.as_mut().unwrap().sample_unwrap();
    }
}

//  Black Pill starts execution here. "-> !" means this function will never return (because of the loop).
fn main() -> ! {
    //  Show "Hello, world!" on the debug console, which is shown in OpenOCD. "mut" means that this object is mutable, i.e. it can change.
    let mut debug_out = hio::hstdout().unwrap();
    writeln!(debug_out, "Initializeing peripherals...").unwrap();

    //  Get peripherals (clocks, flash memory, GPIO) for the STM32 Black Pill microcontroller.
    let bluepill = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();

    let mut nvic = cortex.NVIC;
    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let tim3 = bluepill.TIM3;
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    //  Get GPIO Port C, which also enables the Advanced Peripheral Bus 2 (APB2) clock for Port C.
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    //let mut gpioa = bluepill.GPIOA.split(&mut rcc.apb2);

    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);

    let pb6 = gpiob.pb6;
    let pb7 = gpiob.pb7;
    let mut tim2 =
        bluepill_hal::timer::Timer::tim2(bluepill.TIM2, 1000.hz(), clocks, &mut rcc.apb1);
    let qei = Qei::tim4(bluepill.TIM4, (pb6, pb7), &mut afio.mapr, &mut rcc.apb1);
    unsafe {
        QEIM = Some(QeiManager::new(qei));
    }

    //  Create a delay timer from the RCC clocks.
    let _delay = Delay::new(cortex.SYST, clocks);
    let (mut pwm_pb0, _) = tim3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        Hertz(10000),
        clocks,
        &mut rcc.apb1,
    );

    pwm_pb0.enable();
    nvic.enable(Interrupt::TIM2);
    tim2.listen(bluepill_hal::timer::Event::Update);
    loop {
        writeln!(debug_out, "Count : {}", unsafe {
            QEIM.as_mut().unwrap().count()
        })
        .unwrap();
    }
}

interrupt!(TIM2, tim2_interrupt);

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
