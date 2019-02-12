//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.
#[macro_use]
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
extern crate stm32f1xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
use crate::bluepill_hal::stm32 as f103;

use core::fmt::Write;

use cortex_m::Peripherals as CortexPeripherals;

use crate::bluepill_hal::prelude::*; //  Define HAL traits.
use crate::bluepill_hal::qei::Qei;
use crate::bluepill_hal::serial::Serial;
use crate::f103::Peripherals;
use crate::bluepill_hal::time::Hertz;
use crate::bluepill_hal::spi::*;
use crate::f103::{SPI1, USART3};

use cortex_m::asm;
use cortex_m_semihosting::hio; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.

use qei::QeiManager;

use w5500::*;

use librobot::navigation::{NavigationFrame, Motor, RealWorldPid, PIDParameters, Command};
use librobot::transmission::eth::{init_eth, SOCKET_UDP};
use librobot::units::MilliMeter;

mod PWM;

//  Black Pill starts execution at function main().

#[entry]
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

    {
        let mut pa8 = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
        pa8.set_high();
    }

    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let pb6 = gpiob.pb6; // floating input
    let pb7 = gpiob.pb7; // floating input
    let _pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let _pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);
    let left_engine_dir_pb8 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let right_engine_dir_pb9 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

    // Config des QEI
    let qei_right =
        QeiManager::new(Qei::tim2(bluepill.TIM2, (pa0, pa1), &mut afio.mapr, &mut rcc.apb1));
    let qei_left =
        QeiManager::new(Qei::tim4(bluepill.TIM4, (pb6, pb7), &mut afio.mapr, &mut rcc.apb1));

    // Config des PWM
    let (mut pwm_right_pb0, mut pwm_left_pb1) = bluepill
        .TIM3
        .pwm((pb0, pb1),
             &mut afio.mapr,
             Hertz(10000),
             clocks,
             &mut rcc.apb1);
    pwm_right_pb0.enable();
    pwm_left_pb1.enable();
    let max_duty = pwm_right_pb0.get_max_duty();

    let mut motor_left = Motor::new(pwm_left_pb1, left_engine_dir_pb8);
    let mut motor_right = Motor::new(pwm_right_pb0, right_engine_dir_pb9);

    // Config du PID
    let pid_parameters = PIDParameters {
        coder_radius: MilliMeter(31),
        inter_axial_length: MilliMeter(223),
        pos_kp: 1.0,
        pos_kd: 1.0,
        orient_kp: 1.0,
        orient_kd: 1.0,
        max_output: max_duty / 4,
    };

    let mut pos_pid = RealWorldPid::new(qei_left, qei_right, &pid_parameters);

    // ==== Config de l'ethernet
    // ports pour la com
    // TODO changer pb12 en le vrai port qu'il faut utiliser
    let mut pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    pb12.set_low();

    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    // spi
    let mut spi_eth = Spi::spi1(bluepill.SPI1,
                                (sclk, miso, mosi),
                                &mut afio.mapr,
                                Mode {
                                    polarity: Polarity::IdleLow,
                                    phase: Phase::CaptureOnFirstTransition,
                                },
                                1.mhz(),
                                clocks,
                                &mut rcc.apb2);

    // init w5500
    let mut eth = W5500::new(&mut spi_eth, &mut pb12);
    init_eth(&mut eth,
             &mut spi_eth,
             &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x05),
             &IpAddress::new(192, 168, 0, 222));

    //pos_pid.forward(MilliMeter(50));

    let mut buffer = [0; 2048];

    loop {
        if let Some((_, _, size)) =
            eth.try_receive_udp(&mut spi_eth, SOCKET_UDP, &mut buffer)
                .unwrap() {
            let _id = buffer[0];
            match NavigationFrame::from_json_slice(&buffer[1..size]) {
                Ok(_frame) => {
                    //write!(debug_out, "{:?}", servo.to_string::<U256>().unwrap()).unwrap();
                    // Do something
                }
                Err(e) => panic!("{:#?}", e),
            }
        }

        let (cmd_left, cmd_right) = pos_pid.update();
        //pos_pid.print_qei_state(&mut debug_out);

        // permet d'afficher les valeurs des qei pour le debug
        let tics = RealWorldPid::get_qei_ticks(&mut pos_pid);
        write!(debug_out, "Left : {}, Right : {}\n", tics.0, tics.1).unwrap();

        // Permet d'afficher les valeurs des commandes moteur pour le debug
        //write!(debug_out, "Left : {}, Right : {}\n", cmd_left, cmd_right).unwrap();

        motor_left.apply_command(cmd_left);
        motor_right.apply_command(cmd_right);
    }
}

//  For any hard faults, show a message on the debug console and stop.

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.

#[exception]
fn DefaultHandler(irqn: i16) {
    asm::bkpt();
    panic!("Unhandled exception (IRQn = {})", irqn);
}
