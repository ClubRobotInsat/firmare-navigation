//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

mod robot;

use crate::f103::Peripherals;
use crate::hal::stm32 as f103;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::serial::Write;
use hal::device::USART3;
use hal::serial::Tx;
use librobot::navigation::{Command, PIDParameters, RealWorldPid};
use librobot::transmission::eth::init_eth;
use librobot::units::MilliMeter;
use nb::block;
use numtoa::NumToA;
#[allow(unused_imports)]
use panic_semihosting;
use robot::init_peripherals;
use stm32f1xx_hal as hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
use w5500::*;

//  Black Pill starts execution at function main().

/// Envoie les informations de debug sur la pate PA9 du stm32 en utilisant le périphérique USART
///
/// Pour l'utiliser :
/// 1. Il faut relier PA9 au pin RX d'un module USART
/// 2. Il faut lancer `screen /dev/ttyUSB0 115200` (remplacer /dev/ttyUSB0 par le module USART)
/// 3. ????
///
/// Enjoy !
fn write_info(
    ser: &mut Tx<USART3>,
    qei_left: i64,
    qei_right: i64,
    command_left: Command,
    command_right: Command,
) {
    let mut buffer_0 = [0u8; 64];
    let mut buffer_1 = [0u8; 64];
    let mut buffer_2 = [0u8; 64];
    let mut buffer_3 = [0u8; 64];

    let qei_left_str = qei_left.numtoa(10, &mut buffer_0);
    let qei_right_str = qei_right.numtoa(10, &mut buffer_1);

    let command_left_str = command_left.get_value().numtoa(10, &mut buffer_2);
    let command_right_str = command_right.get_value().numtoa(10, &mut buffer_3);

    let str1 = b"QEI : ";
    let sep = b" | ";
    let ret = b"\n\r";
    let str2 = b"COMMAND : ";

    for b in str1
        .iter()
        .chain(qei_left_str.iter())
        .chain(sep.iter())
        .chain(qei_right_str.iter())
        .chain(ret.iter())
        .chain(str2.iter())
        .chain(command_left_str.iter())
        .chain(sep.iter())
        .chain(command_right_str.iter())
        .chain(ret.iter())
    {
        block!(ser.write(*b)).expect("Failed to send data");
    }
}

#[entry]
fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let mut robot = init_peripherals(chip, cortex);
    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.cs);
    init_eth(
        &mut eth,
        &mut robot.spi_eth,
        &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x05),
        &IpAddress::new(255, 255, 255, 0),
    );

    // Config du PID
    let pid_parameters = PIDParameters {
        coder_radius: MilliMeter(31),
        inter_axial_length: MilliMeter(223),
        pos_kp: 1.0,
        pos_kd: 1.0,
        orient_kp: 1.0,
        orient_kd: 1.0,
        max_output: robot.max_duty,
    };

    let mut pos_pid = RealWorldPid::new(robot.qei_left, robot.qei_right, &pid_parameters);

    // ==== Config de l'ethernet
    // ports pour la com
    // TODO changer pb12 en le vrai port qu'il faut utiliser
    /*let mut pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
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
             &IpAddress::new(192, 168, 0, 222));*/

    pos_pid.forward(MilliMeter(500));

    let _buffer = [0; 2048];

    loop {
        /*if let Some((_, _, size)) =
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
        }*/
        pos_pid.update();
        let (cmd_left, cmd_right) = pos_pid.get_command();
        let qeis = pos_pid.get_qei_ticks();
        write_info(&mut robot.debug, qeis.0, qeis.1, cmd_left, cmd_right);
        robot.motor_left.apply_command(cmd_left);
        robot.motor_right.apply_command(cmd_right);
        robot.delay.delay_ms(50u32);
    }
}
