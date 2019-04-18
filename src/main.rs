//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

mod robot;

use crate::f103::Peripherals;
use crate::f103::SPI1;
use crate::hal::stm32 as f103;
use crate::hal::spi::Spi;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::serial::Write;
use embedded_hal::spi::FullDuplex;
use embedded_hal::Qei;
use hal::device::USART3;
use hal::serial::Tx;
use librobot::navigation::{Command, PIDParameters, RealWorldPid, Coord};
use librobot::transmission::navigation::{NavigationFrame, NavigationCommand};
use librobot::transmission::Jsonizable;
use librobot::transmission::eth::{ init_eth, listen_on, SOCKET_UDP };
use librobot::units::MilliMeter;
use nb::block;
use numtoa::NumToA;
#[allow(unused_imports)]
use panic_semihosting;
use robot::init_peripherals;
use stm32f1xx_hal as hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
use w5500::*;
use heapless::consts::U2048;
use librobot::transmission::id::{ID_NAVIGATION, ELEC_LISTENING_PORT, INFO_LISTENING_PORT};
// use librobot::transmission::MessageKind::Navigation;

use crate::robot::Robot;
use crate::robot::SpiPins;
use librobot::transmission::navigation::NavigationCommand::EmergencyStop;

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
    position: Coord,
) {
    let mut buffer_0 = [0u8; 64];
    let mut buffer_1 = [0u8; 64];
    let mut buffer_2 = [0u8; 64];
    let mut buffer_3 = [0u8; 64];
    let mut buffer_4 = [0u8; 64];
    let mut buffer_5 = [0u8; 64];

    let qei_left_str = qei_left.numtoa(10, &mut buffer_0);
    let qei_right_str = qei_right.numtoa(10, &mut buffer_1);

    let command_left_str = command_left.get_value().numtoa(10, &mut buffer_2);
    let command_right_str = command_right.get_value().numtoa(10, &mut buffer_3);

    let position_x_str = position.x.as_centimeters().numtoa(10, &mut buffer_4);
    let position_y_str = position.y.as_centimeters().numtoa(10, &mut buffer_5);

    let str1 = b"QEI : ";
    let sep = b" | ";
    let ret = b"\n\r";
    let str2 = b"COMMAND : ";
    let str3 = b"POSITION : ";
    let cm = b"cm";

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
        .chain(str3.iter())
        .chain(position_x_str.iter())
        .chain(cm.iter())
        .chain(sep.iter())
        .chain(position_y_str.iter())
        .chain(cm.iter())
        .chain(ret.iter())
    {
        block!(ser.write(*b)).expect("Failed to send data");
    }
}

struct NavigationState {
    command: NavigationCommand,
    counter: u16,
    blocked: bool,
    moving_done: bool,
}

fn send_navigation_state<T, K, L, R>(
    spi: &mut Spi<T, K>,
    eth: &mut W5500,
    pos_pid : &RealWorldPid<L, R>,
    nav_state: &NavigationState,
) where
    Spi<T, K>: FullDuplex<u8>,
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    let angle = pos_pid.get_angle();
    let position = pos_pid.get_position();

    let frame = NavigationFrame {
        angle: angle as i32 * 10,
        x: position.x.as_millimeters() as i32 * 10,
        y: position.y.as_millimeters() as i32 * 10,

        blocked: nav_state.blocked,
        moving_done: nav_state.moving_done,

        // read-only
        command: nav_state.command,
        counter: nav_state.counter,

        asserv_on_off: true,
        reset: true,
        led: true,
        args_cmd1: 0,
        args_cmd2: 0,
    };

    if let Ok(data) = frame.to_string::<U2048>() {
        if let Ok(_) = eth.send_udp(
            spi,
            SOCKET_UDP,
            ELEC_LISTENING_PORT + ID_NAVIGATION,
            &IpAddress::new(192, 168, 1, 254),
            INFO_LISTENING_PORT + ID_NAVIGATION,
            &data.as_bytes(),
        ) {
            // robot.led_feedback.set_low();
        } else {
            // robot.led_feedback.set_high();
        }
    }
}

fn exec_command<L, R>(
    pos_pid: &mut RealWorldPid<L, R>,
    command: NavigationCommand,
    arg1 :u16, _arg2: u16
) where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    match command {
        NavigationCommand::GoForward => {
            pos_pid.forward(MilliMeter(arg1 as i64 / 10));
        },
        NavigationCommand::GoBackward => {
            pos_pid.backward(MilliMeter(arg1 as i64 / 10));
        },
        NavigationCommand::TurnAbsolute => {
            let current_angle = pos_pid.get_angle();
            let diff = arg1 as i64 / 10 - current_angle;

            pos_pid.rotate(diff);

            // TODO ensure minimal rotation
            if diff < 0 {

            }
            else {

            }
        },
        NavigationCommand::TurnRelative => {
            pos_pid.rotate(arg1 as i64 / 10);
        },
        NavigationCommand::EmergencyStop => { },
        NavigationCommand::Stop => {
            pos_pid.stop();
        },
        NavigationCommand::DoNothing => (),
    }
}

#[entry]
fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let mut robot: Robot<SPI1, SpiPins> = init_peripherals(chip, cortex);

    // Config du PID
    let pid_parameters = PIDParameters {
        coder_radius: MilliMeter(31),
        ticks_per_turn: 4096,
        left_wheel_coef: 1.0,
        right_wheel_coef: -1.0,
        inter_axial_length: MilliMeter(223),
        pos_kp: 1.0,
        pos_kd: 0.0,
        orient_kp: 1.0,
        orient_kd: 1.0,
        max_output: robot.max_duty / 4,
    };

    let mut nav_state = NavigationState {
        command: NavigationCommand::DoNothing,
        counter: 0,
        blocked: false,
        moving_done: true,
    };

    let mut pos_pid = RealWorldPid::new(robot.qei_left, robot.qei_right, &pid_parameters);

    // ==== Config de l'ethernet

    // init w5500
    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.cs);
    init_eth(&mut eth,
             &mut robot.spi_eth,
             &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, ID_NAVIGATION as u8),
             &IpAddress::new(192, 168, 1, ID_NAVIGATION as u8));

    listen_on(&mut eth, &mut robot.spi_eth, ELEC_LISTENING_PORT + ID_NAVIGATION, SOCKET_UDP);

    let mut buffer = [0; 2048];

    let mut i = 0;

    loop {
        if let Ok(Some((_, _, size))) =
            eth.try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer) {

            /*for b in buffer.iter() {
                block!(robot.debug.write(*b)).expect("Failed to send data");
            }

            let mut buffer_4 = [0u8; 64];
            for b in size.numtoa(10, &mut buffer_4).iter() {
                block!(robot.debug.write(*b)).expect("Failed to send data");
            }*/

            match NavigationFrame::from_json_slice(&buffer[0..size]) {
                Ok(frame) => {

                    if frame.counter != nav_state.counter {
                        nav_state.counter = frame.counter;
                        nav_state.command = frame.command;

                        exec_command(&mut pos_pid, frame.command, frame.args_cmd1, frame.args_cmd2);
                    }
                }
                Err(e) => panic!("{:#?}", e),
            }

            send_navigation_state(&mut robot.spi_eth, &mut eth, &pos_pid, &nav_state);
        }

        pos_pid.update();
        let (cmd_left, cmd_right) = pos_pid.get_command();
        let qeis = pos_pid.get_qei_ticks();
        let coords = pos_pid.get_position();

        if nav_state.command == EmergencyStop {
            robot.motor_left.apply_command(Command::Front(0));
            robot.motor_right.apply_command(Command::Front(0));
        }
        else {
            robot.motor_left.apply_command(cmd_left.invert());
            robot.motor_right.apply_command(cmd_right.invert());
        }

        i += 1;

        if i % 1000 == 0 {
            /*write_info(
                &mut robot.debug,
                qeis.0,
                -qeis.1,
                cmd_left.invert(),
                cmd_right,
                coords,
            );*/
            i = 0;
        }
    }
}
