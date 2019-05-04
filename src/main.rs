//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

mod robot;

use crate::f103::{interrupt, Peripherals, SPI1};
use crate::hal::spi::Spi;
use crate::hal::stm32 as f103;
use core::f32;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::serial::Write;
use embedded_hal::spi::FullDuplex;
use embedded_hal::Qei;
use hal::device::USART3;
use hal::serial::Tx;
use heapless::consts::U2048;
use librobot::navigation::{Command, Coord, PIDParameters, RealWorldPid};
use librobot::transmission::eth::{init_eth, listen_on, SOCKET_UDP};
use librobot::transmission::id::{ELEC_LISTENING_PORT, ID_NAVIGATION, INFO_LISTENING_PORT};
use librobot::transmission::navigation::{NavigationCommand, NavigationFrame};
use librobot::transmission::Jsonizable;
use librobot::units::MilliMeter;
use nb::block;
use numtoa::NumToA;
#[allow(unused_imports)]
use panic_semihosting;
use robot::init_peripherals;
use stm32f1xx_hal as hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
use w5500::*;

// use librobot::transmission::MessageKind::Navigation;

use crate::robot::SpiPins;
use crate::robot::{MotorLeft, MotorRight, QeiLeft, QeiRight, Robot};
use core::ptr::null_mut;
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

static BLOCKED_COUNTER_THRESHOLD:u16 = 10;
static MOVING_DONE_COUNTER_THRESHOLD:u16 = 10;

#[derive(Clone, Copy)]
struct NavigationState {
    // Command
    command: NavigationCommand,
    counter: u16,

    // Position
    position: Coord,
    angle: i64,

    // Precision
    lin_accuracy: f32,
    ang_accuracy: f32,

    asserv_active: bool,
    blocked_counter: u16,
    moving_done_counter: u16,
}

impl NavigationState {
    fn inc_blocked(&mut self) {
        if self.blocked_counter < BLOCKED_COUNTER_THRESHOLD * 2 {
            self.blocked_counter += 1;
        }
    }

    fn dec_blocked(&mut self) {
        if self.blocked_counter < 3 {
            self.blocked_counter = 0;
        }
        else {
            self.blocked_counter -= 1;
        }
    }

    // On incremente et quand le compteur arrive à MOVING_DONE_COUNTER_THRESHOLD le mouvement est
    // terminé.
    fn inc_moving_done(&mut self) {
        if self.moving_done_counter < MOVING_DONE_COUNTER_THRESHOLD {
            self.moving_done_counter += 1;
        }
    }

    fn dec_moving_done(&mut self) {
        if self.moving_done_counter < 3 {
            self.moving_done_counter = 0;
        }
        else if self.moving_done_counter < MOVING_DONE_COUNTER_THRESHOLD {
            self.moving_done_counter -= 1;
        }
    }
}

fn send_navigation_state<T, K>(
    spi: &mut Spi<T, K>,
    eth: &mut W5500,
    nav_state: &NavigationState,
) where
    Spi<T, K>: FullDuplex<u8>,
{
    let frame = NavigationFrame {
        angle: nav_state.angle as i32 * 10,
        x: nav_state.position.x.as_millimeters() as i32 * 10,
        y: nav_state.position.y.as_millimeters() as i32 * 10,

        blocked: nav_state.blocked_counter >= BLOCKED_COUNTER_THRESHOLD,
        moving_done: nav_state.moving_done_counter >= MOVING_DONE_COUNTER_THRESHOLD,

        // read-only
        command: nav_state.command,
        counter: nav_state.counter,

        asserv_on_off: nav_state.asserv_active,
        reset: false,
        led: true,
        args_cmd1: 0,
        args_cmd2: 0,

        max_lin_speed: 0,
        max_ang_speed: 0,
        lin_accuracy: 0,
        ang_accuracy: 0,
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
    arg1: u16,
    arg2: u16,
) where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    match command {
        NavigationCommand::GoForward => {
            pos_pid.forward(arg1 as f32 / 10.0);
        }
        NavigationCommand::GoBackward => {
            pos_pid.backward(arg1 as f32 / 10.0);
        }
        NavigationCommand::TurnAbsolute => {
            let current_angle = pos_pid.get_angle() as f32;
            let mut diff = arg1 as f32 / 10.0 - current_angle;

            // Find the best angle
            let pi = core::f32::consts::PI;
            while diff < -pi {
                diff += pi * 2.0;
            }

            while diff >= pi {
                diff -= pi * 2.0;
            }

            pos_pid.rotate(diff);
        }
        NavigationCommand::TurnRelative => {
            let multiplier = if arg2 == 1 { -1.0 } else { 1.0 };
            pos_pid.rotate(multiplier * arg1 as f32 / 10.0);
        }
        NavigationCommand::EmergencyStop => {}
        NavigationCommand::Stop => {
            pos_pid.stop();
        }
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
        coder_radius: 31.0,
        ticks_per_turn: 4096,
        left_wheel_coef: 1.0,
        right_wheel_coef: -1.0,
        inter_axial_length: 296.0,
        pos_kp: 30.0,
        pos_kd: 0.0,
        orient_kp: 30.0,
        orient_kd: 0.0,
        max_output: robot.max_duty / 4,
    };

    let mut pos_pid = RealWorldPid::new(robot.qei_left, robot.qei_right, &pid_parameters);

    let mut nav_state = NavigationState {
        command: NavigationCommand::DoNothing,
        counter: 0,
        position: pos_pid.get_position(),
        angle: pos_pid.get_angle(),
        lin_accuracy: 0.0,
        ang_accuracy: 0.0,
        asserv_active: true,
        blocked_counter: 0,
        moving_done_counter: 0,
    };

    // ==== Config de l'ethernet

    // init w5500
    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.cs);
    init_eth(
        &mut eth,
        &mut robot.spi_eth,
        &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, ID_NAVIGATION as u8),
        &IpAddress::new(192, 168, 1, ID_NAVIGATION as u8),
    );

    listen_on(
        &mut eth,
        &mut robot.spi_eth,
        ELEC_LISTENING_PORT + ID_NAVIGATION,
        SOCKET_UDP,
    );

    // ==== Autorisation du timer

    unsafe {
        motor_left_ptr = &mut robot.motor_left as *mut MotorLeft;
        motor_right_ptr = &mut robot.motor_right as *mut MotorRight;
        pid_ptr = &mut pos_pid as *mut RealWorldPid<QeiLeft, QeiRight>;
        nav_state_ptr = &mut nav_state as *mut NavigationState;
        enabled = true;
    }

    let mut buffer = [0; 2048];

    loop {
        if let Ok(Some((_, _, size))) =
            eth.try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer)
        {
            /*for b in buffer.iter() {
                block!(robot.debug.write(*b)).expect("Failed to send data");
            }

            let mut buffer_4 = [0u8; 64];
            for b in size.numtoa(10, &mut buffer_4).iter() {
                block!(robot.debug.write(*b)).expect("Failed to send data");
            }*/

            match NavigationFrame::from_json_slice(&buffer[0..size]) {
                Ok(frame) => {
                    unsafe {
                        enabled = false;
                    }

                    if frame.counter != nav_state.counter {
                        nav_state.counter = frame.counter;
                        nav_state.command = frame.command;

                        nav_state.blocked_counter = 0;
                        nav_state.moving_done_counter = 0;

                        exec_command(
                            &mut pos_pid,
                            frame.command,
                            frame.args_cmd1,
                            frame.args_cmd2,
                        );
                    }
                }
                Err(e) => panic!("{:#?}", e),
            }
            let nav_state_copy = nav_state;
            let qeis = pos_pid.get_qei_ticks();
            let (cmd_left, cmd_right) = pos_pid.get_command();

            unsafe {
                enabled = true;
            }

            send_navigation_state(&mut robot.spi_eth, &mut eth, &nav_state_copy);

            write_info(
                &mut robot.debug,
                qeis.0,
                -qeis.1,
                cmd_left,
                cmd_right,
                nav_state_copy.position,
            );
        }
    }
}

#[allow(non_upper_case_globals)]
static mut pid_ptr: *mut RealWorldPid<QeiLeft, QeiRight> = null_mut();
#[allow(non_upper_case_globals)]
static mut nav_state_ptr: *mut NavigationState = null_mut();
#[allow(non_upper_case_globals)]
static mut motor_left_ptr: *mut MotorLeft = null_mut();
#[allow(non_upper_case_globals)]
static mut motor_right_ptr: *mut MotorRight = null_mut();

#[allow(non_upper_case_globals)]
static mut enabled: bool = false;

#[interrupt]
fn TIM1_UP() {
    static mut COUNTER: u16 = 0;

    unsafe {
        (*f103::TIM1::ptr()).sr.write(|w| w.uif().clear_bit());

        // Blinking led
        if *COUNTER >= 200 {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.br13().set_bit());
        } else {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.bs13().set_bit());
        }
        *COUNTER = (*COUNTER + 1) % 400;

        // PID
        if enabled {
            (*pid_ptr).update();

            // Check if robot is blocked / has terminated its movement
            if (*pid_ptr).is_robot_blocked(10, 0.1) {
                (*nav_state_ptr).inc_blocked();
            }
            else {
                (*nav_state_ptr).dec_blocked();
            }

            if (*pid_ptr).is_goal_reached((*nav_state_ptr).lin_accuracy, (*nav_state_ptr).ang_accuracy) {
                (*nav_state_ptr).inc_moving_done();
            }
            else {
                (*nav_state_ptr).dec_moving_done();
            }


            // Update command
            let (cmd_left, cmd_right) = (*pid_ptr).get_command();

            if (*nav_state_ptr).command == EmergencyStop || !(*nav_state_ptr).asserv_active {
                (*motor_left_ptr).apply_command(Command::Front(0));
                (*motor_right_ptr).apply_command(Command::Front(0));
                (*pid_ptr).stop();
            } else {
                (*motor_left_ptr).apply_command(cmd_left);
                (*motor_right_ptr).apply_command(cmd_right);
            }

            (*nav_state_ptr).position = (*pid_ptr).get_position();
            (*nav_state_ptr).angle = (*pid_ptr).get_angle();
        }
    }
}
