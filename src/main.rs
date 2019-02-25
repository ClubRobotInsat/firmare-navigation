//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

mod robot;

use crate::f103::Peripherals;
use crate::hal::stm32 as f103;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
use librobot::navigation::{PIDParameters, Pid, RobotConstants};
use librobot::transmission::eth::init_eth;
use librobot::units::MilliMeter;
#[allow(unused_imports)]
use panic_semihosting;
use robot::init_peripherals;
use stm32f1xx_hal as hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
use w5500::*;

//  Black Pill starts execution at function main().

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
        max_output: robot.max_duty / 4,
    };
    let robot_constants = RobotConstants {
        coder_radius: MilliMeter(31),
        inter_axial_length: MilliMeter(223),
    };

    let mut pos_pid = Pid::new(
        robot.qei_left,
        robot.qei_right,
        &pid_parameters,
        robot_constants,
    );

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

    //pos_pid.forward(MilliMeter(50));

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

        // permet d'afficher les valeurs des qei pour le debug
        //write!(debug_out, "{:?}\n", pos_pid).unwrap();

        // Permet d'afficher les valeurs des commandes moteur pour le debug
        //write!(debug_out, "Left : {}, Right : {}\n", cmd_left, cmd_right).unwrap();

        robot.motor_left.apply_command(cmd_left);
        robot.motor_right.apply_command(cmd_right);
    }
}
