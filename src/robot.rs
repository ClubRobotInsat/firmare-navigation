use crate::f103;
use crate::f103::{interrupt, Peripherals, SPI1, TIM2, TIM3, TIM4, USART3};
use crate::hal::delay::Delay;
use crate::hal::gpio::{
    gpioa::*, gpiob::*, gpioc::*, Alternate, Floating, Input, Output, PushPull,
};
use crate::hal::prelude::*;
use crate::hal::pwm::{Pwm, C3, C4};
use crate::hal::qei::Qei;
use crate::hal::serial::{Serial, Tx};
use crate::hal::spi::*;
use crate::hal::timer::Timer;
use crate::CortexPeripherals;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use librobot::navigation::Motor;
use qei::QeiManager; //  Stack frame for exception handling.

type QeiLeft =
    (qei::QeiManager<stm32f1xx_hal::qei::Qei<TIM4, (PB6<Input<Floating>>, PB7<Input<Floating>>)>>);

type QeiRight =
    (qei::QeiManager<stm32f1xx_hal::qei::Qei<TIM2, (PA0<Input<Floating>>, PA1<Input<Floating>>)>>);

type MotorLeft = Motor<Pwm<TIM3, C4>, PB8<Output<PushPull>>>;
type MotorRight = Motor<Pwm<TIM3, C3>, PB9<Output<PushPull>>>;

type SpiPins = (
    PA5<Alternate<PushPull>>,
    PA6<Input<Floating>>,
    PA7<Alternate<PushPull>>,
);

pub struct Robot<K, P> {
    pub spi_eth: Spi<K, P>,
    pub delay: Delay,
    pub led_black_pill: PC13<Output<PushPull>>,
    pub qei_left: QeiLeft,
    pub qei_right: QeiRight,
    pub cs: PB13<Output<PushPull>>,
    pub motor_right: MotorRight,
    pub motor_left: MotorLeft,
    pub max_duty: u16,
    pub debug: Tx<USART3>,
}

pub fn init_peripherals(chip: Peripherals, mut cortex: CortexPeripherals) -> Robot<SPI1, SpiPins> {
    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    // Config des horloges
    let mut rcc = chip.RCC.constrain();
    let mut flash = chip.FLASH.constrain();
    let mut afio = chip.AFIO.constrain(&mut rcc.apb2);

    cortex.DCB.enable_trace();
    cortex.DWT.enable_cycle_counter();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    //  Configuration des GPIOs
    let mut gpioa = chip.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = chip.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = chip.GPIOC.split(&mut rcc.apb2);

    // Configuration des PINS

    // Slave select, on le fixe à un état bas (on n'en a pas besoin, une seule communication)
    let mut cs = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    cs.set_low();

    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    {
        // Hardfault LED
        let mut pin = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        pin.set_low();
        // Blinking led
        let mut led = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
        led.set_low();
    }
    let led_black_pill = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let spi = Spi::spi1(
        chip.SPI1,
        (sclk, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let debug_usart = Serial::usart3(
        chip.USART3,
        (pb10, pb11),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb1,
    );

    let (debug_tx, _) = debug_usart.split();

    // Clignotement de la led
    let _ = Timer::tim1(chip.TIM1, 5.hz(), clocks, &mut rcc.apb2);

    // Config des QEI
    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let pb6 = gpiob.pb6; // floating input
    let pb7 = gpiob.pb7; // floating input
    let pa0 = gpioa.pa0; // floating input
    let pa1 = gpioa.pa1; // floating input
    let left_engine_dir_pb8 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let right_engine_dir_pb9 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

    let qei_right = QeiManager::new(Qei::tim2(
        chip.TIM2,
        (pa0, pa1),
        &mut afio.mapr,
        &mut rcc.apb1,
    ));
    let qei_left = QeiManager::new(Qei::tim4(
        chip.TIM4,
        (pb6, pb7),
        &mut afio.mapr,
        &mut rcc.apb1,
    ));

    // Config des PWM
    let (mut pwm_right_pb0, mut pwm_left_pb1) = chip.TIM3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        10000.hz(),
        clocks,
        &mut rcc.apb1,
    );

    pwm_right_pb0.enable();
    pwm_left_pb1.enable();

    let max_duty = pwm_right_pb0.get_max_duty();

    let motor_left = Motor::new(pwm_left_pb1, left_engine_dir_pb8);
    let motor_right = Motor::new(pwm_right_pb0, right_engine_dir_pb9);

    //  Create a delay timer from the RCC clocks.
    let delay = Delay::new(cortex.SYST, clocks);

    Robot {
        spi_eth: spi,
        delay,
        led_black_pill,
        cs,
        motor_right,
        qei_left,
        qei_right,
        motor_left,
        max_duty,
        debug: debug_tx,
    }
}

#[interrupt]
fn TIM1_UP() {
    static mut TOOGLE: bool = false;
    unsafe {
        if *TOOGLE {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.br14().set_bit());
        } else {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.bs14().set_bit());
        }
        *TOOGLE = !(*TOOGLE);
        (*f103::TIM3::ptr()).sr.write(|w| w.uif().clear_bit());
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    unsafe {
        (*f103::GPIOB::ptr()).bsrr.write(|w| w.br12().set_bit());
    }
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.

#[exception]
fn DefaultHandler(irqn: i16) {
    unsafe {
        (*f103::GPIOB::ptr()).bsrr.write(|w| w.br7().set_bit());
    }
    panic!("Unhandled exception (IRQn = {})", irqn);
}
