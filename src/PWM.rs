pub mod pwm {
    extern crate stm32f103xx_hal as bluepill_hal;
    extern crate cortex_m;
    extern crate cortex_m_rt;

    use cortex_m::Peripherals as CortexPeripherals;

    use bluepill_hal::prelude::*; //  Define HAL traits.
    use bluepill_hal::stm32f103xx as f103;
    use bluepill_hal::stm32f103xx::Peripherals;
    use bluepill_hal::time::Hertz;

    use librobot;
    use embedded_hal;

    pub fn init_pwm(){
    }
}