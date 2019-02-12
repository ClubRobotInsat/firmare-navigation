pub mod pwm {
    extern crate stm32f1xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
    extern crate cortex_m;
    extern crate cortex_m_rt;

    use crate::cortex_m::Peripherals as CortexPeripherals;
    use crate::bluepill_hal::stm32 as f103;

    use crate::bluepill_hal::prelude::*; //  Define HAL traits.
    use crate::bluepill_hal::qei::Qei;
    use crate::bluepill_hal::serial::Serial;
    use crate::f103::Peripherals;
    use crate::bluepill_hal::time::Hertz;

    use librobot;
    use embedded_hal;

    pub fn init_pwm() {}
}
