//! Compile-time check that the selected G474 PAC exposes the reserved resources.
pub fn peripheral_presence(p: &stm32g4xx_hal::stm32::Peripherals) {
    let _ = (
        &p.TIM1, &p.TIM2, &p.TIM3, &p.TIM4, &p.TIM5, &p.TIM8, &p.TIM15, &p.TIM16, &p.TIM17,
        &p.TIM20, &p.DMA1, &p.DMA2, &p.DMAMUX, &p.USART1, &p.SPI2, &p.DAC1, &p.IWDG,
    );
}

/// Leave global output permission low before enabling/configuring a watchdog.
/// This carrier gate must inhibit line drivers, valve outputs, analog run and brake
/// release independently of SPI writes. It is not the hardwired SR17 STO chain.
pub fn force_safe(p: &stm32g4xx_hal::stm32::Peripherals) {
    // PB3 reset through BSRR; requires carrier GPIO clock/configuration at boot.
    p.GPIOB
        .bsrr()
        .write(|w| unsafe { w.bits(1u32 << (3 + 16)) });
    p.TIM15.ccer().write(|w| unsafe { w.bits(0) });
    p.TIM16.ccer().write(|w| unsafe { w.bits(0) });
    p.TIM17.ccer().write(|w| unsafe { w.bits(0) });
    p.TIM20.ccer().write(|w| unsafe { w.bits(0) });
    p.TIM5.ccer().write(|w| unsafe { w.bits(0) });
    p.DAC1.dhr12r1().write(|w| unsafe { w.bits(0) });
}

/// IWDG /4, reload 159: 18.82–21.70 ms with DS12288 table 45 LSI limits.
/// Only call during explicit carrier initialization with all output gates low.
/// Once started IWDG cannot be disabled except by reset. Returns false on timeout.
pub fn start_watchdog(p: &stm32g4xx_hal::stm32::Peripherals) -> bool {
    force_safe(p);
    p.IWDG.kr().write(|w| unsafe { w.bits(0xcccc) });
    p.IWDG.kr().write(|w| unsafe { w.bits(0x5555) });
    p.IWDG.pr().write(|w| unsafe { w.bits(0) });
    p.IWDG.rlr().write(|w| unsafe { w.bits(159) });
    for _ in 0..100_000 {
        if p.IWDG.sr().read().bits() == 0 {
            p.IWDG.kr().write(|w| unsafe { w.bits(0xaaaa) });
            return true;
        }
    }
    false
}
/// The foreground validated supervisor is the sole owner of watchdog refresh.
/// Do not call from a timer ISR, UART ISR or unconditional main-loop heartbeat.
pub fn apply_guard(
    p: &stm32g4xx_hal::stm32::Peripherals,
    output: robotkit_device_protocol::router::SafeOutputs,
) {
    if !output.enables || !output.step_dma {
        force_safe(p);
    }
    if output.feed_iwdg {
        p.IWDG.kr().write(|w| unsafe { w.bits(0xaaaa) });
    }
}
