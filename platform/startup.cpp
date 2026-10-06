// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>
#include <cstddef>

extern "C"
{

// symbols exported in linker script
extern uint32_t _estack[];
extern uint32_t _sidata[], _sdata[], _edata[];
extern uint32_t _siccmram[], _sccmram[], _eccmram[];
extern uint32_t _sbss[], _ebss[];

extern void SystemInit();
// NOLINTNEXTLINE
extern void __libc_init_array();
extern int main();

}

using IrqHandlerPfn = void(*)();

extern "C" void Default_Handler()
{
    while (true)
        __asm volatile("nop");
}

#define WEAK_DEFAULT_HANDLER(name) \
    extern "C" [[gnu::weak, gnu::alias("Default_Handler")]] void name();

// Note: this list must be strictly ordered
#define IRQ_SYM_LIST(I, N) \
    I(NMI_Handler) \
    I(HardFault_Handler) \
    I(MemManage_Handler) \
    I(BusFault_Handler) \
    I(UsageFault_Handler) \
    N() \
    N() \
    N() \
    N() \
    I(SVC_Handler) \
    I(DebugMon_Handler) \
    N() \
    I(PendSV_Handler) \
    I(SysTick_Handler) \
    I(WWDG_IRQHandler) \
    I(PVD_PVM_IRQHandler) \
    I(RTC_TAMP_LSECSS_IRQHandler) \
    I(RTC_WKUP_IRQHandler) \
    I(FLASH_IRQHandler) \
    I(RCC_IRQHandler) \
    I(EXTI0_IRQHandler) \
    I(EXTI1_IRQHandler) \
    I(EXTI2_IRQHandler) \
    I(EXTI3_IRQHandler) \
    I(EXTI4_IRQHandler) \
    I(DMA1_Channel1_IRQHandler) \
    I(DMA1_Channel2_IRQHandler) \
    I(DMA1_Channel3_IRQHandler) \
    I(DMA1_Channel4_IRQHandler) \
    I(DMA1_Channel5_IRQHandler) \
    I(DMA1_Channel6_IRQHandler) \
    I(DMA1_Channel7_IRQHandler) \
    I(ADC1_2_IRQHandler) \
    I(USB_HP_IRQHandler) \
    I(USB_LP_IRQHandler) \
    I(FDCAN1_IT0_IRQHandler) \
    I(FDCAN1_IT1_IRQHandler) \
    I(EXTI9_5_IRQHandler) \
    I(TIM1_BRK_TIM15_IRQHandler) \
    I(TIM1_UP_TIM16_IRQHandler) \
    I(TIM1_TRG_COM_TIM17_IRQHandler) \
    I(TIM1_CC_IRQHandler) \
    I(TIM2_IRQHandler) \
    I(TIM3_IRQHandler) \
    I(TIM4_IRQHandler) \
    I(I2C1_EV_IRQHandler) \
    I(I2C1_ER_IRQHandler) \
    I(I2C2_EV_IRQHandler) \
    I(I2C2_ER_IRQHandler) \
    I(SPI1_IRQHandler) \
    I(SPI2_IRQHandler) \
    I(USART1_IRQHandler) \
    I(USART2_IRQHandler) \
    I(USART3_IRQHandler) \
    I(EXTI15_10_IRQHandler) \
    I(RTC_Alarm_IRQHandler) \
    I(USBWakeUp_IRQHandler) \
    I(TIM8_BRK_IRQHandler) \
    I(TIM8_UP_IRQHandler) \
    I(TIM8_TRG_COM_IRQHandler) \
    I(TIM8_CC_IRQHandler) \
    I(ADC3_IRQHandler) \
    I(FMC_IRQHandler) \
    I(LPTIM1_IRQHandler) \
    I(TIM5_IRQHandler) \
    I(SPI3_IRQHandler) \
    I(UART4_IRQHandler) \
    I(UART5_IRQHandler) \
    I(TIM6_DAC_IRQHandler) \
    I(TIM7_DAC_IRQHandler) \
    I(DMA2_Channel1_IRQHandler) \
    I(DMA2_Channel2_IRQHandler) \
    I(DMA2_Channel3_IRQHandler) \
    I(DMA2_Channel4_IRQHandler) \
    I(DMA2_Channel5_IRQHandler) \
    I(ADC4_IRQHandler) \
    I(ADC5_IRQHandler) \
    I(UCPD1_IRQHandler) \
    I(COMP1_2_3_IRQHandler) \
    I(COMP4_5_6_IRQHandler) \
    I(COMP7_IRQHandler) \
    I(HRTIM1_Master_IRQHandler) \
    I(HRTIM1_TIMA_IRQHandler) \
    I(HRTIM1_TIMB_IRQHandler) \
    I(HRTIM1_TIMC_IRQHandler) \
    I(HRTIM1_TIMD_IRQHandler) \
    I(HRTIM1_TIME_IRQHandler) \
    I(HRTIM1_FLT_IRQHandler) \
    I(HRTIM1_TIMF_IRQHandler) \
    I(CRS_IRQHandler) \
    I(SAI1_IRQHandler) \
    I(TIM20_BRK_IRQHandler) \
    I(TIM20_UP_IRQHandler) \
    I(TIM20_TRG_COM_IRQHandler) \
    I(TIM20_CC_IRQHandler) \
    I(FPU_IRQHandler) \
    I(I2C4_EV_IRQHandler) \
    I(I2C4_ER_IRQHandler) \
    I(SPI4_IRQHandler) \
    N() \
    I(FDCAN2_IT0_IRQHandler) \
    I(FDCAN2_IT1_IRQHandler) \
    I(FDCAN3_IT0_IRQHandler) \
    I(FDCAN3_IT1_IRQHandler) \
    I(RNG_IRQHandler) \
    I(LPUART1_IRQHandler) \
    I(I2C3_EV_IRQHandler) \
    I(I2C3_ER_IRQHandler) \
    I(DMAMUX_OVR_IRQHandler) \
    I(QUADSPI_IRQHandler) \
    I(DMA1_Channel8_IRQHandler) \
    I(DMA2_Channel6_IRQHandler) \
    I(DMA2_Channel7_IRQHandler) \
    I(DMA2_Channel8_IRQHandler) \
    I(CORDIC_IRQHandler) \
    I(FMAC_IRQHandler)

// define weak symbols
#define N_NONOP()
IRQ_SYM_LIST(WEAK_DEFAULT_HANDLER, N_NONOP)

// This is the code that gets called when the processor first
// starts execution following a reset event. Only the absolutely
// necessary set is performed, after which the application
// supplied main() routine is called.
extern "C" [[gnu::section(".text.Reset_Handler"), noreturn]]
void Reset_Handler()
{
    SystemInit();

    // Copy .data section
    uint32_t *src = _sidata;
    uint32_t *dst = _sdata;
    while (dst < _edata)
        *dst++ = *src++;

    // Copy .ccmram* section
    src = _siccmram;
    dst = _sccmram;
    while (dst < _eccmram)
        *dst++ = *src++;

    // Zeroing .bss section
    dst = _sbss;
    while (dst < _ebss)
        *dst++ = 0;

    __libc_init_array();
    main();

    while (true)
        __asm volatile("nop");
}

union VecEntry
{
    const void *stack_ptr;
    IrqHandlerPfn handler;
};

// Benefit from this definition, it is possible to use gdb to inspect
// the vector table (`p p_pfnVectors`) in realtime to check whether correct
// handler is set.
extern "C" [[gnu::section(".isr_vector"), gnu::used]]
const VecEntry g_pfnVectors[] = {
    { .stack_ptr = _estack },
    { .handler = Reset_Handler },

#define VEC_ENTRY_HANDLER(sym) { .handler = sym },
#define VEC_ENTRY_NULL()       { .handler = nullptr },
    IRQ_SYM_LIST(VEC_ENTRY_HANDLER, VEC_ENTRY_NULL)
};

