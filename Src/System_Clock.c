/*
 * System_Clock.c
 *
 *  Created on: Dec 6, 2025
 *      Author: wiki
 */

#include "System_Clock.h"
static volatile uint32_t systick_counter = 0;

void System_ClockConfig(void)
{
	/**
	     * Purpose: Configure STM32C031 to run at 48 MHz using HSI (internal oscillator)
	     *
	     * STM32C031 Clock Options:
	     * 1. HSI48: 48 MHz internal RC oscillator (±1% accuracy) ← We use this
	     * 2. HSE: External crystal (not present on Nucleo)
	     * 3. LSI: 32 kHz low-speed internal (for watchdog/RTC)
	     *
	     * WHY HSI48?
	     * - No external components needed
	     * - Accurate enough for UART/ADC
	     * - Maximum performance (48 MHz is the CPU limit for C031)


	     */
	    //setiing the bit 8 of control register
	RCC->CR |= HSI_ON;//(1U <<8)
	while (!(RCC->CR & RCC_CR_HSIRDY));	  /*#define RCC_CR_HSIRDY_Pos  (10U)#define RCC_CR_HSIRDY_Msk (0x1UL << RCC_CR_HSIRDY_Pos)  !< 0x00000400
											#define RCC_CR_HSIRDY                    RCC_CR_HSIRDY_Msk     */
	//Configure HSI divider to get 48 MHz
	//RCC_CR bits [13:11] = HSIDIV (HSI clock division factor)
	RCC->CR &= ~RCC_CR_HSIDIV;// clearing these  bits to have divisor to be 1

	//Select HSI as system clock source
	//Bits [2:0] = SW (System clock switch)
	//if 0 means selecting HSI
	RCC->CFGR &= ~RCC_CFGR_SW;          // Clear SW bits [2:0]
	RCC->CFGR |= RCC_CFGR_SW_HSI;       // Set to 000 (HSI)

	    /* Wait until system clock switch is complete */
	    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);


	//Configure bus prescalers (AHB and APB)
	    //as we want all to run 48MHz so no change is needed let the default peripheral setting to be /1
	    //but its better to not assume and set it to be clear that it is what we wanted







}
void SysTick_Init(uint32_t ticks)
{


/**
     * Configure SysTick to generate 1 ms interrupts
     *
     * SysTick is a 24-bit down-counter built into ALL Cortex-M cores
     * (not an STM32 peripheral - it's part of ARM architecture)
     *
     * Registers (memory-mapped in Cortex-M System Control Space):
     * 0xE000E010: SYST_CSR   (Control and Status Register)
     * 0xE000E014: SYST_RVR   (Reload Value Register - 24-bit)
     * 0xE000E018: SYST_CVR   (Current Value Register - 24-bit)
     *
     *      * For 1 ms @ 48 MHz:
     * ticks = 48,000 cycles (48 MHz / 1000 Hz = 48,000)
     * SYST_RVR = 48,000 - 1 = 47,999
     *
     */
    /* Access SysTick registers via CMSIS structure */
    SysTick->CTRL  = 0;            // Disable SysTick during setup
    SysTick->LOAD  = ticks - 1;    // Set reload value (RVR)
    SysTick->VAL   = 0;            // Clear current value (CVR) - triggers reload

    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   // Bit 2: Use CPU clock
                      | SysTick_CTRL_TICKINT_Msk     // Bit 1: Enable interrupt
                      | SysTick_CTRL_ENABLE_Msk;     // Bit 0: Enable counter

}

void Delay_ms(uint32_t ms)
{

    uint32_t start = systick_counter;
    while ((systick_counter - start) < ms) {
        __WFI();  // Wait For Interrupt: CPU sleeps until next interrupt
    }

}


uint32_t GetTick(void)
{
	/**
	     * Returns elapsed milliseconds since SysTick_Init()
	     * Useful for timestamping sensor readings
	     */
	    return systick_counter;

}

void SysTick_Handler(void)
{
    /**
     * This function is called by HARDWARE every time SysTick hits 0
     *
     * EXECUTION:
     * 1. CPU saves context (registers) to stack automatically
     * 2. Jumps to this handler (from vector table at 0x0800003C)
     * 3. Increments counter
     * 4. Returns from interrupt (NVIC restores context)
     *
     * WHY so simple?
     * - Just increment counter, nothing else
     * - Total execution time: ~10 CPU cycles (208 ns @ 48 MHz)
     * - Minimal impact on main loop
     *
     * INTERRUPT PRIORITY:
     * SysTick defaults to priority 15 (lowest)
     * Can be changed in NVIC if you need higher priority interrupts
     * (e.g., UART RX should not be delayed by SysTick)
     */
    systick_counter++;
    //as it is unsigned!!----uint32_t so after 49.7 days i would overflow and get back to 0 it is deterministic behaviour
    //but if it would be signed that would be problem
}


