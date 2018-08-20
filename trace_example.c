/* 2015 Petteri Aimonen <jpa@git.mail.kapsi.fi>
 * Public domain
 *
 * This file is an example on how to configure the TPIU/DWT/ITM/ETM blocks
 * on a Cortex-M3 microcontroller for tracing the execution. Trace data is
 * output from the TRACESWO pin.
 *
 * Designed to run especially on STM32 Value Line discovery board, but should
 * be easily adaptible to other boards also. Note that the STM32F100 chip on
 * value line discovery does not have ETM feature.
 *
 * What this does:
 * 1) Configures the trace pin to output TPIU formatted trace from both ITM and ETM.
 * 2) Blinks a led, while monitored by ITM tracing.
 * 3) Causes periodic interrupts, where it runs bubblesort while tracing it with ETM.
 */

#include "stm32f10x.h"
#include "core_cm3.h"
#include "arm_etm.h"

int globalCounter; // For watchpoint example

void hardfault_handler(void) { for(;;); }

void configure_tracing()
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
    
    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }
    
    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = 0; // Trace clock = HCLK/(x+1) = 8MHz = UART 's baudrate
                   // The HCLK of F105 is 8MHz so x is 0, and the F103 is 72MHz so x is 8
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    TPI->FFCR = 0x102; // TPIU packet framing enabled when bit 2 is set.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.
    
    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x32, 1 = x512
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                                // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
    
    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports
    
    /* Configure embedded trace macroblock */
    ETM->LAR = 0xC5ACCE55;
    ETM_SetupMode();
    ETM->CR = ETM_CR_ETMEN // Enable ETM output port
            | ETM_CR_STALL_PROCESSOR // Stall processor when fifo is full
            | ETM_CR_BRANCH_OUTPUT; // Report all branches
         // | ETM_CR_PORTIZE_8BIT;  // Add this code in F103 to set port_size 21, 6, 5, 4 as 0, 0, 0, 1 for 8Bit.
    ETM->TRACEIDR = 2; // Trace bus ID for TPIU
    ETM->TECR1 = ETM_TECR1_EXCLUDE; // Trace always enabled
    ETM->FFRR = ETM_FFRR_EXCLUDE; // Stalling always enabled
    ETM->FFLR = 24; // Stall when less than N bytes free in FIFO (range 1..24)
                    // Larger values mean less latency in trace, but more stalls.
    // ETM->TRIGGER = 0x0000406F; // Add this code in F103 to define the trigger event
    // ETM->TEEVR = 0x0000006F;   // Add this code in F103 to  define an event to start/stop
    // Note: we do not enable ETM trace yet, only for specific parts of code.
}

void configure_watchpoint()
{
    /* This is an example of how to configure DWT to monitor a watchpoint.
       The data value is reported when the watchpoint is hit. */
    
    /* Monitor all accesses to GPIOC (range length 32 bytes) */
    DWT->COMP0 = (uint32_t)GPIOC;
    DWT->MASK0 = 5;
    DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos) // Report data and addr on watchpoint hit
                   | (1 << DWT_FUNCTION_EMITRANGE_Pos);
    
    /* Monitor all accesses to globalCounter (range length 4 bytes) */
    DWT->COMP1 = (uint32_t)&globalCounter;
    DWT->MASK1 = 2;
    DWT->FUNCTION1 = (3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
}

// Print a given string to ITM.
// This uses 8 bit writes, as that seems to be the most common way to write text
// through ITM. Otherwise there is no good way for the PC software to know what
// is text and what is some other data.
void ITM_Print(int port, const char *p)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (*p)
        {
            while (ITM->PORT[port].u32 == 0);
            ITM->PORT[port].u8 = *p++;
        }
    }
}

// Write a 32-bit value to ITM.
// This can be used as a fast way to log important values from code.
void ITM_SendValue (int port, uint32_t value)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (ITM->PORT[port].u32 == 0);
        ITM->PORT[port].u32 = value;
    }
}

void delay()
{
    for (int i = 0; i < 10000; i++)
        asm("nop");
}

void bubble_sort (int *a, int n) {
    int i, t, s = 1;
    while (s) {
        s = 0;
        for (i = 1; i < n; i++) {
            if (a[i] < a[i - 1]) {
                t = a[i];
                a[i] = a[i - 1];
                a[i - 1] = t;
                s = 1;
            }
        }
    }
}

void TIM2_IRQ()
{
    int values[5] = {35,2,235,11,2};

    // We are very interested in how the values get sorted,
    // so we enable ETM tracing for it.
    ETM_TraceMode();
    GPIOC->BSRR = (1 << 9); // Toggle a led so that we can see the latency in ETM trace
    bubble_sort(values, 5);
    GPIOC->BRR = (1 << 9);
    ETM_SetupMode();

    // We can also use ITM to send the result of the sort
    // Note that we use port 1 here so that output does not get mixed
    // with port 0 output from main thread.
    ITM_Print(1, "Sort");
    for (int i = 0; i < 5; i++)
        ITM_SendValue(1, values[i]);
    
    TIM2->SR = 0; // Clear interrupt flag
}

int main(void)
{
    configure_tracing();
    configure_watchpoint();

    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH = 0x44444433; // GPIOC 8 and 9 as output (STM32F1 discovery leds)
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->ARR = 50000;
    TIM2->DIER = 1;
    TIM2->CR1 = 1;
    
    ITM_Print(0, "Boot");
    
    globalCounter = 0;
    
    for (;;)
    {
        delay();
        GPIOC->BSRR = (1 << 8);
        ITM_Print(0, "On");

        delay();
        GPIOC->BRR = (1 << 8);
        ITM_Print(0, "Off");
        
        globalCounter++; // This will trigger the watchpoint
    }
}


void* myvectors[] 
__attribute__ ((section("vectors")))= {
    (void*)0x20002000, // Stack ptr
    main,       // Reset addr
    hardfault_handler,
    hardfault_handler,
    [16 + TIM2_IRQn] = TIM2_IRQ
};

