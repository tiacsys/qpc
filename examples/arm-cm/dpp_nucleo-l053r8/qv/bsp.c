//============================================================================
// Product: DPP example, NUCLEO-L053R8 board, cooperative QV kernel
// Last updated for version 7.3.0
// Last updated on  2023-05-26
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <www.gnu.org/licenses/>.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
// ============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include "stm32l0xx.h"  // CMSIS-compliant header file for the MCU used
// add other drivers if necessary...

Q_DEFINE_THIS_FILE

// Local-scope defines -----------------------------------------------------
// LED pins available on the board (just one user LED LD2--Green on PA.5)
#define LD2_PIN  5U

// Button pins available on the board (just one user Button B1 on PC.13)
#define B1_PIN   13U

static uint32_t l_rnd;  // random seed

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    // QSpy source IDs
    static QSpyId const l_SysTick_Handler = { 0U };

    enum AppRecords { // application-specific trace records
        PHILO_STAT = QS_USER,
        CONTEXT_SW,
    };

#endif

// ISRs used in the application ============================================
void SysTick_Handler(void);
void EXTI0_1_IRQHandler(void);
void USART2_IRQHandler(void);

//............................................................................
void SysTick_Handler(void) {   // system clock tick ISR
    uint32_t tmp;

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; // clear CTRL_COUNTFLAG
        QS_tickTime_ += QS_tickPeriod_; // account for the clock rollover
    }
#endif

    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // process time events for rate 0
    //QTICKER_TICK(the_Ticker0, &l_SysTick_Handler); // trigger ticker AO

    // get state of the user button
    // Perform the debouncing of buttons. The algorithm for debouncing
    // adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    // and Michael Barr, page 71.
    //
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current = ~GPIOC->IDR; // read Port C with state of Button B1
    tmp = buttons.depressed; // save the debounced depressed buttons
    buttons.depressed |= (buttons.previous & current); // set depressed
    buttons.depressed &= (buttons.previous | current); // clear released
    buttons.previous   = current; // update the history
    tmp ^= buttons.depressed;     // changed debounced depressed
    if ((tmp & (1U << B1_PIN)) != 0U) { // debounced B1 state changed?
        if ((buttons.depressed & (1U << B1_PIN)) != 0U) { // B1 depressed?
            static QEvt const pauseEvt = QEVT_INITIALIZER(PAUSE_SIG);
            QACTIVE_PUBLISH(&pauseEvt, &l_SysTick_Handler);
        }
        else { // the button is released
            static QEvt const serveEvt = QEVT_INITIALIZER(SERVE_SIG);
            QACTIVE_PUBLISH(&serveEvt, &l_SysTick_Handler);
        }
    }
    QV_ARM_ERRATUM_838869();
}
//............................................................................
// interrupt handler for testing preemptions in QV
void EXTI0_1_IRQHandler(void) {
    static QEvt const testEvt = QEVT_INITIALIZER( TEST_SIG);
    QACTIVE_POST(AO_Table, &testEvt, (void *)0);
    QV_ARM_ERRATUM_838869();
}
//............................................................................
#ifdef Q_SPY
void USART2_IRQHandler(void) { // used in QS-RX (kernel UNAWARE interrutp)
    // is RX register NOT empty?
    if ((USART2->ISR & (1U << 5)) != 0) {
        uint32_t b = USART2->RDR;
        QS_RX_PUT(b);
    }
    QV_ARM_ERRATUM_838869();
}
#endif

// BSP functions ===========================================================
//............................................................................
void BSP_init(void) {
    // Configure the MPU to prevent NULL-pointer dereferencing
    // see: state-machine.com/null-pointer-protection-with-arm-cortex-m-mpu
    //
    MPU->RBAR = 0x0U                          // base address (NULL)
                | MPU_RBAR_VALID_Msk          // valid region
                | (MPU_RBAR_REGION_Msk & 7U); // region #7
    MPU->RASR = (7U << MPU_RASR_SIZE_Pos)     // 2^(7+1) region
                | (0x0U << MPU_RASR_AP_Pos)   // no-access region
                | MPU_RASR_ENABLE_Msk;        // region enable

    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk       // enable background region
                | MPU_CTRL_ENABLE_Msk;        // enable the MPU
    __ISB();
    __DSB();

    // NOTE: SystemInit() has been already called from the startup code
    // but SystemCoreClock needs to be updated
    //
    SystemCoreClockUpdate();

    // enable GPIOA clock port for the LED LD2
    RCC->IOPENR |= (1U << 0U);

    // set all used GPIOA pins as push-pull output, no pull-up, pull-down
    GPIOA->MODER   &= ~(3U << 2U*LD2_PIN);
    GPIOA->MODER   |=  (1U << 2U*LD2_PIN);
    GPIOA->OTYPER  &= ~(1U <<    LD2_PIN);
    GPIOA->OSPEEDR &= ~(3U << 2U*LD2_PIN);
    GPIOA->OSPEEDR |=  (1U << 2U*LD2_PIN);
    GPIOA->PUPDR   &= ~(3U << 2U*LD2_PIN);

    // enable GPIOC clock port for the Button B1
    RCC->IOPENR |=  (1U << 2U);

    // configure Button (PC.13) pins as input, no pull-up, pull-down
    GPIOC->MODER   &= ~(3U << 2U*B1_PIN);
    GPIOC->OSPEEDR &= ~(3U << 2U*B1_PIN);
    GPIOC->OSPEEDR |=  (1U << 2U*B1_PIN);
    GPIOC->PUPDR   &= ~(3U << 2U*B1_PIN);

    BSP_randomSeed(1234U); // seed the random number generator

    // initialize the QS software tracing...
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(CONTEXT_SW);

    // setup the QS filters...
    QS_GLB_FILTER(QS_ALL_RECORDS); // all records
    QS_GLB_FILTER(-QS_QF_TICK);    // exclude the clock tick
}
//............................................................................
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    if (stat[0] == 'h') {
        GPIOA->BSRR = (1U << LD2_PIN);  // turn LED on
    }
    else {
        GPIOA->BSRR = (1U << (LD2_PIN + 16U));  // turn LED off
    }

    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio) // app-specific record
        QS_U8(1, n);                  // Philosopher number
        QS_STR(stat);                 // Philosopher status
    QS_END()
}
//............................................................................
void BSP_displayPaused(uint8_t paused) {
    // not enough LEDs to implement this feature
    if (paused != 0U) {
        //GPIOA->BSRR = (1U << LD2_PIN);  // turn LED[n] on
    }
    else {
        //GPIOA->BSRR = (1U << (LD2_PIN + 16U));  // turn LED[n] off
    }
}
//............................................................................
uint32_t BSP_random(void) { // a very cheap pseudo-random-number generator
    // "Super-Duper" Linear Congruential Generator (LCG)
    // LCG(2^32, 3*7*11*13*23, 0, seed)
    //
    l_rnd = l_rnd * (3U*7U*11U*13U*23U);
    return l_rnd >> 8;
}
//............................................................................
void BSP_randomSeed(uint32_t seed) {
    l_rnd = seed;
}
//............................................................................
void BSP_terminate(int16_t result) {
    (void)result;
}
//............................................................................
void BSP_wait4SW1(void) {
    while ((GPIOC->IDR  & (1U << B1_PIN)) != 0U) {
        GPIOA->BSRR = (1U << LD2_PIN);         // turn LED2 on
        GPIOA->BSRR = (1U << (LD2_PIN + 16U)); // turn LED2 off
    }
}
//............................................................................
void BSP_ledOn(void) {
    //GPIOA->BSRR = (1U << LD2_PIN);        // turn LED2 on
}
//............................................................................
void BSP_ledOff(void) {
    //GPIOA->BSRR = (1U << (LD2_PIN + 16U));  // turn LED2 off
}

// QF callbacks ============================================================
void QF_onStartup(void) {
    // set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    // set priorities of ALL ISRs used in the system
    NVIC_SetPriority(USART2_IRQn,    0);
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(EXTI0_1_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 1U);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(EXTI0_1_IRQn);
#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); // UART2 interrupt used for QS-RX
#endif
}
//............................................................................
void QF_onCleanup(void) {
}
//............................................................................
#ifdef QF_ON_CONTEXT_SW
// NOTE: the context-switch callback is called with interrupts DISABLED
void QF_onContextSw(QActive *prev, QActive *next) {
    QS_BEGIN_NOCRIT(CONTEXT_SW, 0U) // no critical section!
        QS_OBJ(prev);
        QS_OBJ(next);
    QS_END_NOCRIT()
}
#endif // QF_ON_CONTEXT_SW
//............................................................................
void QV_onIdle(void) {  // called with interrupts disabled, see NOTE01

    // toggle an LED on and then off (not enough LEDs, see NOTE02)
    //GPIOA->BSRR = (1U << LD2_PIN);         // turn LED[n] on
    //GPIOA->BSRR = (1U << (LD2_PIN + 16U)); // turn LED[n] off

#ifdef Q_SPY
    QF_INT_ENABLE();
    QS_rxParse();  // parse all the received bytes

    if ((USART2->ISR & (1U << 7U)) != 0U) {  // is TXE empty?
        QF_INT_DISABLE();
        uint16_t b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {   // not End-Of-Data?
            USART2->TDR = b; // put into the DR register
        }
    }
#elif defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M MCU.
    //
    // !!!CAUTION!!!
    // QV_CPU_SLEEP() contains the WFI instruction, which stops the CPU
    // clock, which unfortunately disables the JTAG port, so the ST-Link
    // debugger can no longer connect to the board. For that reason, the call
    // to QV_CPU_SLEEP() has to be used with CAUTION.
    //
    // NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    // reset the board, then connect with ST-Link Utilities and erase the part.
    // The trick with BOOT(0) is it gets the part to run the System Loader
    // instead of your broken code. When done disconnect BOOT0, and start over.
    //
    //QV_CPU_SLEEP();  // atomically go to sleep and enable interrupts
    QF_INT_ENABLE(); // for now, just enable interrupts
#else
    QF_INT_ENABLE(); // just enable interrupts
#endif
}

//............................................................................
Q_NORETURN Q_onAssert(char const * const module, int_t const id) {
    //
    // NOTE: add here your application-specific error handling
    //
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);

    QS_ASSERTION(module, id, 10000U); // report assertion to QS

#ifndef NDEBUG
    BSP_wait4SW1();
#endif
    NVIC_SystemReset();
}
//............................................................................
void assert_failed(char const * const module, int_t const id); // prototype
void assert_failed(char const * const module, int_t const id) {
    Q_onAssert(module, id);
}

// QS callbacks ============================================================
#ifdef Q_SPY
//............................................................................
#define __DIV(__PCLK, __BAUD)       (((__PCLK / 4) *25)/(__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   \
    (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) \
        * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) \
    ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

// USART2 pins PA.2 and PA.3
#define USART2_TX_PIN 2U
#define USART2_RX_PIN 3U

//............................................................................
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[2*1024]; // buffer for Quantum Spy
    static uint8_t qsRxBuf[256];  // buffer for QS-RX channel

    (void)arg; // avoid the "unused parameter" compiler warning

    QS_initBuf(qsBuf, sizeof(qsBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    // enable peripheral clock for USART2
    RCC->IOPENR  |= ( 1U <<  0U);  // Enable GPIOA clock for USART pins
    RCC->APB1ENR |= ( 1U << 17U);  // Enable USART#2 clock

    // Configure PA3 to USART2_RX, PA2 to USART2_TX
    GPIOA->AFR[0] &= ~((15U << 4U*USART2_RX_PIN) | (15U << 4U*USART2_TX_PIN));
    GPIOA->AFR[0] |=  (( 4U << 4U*USART2_RX_PIN) | ( 4U << 4U*USART2_TX_PIN));
    GPIOA->MODER  &= ~(( 3U << 2U*USART2_RX_PIN) | ( 3U << 2U*USART2_TX_PIN));
    GPIOA->MODER  |=  (( 2U << 2U*USART2_RX_PIN) | ( 2U << 2U*USART2_TX_PIN));

    USART2->BRR  = __USART_BRR(SystemCoreClock, 115200U);  // baud rate
    USART2->CR3  = 0x0000U |      // no flow control
                   (1U << 12U);   // disable overrun detection (OVRDIS)
    USART2->CR2  = 0x0000U;       // 1 stop bit
    USART2->CR1  = ((1U <<  2U) | // enable RX
                    (1U <<  3U) | // enable TX
                    (1U <<  5U) | // enable RX interrupt
                    (0U << 12U) | // 8 data bits
                    (0U << 28U) | // 8 data bits
                    (1U <<  0U)); // enable USART

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; // to start the timestamp at zero

    return 1U; // return success
}
//............................................................................
void QS_onCleanup(void) {
}
//............................................................................
QSTimeCtr QS_onGetTime(void) { // NOTE: invoked with interrupts DISABLED
    if ((SysTick->CTRL & 0x00010000) == 0) {  // COUNT no set?
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { // the rollover occured, but the SysTick_ISR did not run yet
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
//............................................................................
void QS_onFlush(void) {
    uint16_t b;

    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) {    // while not End-Of-Data...
        QF_INT_ENABLE();
        while ((USART2->ISR & (1U << 7U)) == 0U) { // while TXE not empty
        }
        USART2->TDR = b; // put into the DR register
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
}
//............................................................................
//! callback function to reset the target (to be implemented in the BSP)
void QS_onReset(void) {
    NVIC_SystemReset();
}
//............................................................................
//! callback function to execute a user command (to be implemented in BSP)
void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}

#endif // Q_SPY
//----------------------------------------------------------------------------

//============================================================================
// NOTE01:
// The QV_onIdle() callback is called with interrupts disabled, because the
// determination of the idle condition might change by any interrupt posting
// an event. QV_onIdle() must internally enable interrupts, ideally
// atomically with putting the CPU to the power-saving mode.
//
// NOTE02:
// The User LED is used to visualize the idle loop activity. The brightness
// The User LED is used to visualize the idle loop activity. The brightness
// of the LED is proportional to the frequency of invcations of the idle loop.
// Please note that the LED is toggled with interrupts locked, so no interrupt
// execution time contributes to the brightness of the User LED.
//
