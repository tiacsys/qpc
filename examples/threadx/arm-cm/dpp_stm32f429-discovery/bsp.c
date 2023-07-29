//============================================================================
// Product: "DPP example, STM32F429 Discovery board, ThreadX kernel
// Last updated for: @ref qpc_7_0_0
// Last updated on  2021-12-03
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
//============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include "stm32f4xx.h"  // CMSIS-compliant header file for the MCU used
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
// add other drivers if necessary...

Q_DEFINE_THIS_FILE

// Local-scope defines -----------------------------------------------------
#define LED_GPIO_PORT     GPIOD
#define LED_GPIO_CLK      RCC_AHB1Periph_GPIOD

#define LED4_PIN          GPIO_Pin_12
#define LED3_PIN          GPIO_Pin_13
#define LED5_PIN          GPIO_Pin_14
#define LED6_PIN          GPIO_Pin_15

#define BTN_GPIO_PORT     GPIOA
#define BTN_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define BTN_B1            GPIO_Pin_0

static uint32_t l_rnd; // random seed
static TX_TIMER l_tick_timer; // ThreadX timer to call QTIMEEVT_TICK_X()

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    enum AppRecords { // application-specific trace records
        PHILO_STAT = QS_USER,
        COMMAND_STAT
    };

    // QSpy source IDs
    static QSpyId const l_clock_tick = { QS_AP_ID };
#endif

// ISRs used in the application ==========================================

#ifdef Q_SPY
//
// ISR for receiving bytes from the QSPY Back-End
// NOTE: This ISR is "QF-unaware" meaning that it does not interact with
// the QF/QK and is not disabled. Such ISRs don't need to call QK_ISR_ENTRY/
// QK_ISR_EXIT and they cannot post or publish events.
//
//TBD...
#endif

// BSP functions ===========================================================
void BSP_init(void) {
    GPIO_InitTypeDef GPIO_struct;

    // NOTE: SystemInit() already called from the startup code
    // but SystemCoreClock needs to be updated
    //
    SystemCoreClockUpdate();

    // NOTE The VFP (Floating Point Unit) unit is configured by the RTOS

    // Initialize thr port for the LEDs
    RCC_AHB1PeriphClockCmd(LED_GPIO_CLK , ENABLE);

    // GPIO Configuration for the LEDs...
    GPIO_struct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_struct.GPIO_OType = GPIO_OType_PP;
    GPIO_struct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_struct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_struct.GPIO_Pin = LED3_PIN;
    GPIO_Init(LED_GPIO_PORT, &GPIO_struct);
    LED_GPIO_PORT->BSRRH = LED3_PIN; // turn LED off

    GPIO_struct.GPIO_Pin = LED4_PIN;
    GPIO_Init(LED_GPIO_PORT, &GPIO_struct);
    LED_GPIO_PORT->BSRRH = LED4_PIN; // turn LED off

    GPIO_struct.GPIO_Pin = LED5_PIN;
    GPIO_Init(LED_GPIO_PORT, &GPIO_struct);
    LED_GPIO_PORT->BSRRH = LED5_PIN; // turn LED off

    GPIO_struct.GPIO_Pin = LED6_PIN;
    GPIO_Init(LED_GPIO_PORT, &GPIO_struct);
    LED_GPIO_PORT->BSRRH = LED6_PIN; // turn LED off

    // Initialize thr port for Button
    RCC_AHB1PeriphClockCmd(BTN_GPIO_CLK , ENABLE);

    // GPIO Configuration for the Button...
    GPIO_struct.GPIO_Pin   = BTN_B1;
    GPIO_struct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_struct.GPIO_OType = GPIO_OType_PP;
    GPIO_struct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BTN_GPIO_PORT, &GPIO_struct);

    // seed the random number generator
    BSP_randomSeed(1234U);

    if (QS_INIT((void *)0) == 0U) { // initialize the QS software tracing
        Q_ERROR();
    }
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);

    // setup the QS filters...
    QS_GLB_FILTER(QS_ALL_RECORDS);
    QS_GLB_FILTER(-QS_QF_TICK);
}
//............................................................................
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    // exercise the FPU with some floating point computations
    float volatile x;
    x = 3.1415926F;
    x = x + 2.7182818F;

    if (stat[0] == 'h') {
        LED_GPIO_PORT->BSRRL = LED3_PIN; // turn LED on
    }
    else {
        LED_GPIO_PORT->BSRRH = LED3_PIN; // turn LED off
    }
    if (stat[0] == 'e') {
        LED_GPIO_PORT->BSRRL = LED5_PIN; // turn LED on
    }
    else {
        LED_GPIO_PORT->BSRRH = LED5_PIN; // turn LED on
    }
    (void)n; // unused parameter (in all but Spy build configuration)

    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio) // app-specific record
        QS_U8(1, n);  // Philosopher number
        QS_STR(stat); // Philosopher status
    QS_END()
}
//............................................................................
void BSP_displayPaused(uint8_t paused) {
    if (paused) {
        LED_GPIO_PORT->BSRRL = LED4_PIN; // turn LED on
    }
    else {
        LED_GPIO_PORT->BSRRH = LED4_PIN; // turn LED on
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


#ifdef Q_SPY
    // ThreadX "idle" thread for QS output, see NOTE1
    static TX_THREAD idle_thread;
    static void idle_thread_fun(ULONG thread_input);
    static ULONG idle_thread_stack[64];
#endif

// QF callbacks ============================================================
static VOID timer_expiration(ULONG id) {
    QTIMEEVT_TICK_X(id, &l_clock_tick); // perform the QF clock tick processing
}
//............................................................................
void QF_onStartup(void) {
    //
    // NOTE:
    // This application uses the ThreadX timer to periodically call
    // the QTimeEvt_tick_(0) function. Here, only the clock tick rate of 0
    // is used, but other timers can be used to call QTimeEvt_tick_() for
    // other clock tick rates, if needed.
    //
    // The choice of a ThreadX timer is not the only option. Applications
    // might choose to call QTIMEEVT_TICK_X() directly from timer interrupts
    // or from active object(s).
    //
    UINT tx_err = tx_timer_create(&l_tick_timer, // ThreadX timer object
        (CHAR *)"QP-tick", // name of the timer
        &timer_expiration, // expiration function
        0U,       // expiration function input (tick rate)
        1U,       // initial ticks
        1U,       // reschedule ticks
        TX_AUTO_ACTIVATE);
    Q_ASSERT(tx_err == TX_SUCCESS);

#ifdef Q_SPY
    //TBD: enable the UART ISR for receiving bytes...

    // start a ThreadX "idle" thread. See NOTE1...
    tx_err = tx_thread_create(&idle_thread, // thread control block
        (CHAR *)("idle"), // thread name
        &idle_thread_fun,       // thread function
        0LU,                    // thread input (unsued)
        idle_thread_stack,       // stack start
        sizeof(idle_thread_stack), // stack size in bytes
        TX_MAX_PRIORITIES - 1U, // ThreadX priority (LOWEST possible), NOTE1
        TX_MAX_PRIORITIES - 1U, // preemption threshold disabled
        TX_NO_TIME_SLICE,
        TX_AUTO_START);
    Q_ASSERT(tx_err == TX_SUCCESS);
#endif // Q_SPY
}
//............................................................................
void QF_onCleanup(void) {
}

//............................................................................
Q_NORETURN Q_onAssert(char const * const module, int_t const id) {
    //
    // NOTE: add here your application-specific error handling
    //
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);

    QS_ASSERTION(module, id, 10000U); // report assertion to QS
    NVIC_SystemReset();
    for (;;) {} // explicitly no-retur
}
//............................................................................
void assert_failed(char const * const module, int_t const id); // prototype
void assert_failed(char const * const module, int_t const id) {
    Q_onAssert(module, id);
}

// QS callbacks ============================================================
#ifdef Q_SPY

//............................................................................
static void idle_thread_fun(ULONG thread_input) { // see NOTE1
    for (;;) {
        QS_rxParse();  // parse all the received bytes

        // turn the LED6 on an off to visualize the QS activity
        LED_GPIO_PORT->BSRRL = LED6_PIN; // turn LED on
        __NOP(); // wait a little to actually see the LED glow
        __NOP();
        __NOP();
        __NOP();
        LED_GPIO_PORT->BSRRH = LED6_PIN; // turn LED off

        if ((USART2->SR & 0x80U) != 0U) { // is TXE empty?
            QF_CRIT_STAT
            QF_CRIT_ENTRY();
            uint16_t b = QS_getByte();
            QF_CRIT_EXIT();

            if (b != QS_EOD) {  // not End-Of-Data?
                USART2->DR  = (b & 0xFFU);  // put into the DR register
            }
        }

        // no blocking in this "idle" thread; see NOTE1
    }
}

//............................................................................
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[2*1024]; // buffer for Quantum Spy
    GPIO_InitTypeDef GPIO_struct;
    USART_InitTypeDef USART_struct;

    (void)arg; // avoid the "unused parameter" compiler warning
    QS_initBuf(qsBuf, sizeof(qsBuf));

    // enable peripheral clock for USART2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // GPIOA clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // GPIOA Configuration:  USART2 TX on PA2
    GPIO_struct.GPIO_Pin = GPIO_Pin_2;
    GPIO_struct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_struct.GPIO_OType = GPIO_OType_PP;
    GPIO_struct.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_struct);

    // Connect USART2 pins to AF2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // TX = PA2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // RX = PA3

    USART_struct.USART_BaudRate = 115200;
    USART_struct.USART_WordLength = USART_WordLength_8b;
    USART_struct.USART_StopBits = USART_StopBits_1;
    USART_struct.USART_Parity = USART_Parity_No;
    USART_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_struct.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_struct);

    USART_Cmd(USART2, ENABLE); // enable USART2

    // configure UART interrupts (for the RX channel)
    //TBD...

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; // to start the timestamp at zero

    return 1U; // return success
}
//............................................................................
void QS_onCleanup(void) {
}
//............................................................................
QSTimeCtr QS_onGetTime(void) {  // NOTE: invoked with interrupts DISABLED
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { // not set?
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { // the rollover occured, but the SysTick_ISR did not run yet
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
//............................................................................
void QS_onFlush(void) {
    uint16_t b;
    QF_CRIT_STAT
    QF_CRIT_ENTRY();
    while ((b = QS_getByte()) != QS_EOD) { // while not End-Of-Data...
        QF_CRIT_EXIT();
        while ((USART2->SR & USART_FLAG_TXE) == 0) { // while TXE not empty
        }
        USART2->DR = (b & 0xFFU); // put into the DR register
        QF_CRIT_ENTRY();
    }
    QF_CRIT_EXIT();
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

    QS_BEGIN_ID(COMMAND_STAT, 0U) // app-specific record
        QS_U8(2, cmdId);
        QS_U32(8, param1);
        QS_U32(8, param2);
        QS_U32(8, param3);
    QS_END()
}

#endif // Q_SPY
//----------------------------------------------------------------------------

//============================================================================
// NOTE1:
// ThreadX apparently does not have a concpet of an "idle" thread, but
// it can be emulated by a regular, but NON-BLOCKING ThreadX thread of
// the lowest priority.
//
// In the Q_SPY configuration, this "idle" thread is uded to perform
// the QS data output to the host. This is not the only choice available, and
// other applications might choose to peform the QS output some other way.
//

