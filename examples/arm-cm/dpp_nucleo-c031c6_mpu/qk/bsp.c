//============================================================================
// Product: DPP example, NUCLEO-C031C6 board, QK kernel, MPU isolation
// Last updated for version 7.3.0
// Last updated on  2023-06-30
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. <state-machine.com>
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

#include "stm32c0xx.h"  // CMSIS-compliant header file for the MCU used
// add other drivers if necessary...

Q_DEFINE_THIS_FILE

// Local-scope defines -----------------------------------------------------
// LED pins available on the board (just one user LED LD4--Green on PA.5)
#define LD4_PIN  5U

// Button pins available on the board (just one user Button B1 on PC.13)
#define B1_PIN   13U

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    // QSpy source IDs
    static QSpyId const l_SysTick_Handler = { 0U };

    enum AppRecords { // application-specific trace records
        PHILO_STAT = QS_USER
    };
    #ifdef QF_MEM_ISOLATE
    // Shared QS filters...
    __attribute__((aligned(256)))
    QSpyFilter QS_filter_;
    #endif

#endif

// ISRs used in the application ============================================
void SysTick_Handler(void);
void EXTI0_1_IRQHandler(void);
void USART2_IRQHandler(void);

//............................................................................
void SysTick_Handler(void) { // system clock tick ISR -- kernel aware
    QK_ISR_ENTRY();   // inform QK about entering an ISR

    uint32_t tmp;

#ifdef Q_SPY
    {
        QF_INT_DISABLE();
        QF_MEM_SYS_();
        tmp = SysTick->CTRL; // clear CTRL_COUNTFLAG
        QS_tickTime_ += QS_tickPeriod_; // account for the clock rollover
        QF_MEM_APP_();
        QF_INT_ENABLE();
    }
#endif

    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events for tick-rate 0

    // get state of the user button
    // Perform the debouncing of buttons. The algorithm for debouncing
    // adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    // and Michael Barr, page 71.
    //
    // state of the button debouncing, bee CAREFUL: static memory!
    QF_INT_DISABLE();
    QF_MEM_SYS_();
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current = ~GPIOC->IDR; // Port C with state of Button B1
    tmp = buttons.depressed; // save the debounced depressed buttons
    buttons.depressed |= (buttons.previous & current); // set depressed
    buttons.depressed &= (buttons.previous | current); // clear released
    buttons.previous   = current; // update the history
    tmp ^= buttons.depressed;     // changed debounced depressed
    current = buttons.depressed;
    QF_INT_ENABLE();
    QF_MEM_APP_();

    if ((tmp & (1U << B1_PIN)) != 0U) { // debounced B1 state changed?
        if ((current & (1U << B1_PIN)) != 0U) { // is B1 depressed?
            static QEvt const pauseEvt = QEVT_INITIALIZER(PAUSE_SIG);
            QACTIVE_PUBLISH(&pauseEvt, &l_SysTick_Handler);
        }
        else { // the button is released
            static QEvt const serveEvt = QEVT_INITIALIZER(SERVE_SIG);
            QACTIVE_PUBLISH(&serveEvt, &l_SysTick_Handler);
        }
    }

    QK_ISR_EXIT();  // inform QK about exiting an ISR
}
//............................................................................
// interrupt handler for testing preemptions in QV
void EXTI0_1_IRQHandler(void) {
    QK_ISR_ENTRY(); // inform QK about entering an ISR

    static QEvt const testEvt = QEVT_INITIALIZER(TEST_SIG);
    QACTIVE_POST(AO_Table, &testEvt, (void *)0);

    QK_ISR_EXIT();  // inform QK about exiting an ISR
}
//............................................................................
#ifdef Q_SPY
void USART2_IRQHandler(void) { // used in QS-RX (kernel UNAWARE interrutp)
    // is RX register NOT empty?
    QF_MEM_SYS_();
    if ((USART2->ISR & (1U << 5)) != 0) {
        uint32_t b = USART2->RDR;
        QS_RX_PUT(b);
    }
    QK_ARM_ERRATUM_838869();
}
#endif

//==========================================================================
// MPU storage and settings...
typedef struct {
    uint32_t RBAR;
    uint32_t RASR;
} MPU_Region;

#ifdef QF_MEM_ISOLATE
//............................................................................
__attribute__(( used ))
void QF_onMemSys(void) {
    MPU->CTRL = MPU_CTRL_ENABLE_Msk  // enable the MPU
                | MPU_CTRL_PRIVDEFENA_Msk; // enable background region
    __ISB();
    __DSB();
}
//............................................................................
__attribute__(( used ))
void QF_onMemApp(void) {
    MPU->CTRL = MPU_CTRL_ENABLE_Msk; // enable the MPU
                                     // but do NOT enable background region
    __ISB();
    __DSB();
}
#endif

// Stack ...................................................................
// NOTE
// The stack size (provided here as power of 2), MUST match the actual
// stack size defined in the linker-script/startup-code.
//
enum { STACK_SIZE_POW2 = 11 };

// Table AO.................................................................
__attribute__((aligned((1U << TABLE_SIZE_POW2))))
uint8_t Table_sto[1U << TABLE_SIZE_POW2];

QActive * const AO_Table = (QActive *)Table_sto; // "opaque" pointer to AO

#ifdef QF_MEM_ISOLATE
static MPU_Region const MPU_Table[3] = {
    { (uint32_t)Table_sto + 0x10U,             //---- region #0
      ((TABLE_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { GPIOA_BASE + 0x11U,                      //---- region #1
      ((9U - 1U) << MPU_RASR_SIZE_Pos)                // 2^9=512B size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (0U << MPU_RASR_C_Pos)                       // C=0
       + (1U << MPU_RASR_B_Pos)                       // B=1
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U },
};
#endif

// Philo AOs................................................................
__attribute__((aligned((1U << PHILO_SIZE_POW2))))
static uint8_t Philo_sto[N_PHILO][1U << PHILO_SIZE_POW2];

QActive * const AO_Philo[N_PHILO] = {
    (QActive *)Philo_sto[0], // "opaque" pointer to AO
    (QActive *)Philo_sto[1], // "opaque" pointer to AO
    (QActive *)Philo_sto[2], // "opaque" pointer to AO
    (QActive *)Philo_sto[3], // "opaque" pointer to AO
    (QActive *)Philo_sto[4], // "opaque" pointer to AO
};

enum { PHILO_SHARED_SIZE_POW2 = 8 };
__attribute__((aligned((1U << PHILO_SHARED_SIZE_POW2))))
static union {
    uint32_t rnd_seed;
    uint8_t byteso[1U << PHILO_SHARED_SIZE_POW2];
} Philo_shared_sto;

#ifdef QF_MEM_ISOLATE
static MPU_Region const MPU_Philo[N_PHILO][3] = {
    {{ (uint32_t)Philo_sto[0] + 0x10U,         //---- region #0
       ((PHILO_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { (uint32_t)&Philo_shared_sto + 0x11U,            //---- region #1
      ((PHILO_SHARED_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U }},
    {{ (uint32_t)Philo_sto[1] + 0x10U,         //---- region #0
       ((PHILO_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { (uint32_t)&Philo_shared_sto + 0x11U,            //---- region #1
      ((PHILO_SHARED_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U }},
    {{ (uint32_t)Philo_sto[2] + 0x10U,         //---- region #0
       ((PHILO_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { (uint32_t)&Philo_shared_sto + 0x11U,            //---- region #1
      ((PHILO_SHARED_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U }},
    {{ (uint32_t)Philo_sto[3] + 0x10U,         //---- region #0
       ((PHILO_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { (uint32_t)&Philo_shared_sto + 0x11U,            //---- region #1
      ((PHILO_SHARED_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U }},
    {{ (uint32_t)Philo_sto[4] + 0x10U,         //---- region #0
       ((PHILO_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { (uint32_t)&Philo_shared_sto + 0x11U,            //---- region #1
      ((PHILO_SHARED_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)   // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk },                       // region enable
    { 0U + 0x12U,                              //---- region #2
      0U }},
};
#endif

// Shared Event-pools......................................................
enum { EPOOLS_SIZE_POW2 = 8 };

__attribute__((aligned((1U << EPOOLS_SIZE_POW2))))
static struct EPools {
    QF_MPOOL_EL(TableEvt) smlPool[2*N_PHILO];
    // ... other pools
} EPools_sto;
Q_ASSERT_STATIC(sizeof(EPools_sto) <= (1U << EPOOLS_SIZE_POW2));

//............................................................................
#ifdef QF_MEM_ISOLATE
static void STM32C031C6_MPU_setup(void) {

    MPU->CTRL = 0U; // disable the MPU

    // region #7: NULL-pointer protection region
    MPU->RBAR = 0x00000000U + 0x17U;        // base address + region #7
    MPU->RASR = ((8U - 1U) << MPU_RASR_SIZE_Pos) // 2^8=256B size
       + (0U << MPU_RASR_AP_Pos)                      // PA:na/UA:na
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (0U << MPU_RASR_S_Pos)                       // S=0
       + (0U << MPU_RASR_C_Pos)                       // C=0
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable

    // region #6: stack region
    MPU->RBAR = 0x20000000U + 0x16U;        // base address + region #6
    MPU->RASR = ((STACK_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable

    // region #5: ROM region for STM32C031C6, whole 32K
    MPU->RBAR = 0x08000000U + 0x15U;        // base address + region #5
    MPU->RASR = ((15U - 1U) << MPU_RASR_SIZE_Pos)     // 2^15=32K size
       + (6U << MPU_RASR_AP_Pos)                      // PA:ro/UA:ro
       + (0U << MPU_RASR_XN_Pos)                      // XN=0
       + (0U << MPU_RASR_S_Pos)                       // S=0
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable

    // region #4: Event-pools region
    MPU->RBAR = (uint32_t)&EPools_sto + 0x14U;// base address + region #4
    MPU->RASR = ((EPOOLS_SIZE_POW2 - 1U) << MPU_RASR_SIZE_Pos)  // size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable

#ifdef Q_SPY
    // region #3: QS-filters region
    MPU->RBAR = (uint32_t)&QS_filter_ + 0x13U;// base address + region #3
    MPU->RASR = ((8U - 1U) << MPU_RASR_SIZE_Pos)      // 2^8=256B size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (1U << MPU_RASR_XN_Pos)                      // XN=1
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable
#endif

    // region #0: temporary 4G region for initial transient
    MPU->RBAR = 0x00000000U + 0x10U;        // base address + region #0
    MPU->RASR = ((32U - 1U) << MPU_RASR_SIZE_Pos) // 2^32=4G size
       + (3U << MPU_RASR_AP_Pos)                      // PA:rw/UA:rw
       + (0U << MPU_RASR_XN_Pos)                      // XN=0
       + (1U << MPU_RASR_S_Pos)                       // S=1
       + (1U << MPU_RASR_C_Pos)                       // C=1
       + (0U << MPU_RASR_B_Pos)                       // B=0
       + (0U << MPU_RASR_TEX_Pos)                     // TEX=0
       + MPU_RASR_ENABLE_Msk;                         // region enable

    MPU->CTRL = MPU_CTRL_ENABLE_Msk;        // enable the MPU
    __ISB();
    __DSB();
}
#endif

// BSP functions ===========================================================
void BSP_init(void) {
#ifdef QF_MEM_ISOLATE
    // setup the MPU
    STM32C031C6_MPU_setup();
#endif

    // NOTE: SystemInit() has been already called from the startup code
    // but SystemCoreClock needs to be updated
    //
    SystemCoreClockUpdate();

    // enable GPIOA clock port for the LED LD4
    RCC->IOPENR |= (1U << 0U);

    // set all used GPIOA pins as push-pull output, no pull-up, pull-down
    GPIOA->MODER   &= ~(3U << 2U*LD4_PIN);
    GPIOA->MODER   |=  (1U << 2U*LD4_PIN);
    GPIOA->OTYPER  &= ~(1U <<    LD4_PIN);
    GPIOA->OSPEEDR &= ~(3U << 2U*LD4_PIN);
    GPIOA->OSPEEDR |=  (1U << 2U*LD4_PIN);
    GPIOA->PUPDR   &= ~(3U << 2U*LD4_PIN);

    // enable GPIOC clock port for the Button B1
    RCC->IOPENR |=  (1U << 2U);

    // configure Button B1 pin on GPIOC as input, no pull-up, pull-down
    GPIOC->MODER &= ~(3U << 2U*B1_PIN);
    GPIOC->PUPDR &= ~(3U << 2U*B1_PIN);

    BSP_randomSeed(1234U); // seed the random number generator

    // initialize the QS software tracing...
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }

    // dictionaries...
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_USR_DICTIONARY(PHILO_STAT);

    // initialize event pools
    QF_poolInit(EPools_sto.smlPool,
                sizeof(EPools_sto.smlPool),
                sizeof(EPools_sto.smlPool[0]));

    // invoke the AO/thread ctors...
    for (uint8_t n = 0U; n < N_PHILO; ++n) {
#ifdef QF_MEM_ISOLATE
        Philo_ctor(AO_Philo[n], n, MPU_Philo[n]);
#else
        Philo_ctor(AO_Philo[n], n, (void *)0);
#endif
    }
#ifdef QF_MEM_ISOLATE
    Table_ctor(AO_Table, MPU_Table);
#else
    Table_ctor(AO_Table, (void *)0);
#endif
}
//............................................................................
void BSP_ledOn(void) {
    GPIOA->BSRR = (1U << LD4_PIN);  // turn LED on
}
//............................................................................
void BSP_ledOff(void) {
    GPIOA->BSRR = (1U << (LD4_PIN + 16U));  // turn LED off
}
//............................................................................
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    if (stat[0] == 'h') {
        GPIOA->BSRR = (1U << LD4_PIN);  // turn LED on
    }
    else {
        GPIOA->BSRR = (1U << (LD4_PIN + 16U));  // turn LED off
    }

    QS_BEGIN_ID(PHILO_STAT, AO_Table->prio) // app-specific record
        QS_U8(1, n);  // Philosopher number
        QS_STR(stat); // Philosopher status
    QS_END()
}
//............................................................................
void BSP_displayPaused(uint8_t paused) {
    // not enough LEDs to implement this feature
    if (paused != 0U) {
        //GPIOA->BSRR = (1U << LD4_PIN);  // turn LED[n] on
    }
    else {
        //GPIOA->BSRR = (1U << (LD4_PIN + 16U));  // turn LED[n] off
    }
}
//............................................................................
uint32_t BSP_random(void) { // a very cheap pseudo-random-number generator
    // "Super-Duper" Linear Congruential Generator (LCG)
    // LCG(2^32, 3*7*11*13*23, 0, seed)
    //
    Philo_shared_sto.rnd_seed = Philo_shared_sto.rnd_seed * (3U*7U*11U*13U*23U);
    return Philo_shared_sto.rnd_seed >> 8;
}
//............................................................................
void BSP_randomSeed(uint32_t const seed) {
    Philo_shared_sto.rnd_seed = seed;
}
//............................................................................
void BSP_terminate(int16_t result) {
    (void)result;
}

// QF callbacks ============================================================
//............................................................................
void QF_onStartup(void) {
    // setup the QS filters...
    QS_GLB_FILTER(QS_ALL_RECORDS);
    QS_GLB_FILTER(-QS_QF_TICK);

    // set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    // set priorities of ALL ISRs used in the system, see NOTE1
    NVIC_SetPriority(USART2_IRQn,    0); // kernel UNAWARE interrupt
    NVIC_SetPriority(EXTI0_1_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 1U);
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
    if (next != (QActive *)0) {
        MPU->CTRL = 0U; // disable the MPU

        MPU_Region const * const region = (MPU_Region const *)next->thread;
        MPU->RBAR = region[0].RBAR;
        MPU->RASR = region[0].RASR;
        MPU->RBAR = region[1].RBAR;
        MPU->RASR = region[1].RASR;
        MPU->RBAR = region[2].RBAR;
        MPU->RASR = region[2].RASR;

        MPU->CTRL = MPU_CTRL_ENABLE_Msk        // enable the MPU
                    | MPU_CTRL_PRIVDEFENA_Msk; // enable background region
        __ISB();
        __DSB();
    }
}
#endif // QF_ON_CONTEXT_SW
//............................................................................
void QK_onIdle(void) { // called with interrupts enabled

    // toggle an LED on and then off (not enough LEDs, see NOTE02)
    //QF_INT_DISABLE();
    //QF_MEM_SYS_();
    //GPIOA->BSRR = (1U << LD4_PIN);         // turn LED[n] on
    //GPIOA->BSRR = (1U << (LD4_PIN + 16U)); // turn LED[n] off
    //QF_MEM_APP_();
    //QF_INT_ENABLE();

#ifdef Q_SPY
    QF_INT_DISABLE();
    QF_MEM_SYS_();
    QS_rxParse();  // parse all the received bytes
    QF_MEM_APP_();
    QF_INT_ENABLE();
    QF_CRIT_EXIT_NOP();

    QF_INT_DISABLE();
    QF_MEM_SYS_();
    if ((USART2->ISR & (1U << 7U)) != 0U) {  // is TXE empty?
        uint16_t b = QS_getByte();
        if (b != QS_EOD) {  // not End-Of-Data?
            USART2->TDR = b; // put into the DR register
        }
    }
    QF_MEM_APP_();
    QF_INT_ENABLE();
#elif defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M MCU.
    //
    // !!!CAUTION!!!
    // The WFI instruction stops the CPU clock, which unfortunately disables
    // the JTAG port, so the ST-Link debugger can no longer connect to the
    // board. For that reason, the call to __WFI() has to be used with CAUTION.
    //
    // NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    // reset the board, then connect with ST-Link Utilities and erase the part.
    // The trick with BOOT(0) is it gets the part to run the System Loader
    // instead of your broken code. When done disconnect BOOT0, and start over.
    //
    //__WFI(); // Wait-For-Interrupt
#endif
}

//............................................................................
Q_NORETURN Q_onAssert(char const * const module, int_t const id) {
    //
    // NOTE: add here your application-specific error handling
    //
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);

    MPU->CTRL = 0U; // disable the MPU
    GPIOA->BSRR = (1U << LD4_PIN);  // turn LED on

    QS_ASSERTION(module, id, 10000U); // report assertion to QS
#ifndef NDEBUG
    // for debugging, hang on in an endless loop...
    for (;;) {
    }
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
static uint16_t const UARTPrescTable[12] = {
    1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U
};

#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__, __CLOCKPRESCALER__) \
  ((((__PCLK__)/UARTPrescTable[(__CLOCKPRESCALER__)]) \
  + ((__BAUD__)/2U)) / (__BAUD__))

#define UART_PRESCALER_DIV1  0U

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
    RCC->APBENR1 |= ( 1U << 17U);  // Enable USART#2 clock

    // Configure PA to USART2_RX, PA to USART2_TX
    GPIOA->AFR[0] &= ~((15U << 4U*USART2_RX_PIN) | (15U << 4U*USART2_TX_PIN));
    GPIOA->AFR[0] |=  (( 1U << 4U*USART2_RX_PIN) | ( 1U << 4U*USART2_TX_PIN));
    GPIOA->MODER  &= ~(( 3U << 2U*USART2_RX_PIN) | ( 3U << 2U*USART2_TX_PIN));
    GPIOA->MODER  |=  (( 2U << 2U*USART2_RX_PIN) | ( 2U << 2U*USART2_TX_PIN));

    // baud rate
    USART2->BRR  = UART_DIV_SAMPLING16(
                       SystemCoreClock, 115200U, UART_PRESCALER_DIV1);
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
    for (;;) {
        QF_INT_DISABLE();
        QF_MEM_SYS_();
        uint16_t b = QS_getByte();
        if (b != QS_EOD) {
            while ((USART2->ISR & (1U << 7U)) == 0U) { // while TXE not empty
                QF_MEM_APP_();
                QF_INT_ENABLE();
                QF_CRIT_EXIT_NOP();

                QF_INT_DISABLE();
                QF_MEM_SYS_();
            }
            USART2->TDR = b; // put into the DR register
            QF_MEM_APP_();
            QF_INT_ENABLE();
        }
        else {
            QF_MEM_APP_();
            QF_INT_ENABLE();
            break;
        }
    }
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
    Q_UNUSED_PAR(cmdId);
    Q_UNUSED_PAR(param1);
    Q_UNUSED_PAR(param2);
    Q_UNUSED_PAR(param3);
}

#endif // Q_SPY
//----------------------------------------------------------------------------

//==========================================================================
// NOTE0:
// Usually, one of the LEDs is used to visualize the idle loop activity.
// However, the board has not enough LEDs (only one, actually), so this
// feature is disabled.
//

