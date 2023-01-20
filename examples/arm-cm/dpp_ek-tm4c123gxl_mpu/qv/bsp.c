//============================================================================
// Product: DPP example, EK-TM4C123GXL board, QV kernel, MPU isolation
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
//============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include "TM4C123GH6PM.h"        // the device specific header (TI)
#include "rom.h"                 // the built-in ROM functions (TI)
#include "sysctl.h"              // system control driver (TI)
#include "gpio.h"                // GPIO driver (TI)
// add other drivers if necessary...

Q_DEFINE_THIS_FILE  // define the name of this file for assertions

// Local-scope objects -----------------------------------------------------
#define LED_RED     (1U << 1U)
#define LED_GREEN   (1U << 3U)
#define LED_BLUE    (1U << 2U)

#define BTN_SW1     (1U << 4U)
#define BTN_SW2     (1U << 0U)

#ifdef Q_SPY

    // QSpy source IDs
    static QSpyId const l_SysTick_Handler = { 0U };
    static QSpyId const l_GPIOPortA_IRQHandler = { 0U };

    #define UART_BAUD_RATE      115200U
    #define UART_FR_TXFE        (1U << 7U)
    #define UART_FR_RXFE        (1U << 4U)
    #define UART_TXFIFO_DEPTH   16U

    enum AppRecords { // application-specific trace records
        PHILO_STAT = QS_USER,
        PAUSED_STAT,
        CONTEXT_SW,
        COMMAND_STAT
    };
    #ifdef QF_MEM_ISOLATE
    // Shared QS filters...
    __attribute__((aligned(32)))
    QSpyFilter QS_filter_;
    #endif

#endif

// ISRs used in the application ============================================
void SysTick_Handler(void);
void GPIOPortA_IRQHandler(void);
void UART0_IRQHandler(void);

//............................................................................
void SysTick_Handler(void) {

    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events for tick-rate 0

    // get state of the user button
    // Perform the debouncing of buttons. The algorithm for debouncing
    // adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    // and Michael Barr, page 71.
    //
    // state of the button debouncing, bee CAREFUL: static memory!
    QF_INT_DISABLE();
    QF_MEM_SYS();
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current = ~GPIOF_AHB->DATA_Bits[BTN_SW1 | BTN_SW2]; // read SW1&SW2
    uint32_t tmp = buttons.depressed; // save debounced depressed buttons
    buttons.depressed |= (buttons.previous & current); // set depressed
    buttons.depressed &= (buttons.previous | current); // clear released
    buttons.previous   = current; // update the history
    tmp ^= buttons.depressed;     // changed debounced depressed
    current = buttons.depressed;
    QF_INT_ENABLE();
    QF_MEM_APP();

    if ((tmp & BTN_SW1) != 0U) { // debounced SW1 state changed?
        if ((current & BTN_SW1) != 0U) { // is SW1 depressed?
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
void GPIOPortA_IRQHandler(void) {
    QACTIVE_POST(AO_Table, Q_NEW(QEvt, MAX_PUB_SIG), // for testing...
                 &l_GPIOPortA_IRQHandler);
}
//............................................................................
#ifdef Q_SPY
//
// ISR for receiving bytes from the QSPY Back-End
// NOTE: This ISR is "QF-unaware" meaning that it does not interact with
// the QF/QK and is not disabled. Such ISRs don't need to call QK_ISR_ENTRY/
// QK_ISR_EXIT and they cannot post or publish events.
//
void UART0_IRQHandler(void) {
    QF_MEM_SYS();

    uint32_t status = UART0->RIS; // get the raw interrupt status
    UART0->ICR = status;          // clear the asserted interrupts

    while ((UART0->FR & UART_FR_RXFE) == 0) { // while RX FIFO NOT empty
        uint32_t b = UART0->DR;
        QS_RX_PUT(b);
    }
    QV_ARM_ERRATUM_838869();
}
#else
void UART0_IRQHandler(void) {}
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
    { GPIOF_AHB_BASE + 0x11U,                  //---- region #1
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

enum { PHILO_SHARED_SIZE_POW2 = 5 };
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
static void TM4C123GXL_MPU_setup(void) {

    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

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

    // region #5: ROM region for TM4C123GXL, whole 256K
    MPU->RBAR = 0x00000000U + 0x15U;        // base address + region #5
    MPU->RASR = ((18U - 1U) << MPU_RASR_SIZE_Pos)     // 2^18=256K size
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
    MPU->RASR = ((5U - 1U) << MPU_RASR_SIZE_Pos)      // 2^5=32B size
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
    TM4C123GXL_MPU_setup();
#endif

    // NOTE: SystemInit() has been already called from the startup code
    // but SystemCoreClock needs to be updated
    //
    SystemCoreClockUpdate();

    // NOTE: The VFP (hardware Floating Point) unit is configured by QV

    // enable clock for to the peripherals used by this application...
    SYSCTL->RCGCGPIO  |= (1U << 5); // enable Run mode for GPIOF
    SYSCTL->GPIOHBCTL |= (1U << 5); // enable AHB for GPIOF
    __ISB();
    __DSB();

    // configure LEDs (digital output)
    GPIOF_AHB->DIR |= (LED_RED | LED_BLUE | LED_GREEN);
    GPIOF_AHB->DEN |= (LED_RED | LED_BLUE | LED_GREEN);
    GPIOF_AHB->DATA_Bits[LED_RED | LED_BLUE | LED_GREEN] = 0U;

    // configure switches...

    // unlock access to the SW2 pin because it is PROTECTED
    GPIOF_AHB->LOCK = 0x4C4F434BU; // unlock GPIOCR register for SW2
    // commit the write (cast const away)
    *(uint32_t volatile *)&GPIOF_AHB->CR = 0x01U;

    GPIOF_AHB->DIR &= ~(BTN_SW1 | BTN_SW2); // input
    GPIOF_AHB->DEN |= (BTN_SW1 | BTN_SW2); // digital enable
    GPIOF_AHB->PUR |= (BTN_SW1 | BTN_SW2); // pull-up resistor enable

    *(uint32_t volatile *)&GPIOF_AHB->CR = 0x00U;
    GPIOF_AHB->LOCK = 0x0; // lock GPIOCR register for SW2

    BSP_randomSeed(1234U); // seed the random number generator

    // initialize the QS software tracing...
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }

    // dictionaries...
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_OBJ_DICTIONARY(&l_GPIOPortA_IRQHandler);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(PAUSED_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);

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
    GPIOF_AHB->DATA_Bits[LED_RED] = 0xFFU;
}
//............................................................................
void BSP_ledOff(void) {
    GPIOF_AHB->DATA_Bits[LED_RED] = 0x00U;
}
//............................................................................
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    GPIOF_AHB->DATA_Bits[LED_GREEN] = ((stat[0] == 'e') ? LED_GREEN : 0U);

    QS_BEGIN_ID(PHILO_STAT, AO_Table->prio) // app-specific record
        QS_U8(1, n);  // Philosopher number
        QS_STR(stat); // Philosopher status
    QS_END()
}
//............................................................................
void BSP_displayPaused(uint8_t paused) {
    GPIOF_AHB->DATA_Bits[LED_BLUE] = ((paused != 0U) ? LED_BLUE : 0U);

    QS_BEGIN_ID(PAUSED_STAT, 0U) // app-specific record
        QS_U8(1, paused);  // Paused status
    QS_END()
}
//............................................................................
uint32_t BSP_random(void) { // a very cheap pseudo-random-number generator
    // Some flating point code is to exercise the VFP...
    float volatile x = 3.1415926F;
    x = x + 2.7182818F;

    // NOTE: no need for scheduler locking in QV
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
    NVIC_SetPriority(UART0_IRQn,   0U); // kernel unaware interrupt
    NVIC_SetPriority(GPIOA_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI + 1U);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(GPIOA_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(UART0_IRQn);  // UART0 interrupt used for QS-RX
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
void QV_onIdle(void) { // called with interrupts disabled, see NOTE2
    // toggle the User LED on and then off, see NOTE3
    QF_MEM_SYS();
    GPIOF_AHB->DATA_Bits[LED_BLUE] = 0xFFU;  // turn the Blue LED on
    GPIOF_AHB->DATA_Bits[LED_BLUE] = 0U;     // turn the Blue LED off

#ifdef Q_SPY
    QS_rxParse();  // parse all the received bytes
    QF_MEM_APP();
    QF_INT_ENABLE();
    QF_CRIT_EXIT_NOP();

    QF_INT_DISABLE();
    QF_MEM_SYS();
    if ((UART0->FR & UART_FR_TXFE) != 0U) {  // TX done?
        uint16_t fifo = UART_TXFIFO_DEPTH;   // max bytes we can accept

        // try to get next contiguous block to transmit
        uint8_t const *block = QS_getBlock(&fifo);

        while (fifo-- != 0) {     // any bytes in the block?
            UART0->DR = *block++; // put into the FIFO
        }
    }
    QF_MEM_APP();
    QF_INT_ENABLE();
#elif defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M MCU.
    //
    QV_CPU_SLEEP();  // atomically go to sleep and enable interrupts
#else
    QF_MEM_APP();
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
    // light up all LEDs
    GPIOF_AHB->DATA_Bits[LED_GREEN | LED_RED | LED_BLUE] = 0xFFU;
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
uint8_t QS_onStartup(void const *arg) {
    Q_UNUSED_PAR(arg);

    static uint8_t qsTxBuf[2*1024]; // buffer for QS-TX channel
    static uint8_t qsRxBuf[100];    // buffer for QS-RX channel

    QS_initBuf  (qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    // enable clock for UART0 and GPIOA (used by UART0 pins)
    SYSCTL->RCGCUART |= (1U << 0); // enable Run mode for UART0
    SYSCTL->RCGCGPIO |= (1U << 0); // enable Run mode for GPIOA

    // configure UART0 pins for UART operation
    uint32_t tmp = (1U << 0) | (1U << 1);
    GPIOA->DIR   &= ~tmp;
    GPIOA->SLR   &= ~tmp;
    GPIOA->ODR   &= ~tmp;
    GPIOA->PUR   &= ~tmp;
    GPIOA->PDR   &= ~tmp;
    GPIOA->AMSEL &= ~tmp;  // disable analog function on the pins
    GPIOA->AFSEL |= tmp;   // enable ALT function on the pins
    GPIOA->DEN   |= tmp;   // enable digital I/O on the pins
    GPIOA->PCTL  &= ~0x00U;
    GPIOA->PCTL  |= 0x11U;

    // configure the UART for the desired baud rate, 8-N-1 operation
    tmp = (((SystemCoreClock * 8U) / UART_BAUD_RATE) + 1U) / 2U;
    UART0->IBRD   = tmp / 64U;
    UART0->FBRD   = tmp % 64U;
    UART0->LCRH   = (0x3U << 5); // configure 8-N-1 operation
    UART0->LCRH  |= (0x1U << 4); // enable FIFOs
    UART0->CTL    = (1U << 0)    // UART enable
                    | (1U << 8)  // UART TX enable
                    | (1U << 9); // UART RX enable

    // configure UART interrupts (for the RX channel)
    UART0->IM   |= (1U << 4) | (1U << 6); // enable RX and RX-TO interrupt
    UART0->IFLS |= (0x2U << 2);    // interrupt on RX FIFO half-full
    // NOTE: do not enable the UART0 interrupt yet. Wait till QF_onStartup()

    // configure TIMER5 to produce QS time stamp
    SYSCTL->RCGCTIMER |= (1U << 5);  // enable run mode for Timer5
    TIMER5->CTL  = 0U;               // disable Timer1 output
    TIMER5->CFG  = 0x0U;             // 32-bit configuration
    TIMER5->TAMR = (1U << 4) | 0x02; // up-counting periodic mode
    TIMER5->TAILR= 0xFFFFFFFFU;      // timer interval
    TIMER5->ICR  = 0x1U;             // TimerA timeout flag bit clears
    TIMER5->CTL |= (1U << 0);        // enable TimerA module

    return 1U; // return success
}
//............................................................................
void QS_onCleanup(void) {
}
//............................................................................
QSTimeCtr QS_onGetTime(void) {  // NOTE: invoked with interrupts DISABLED
    return TIMER5->TAV;
}
//............................................................................
void QS_onFlush(void) {
    for (;;) {
        QF_INT_DISABLE();
        QF_MEM_SYS();
        uint16_t b = QS_getByte();
        if (b != QS_EOD) {
            while ((UART0->FR & UART_FR_TXFE) == 0U) { // while TXE not empty
                QF_MEM_APP();
                QF_INT_ENABLE();
                QF_CRIT_EXIT_NOP();

                QF_INT_DISABLE();
                QF_MEM_SYS();
            }
            UART0->DR = b; // put into the DR register
            QF_MEM_APP();
            QF_INT_ENABLE();
        }
        else {
            QF_MEM_APP();
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
// The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
// ISR priority that is disabled by the QF framework. The value is suitable
// for the NVIC_SetPriority() CMSIS function.
//
// Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
// with the numerical values of priorities equal or higher than
// QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QK_ISR_ENTRY/QK_ISR_ENTRY
// macros or any other QF services. These ISRs are "QF-aware".
//
// Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
// level (i.e., with the numerical values of priorities less than
// QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
// Such "QF-unaware" ISRs cannot call any QF services. The only mechanism
// by which a "QF-unaware" ISR can communicate with the QF framework is by
// triggering a "QF-aware" ISR, which can post/publish events.
//
// NOTE2:
// The QV_onIdle() callback is called with interrupts disabled, because the
// determination of the idle condition might change by any interrupt posting
// an event. QV_onIdle() must internally enable interrupts, ideally
// atomically with putting the CPU to the power-saving mode.
//
// NOTE3:
// The User LED is used to visualize the idle loop activity. The brightness
// of the LED is proportional to the frequency of invcations of the idle loop.
// Please note that the LED is toggled with interrupts locked, so no interrupt
// execution time contributes to the brightness of the User LED.
//

