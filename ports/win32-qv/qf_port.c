//============================================================================
// QP/C Real-Time Embedded Framework (RTEF)
// Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
//
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-QL-commercial
//
// This software is dual-licensed under the terms of the open source GNU
// General Public License version 3 (or any later version), or alternatively,
// under the terms of one of the closed source Quantum Leaps commercial
// licenses.
//
// The terms of the open source GNU General Public License version 3
// can be found at: <www.gnu.org/licenses/gpl-3.0>
//
// The terms of the closed source Quantum Leaps commercial licenses
// can be found at: <www.state-machine.com/licensing>
//
// Redistributions in source code must retain this top-level comment block.
// Plagiarizing this software to sidestep the license obligations is illegal.
//
// Contact information:
// <www.state-machine.com>
// <info@state-machine.com>
//============================================================================
//!
//! @date Last updated on: 2023-07-17
//! @version Last updated for: @ref qpc_7_3_0
//!
//! @file
//! @brief QF/C port to Win32 API (single-threaded, like the QV kernel)
//!
#define QP_IMPL           // this is QP implementation
#include "qf_port.h"      // QF port
#include "qf_pkg.h"       // QF package-scope interface
#include "qassert.h"      // QP Functional Safety (FuSa) System
#ifdef Q_SPY              // QS software tracing enabled?
    #include "qs_port.h"  // QS port
    #include "qs_pkg.h"   // QS package-scope internal interface
#else
    #include "qs_dummy.h" // disable the QS software tracing
#endif // Q_SPY

#include <limits.h>       // limits of dynamic range for integers
#include <conio.h>        // console input/output

Q_DEFINE_THIS_MODULE("qf_port")

// Global objects ==========================================================
QPSet QF_readySet_;
QPSet QF_readySet_dis_;
HANDLE QV_win32Event_; // Win32 event to signal events

// Local objects ===========================================================
static CRITICAL_SECTION l_win32CritSect;
static DWORD l_tickMsec = 10U; // clock tick in msec (argument for Sleep())
static int_t l_tickPrio = 50;  // default priority of the "ticker" thread
static bool  l_isRunning;      // flag indicating when QF is running

static DWORD WINAPI ticker_thread(LPVOID arg);

// QF functions ============================================================
void QF_init(void) {
    QPSet_setEmpty(&QF_readySet_);
#ifndef Q_UNSAFE
    QPSet_update_(&QF_readySet_, &QF_readySet_dis_);
#endif

    InitializeCriticalSection(&l_win32CritSect);
    QV_win32Event_ = CreateEvent(NULL, FALSE, FALSE, NULL);
}
//..........................................................................
void QF_enterCriticalSection_(void) {
    EnterCriticalSection(&l_win32CritSect);
}
//============================================================================
void QF_leaveCriticalSection_(void) {
    LeaveCriticalSection(&l_win32CritSect);
}
//..........................................................................
void QF_stop(void) {
    l_isRunning = false;  // terminate the QV event loop and ticker thread
    SetEvent(QV_win32Event_); // unblock the event-loop so it can terminate
}
//..........................................................................
int_t QF_run(void) {
    QF_CRIT_STAT_

    QF_onStartup(); // application-specific startup callback

    l_isRunning = true; // QF is running

    if (l_tickMsec != 0U) { // system clock tick configured?
        // create the ticker thread...
        HANDLE ticker = CreateThread(NULL, 1024, &ticker_thread,
                                    (void *)0, 0, NULL);
        QF_CRIT_E_();
        Q_ASSERT_NOCRIT_(310, ticker != 0); // thread must be created
        QF_CRIT_X_();
    }

    // the combined event-loop and background-loop of the QV kernel
    QF_CRIT_E_();

    // produce the QS_QF_RUN trace record
    QS_BEGIN_NOCRIT_PRE_(QS_QF_RUN, 0U)
    QS_END_NOCRIT_PRE_()

    while (l_isRunning) {
        Q_ASSERT_NOCRIT_(200,
            QPSet_verify_(&QF_readySet_, &QF_readySet_dis_));

        // find the maximum priority AO ready to run
        if (QPSet_notEmpty(&QF_readySet_)) {
            uint_fast8_t p = QPSet_findMax(&QF_readySet_);
            QActive *a = QActive_registry_[p];

            // the active object 'a' must still be registered in QF
            // (e.g., it must not be stopped)
            //
            Q_ASSERT_NOCRIT_(320, a != (QActive *)0);
            QF_CRIT_X_();

            // perform the run-to-completion (RTS) step...
            // 1. retrieve the event from the AO's event queue, which by this
            //   time must be non-empty and The "Vanilla" kernel asserts it.
            // 2. dispatch the event to the AO's state machine.
            // 3. determine if event is garbage and collect it if so
            //
            QEvt const *e = QActive_get_(a);
            // dispatch event (virtual call)
            (*a->super.vptr->dispatch)(&a->super, e, a->prio);
            QF_gc(e);

            QF_CRIT_E_();

            if (a->eQueue.frontEvt == (QEvt *)0) { // empty queue?
                QPSet_remove(&QF_readySet_, p);
#ifndef Q_UNSAFE
                QPSet_update_(&QF_readySet_, &QF_readySet_dis_);
#endif
            }
        }
        else {
            // the QV kernel in embedded systems calls here the QV_onIdle()
            // callback. However, the Win32-QV port does not do busy-waiting
            // for events. Instead, the Win32-QV port efficiently waits until
            // QP events become available.
            //
            QF_CRIT_X_();

            (void)WaitForSingleObject(QV_win32Event_, (DWORD)INFINITE);

            QF_CRIT_E_();
        }
    }
    QF_CRIT_X_();
    QF_onCleanup();  // cleanup callback
    QS_EXIT();       // cleanup the QSPY connection

    return 0; // return success
}
//..........................................................................
void QF_setTickRate(uint32_t ticksPerSec, int_t tickPrio) {
    if (ticksPerSec != 0U) {
        l_tickMsec = 1000UL / ticksPerSec;
    }
    else {
        l_tickMsec = 0U; // means NO system clock tick
    }
    l_tickPrio = tickPrio;
}

//..........................................................................
void QF_consoleSetup(void) {
}
//..........................................................................
void QF_consoleCleanup(void) {
}
//..........................................................................
int QF_consoleGetKey(void) {
    if (_kbhit()) { // any key pressed?
        return (int)_getwch();
    }
    return 0;
}
//..........................................................................
int QF_consoleWaitForKey(void) {
    return (int)_getwch();
}

// QActive functions =======================================================
void QActive_start_(QActive * const me, QPrioSpec const prioSpec,
                    QEvt const * * const qSto, uint_fast16_t const qLen,
                    void * const stkSto, uint_fast16_t const stkSize,
                    void const * const par)
{
    Q_UNUSED_PAR(stkSize);

    // no per-AO stack needed for this port
    QF_CRIT_STAT_
    QF_CRIT_E_();
    Q_REQUIRE_NOCRIT_(600, stkSto == (void *)0);
    QF_CRIT_X_();

    me->prio  = (uint8_t)(prioSpec & 0xFFU); // QF-priority of the AO
    me->pthre = (uint8_t)(prioSpec >> 8U);   // preemption-threshold
    QActive_register_(me); // register this AO

    QEQueue_init(&me->eQueue, qSto, qLen);

    // top-most initial tran. (virtual call)
    (*me->super.vptr->init)(&me->super, par, me->prio);
    QS_FLUSH(); // flush the trace buffer to the host
}
//..........................................................................
#ifdef QF_ACTIVE_STOP
void QActive_stop(QActive * const me) {
    QActive_unsubscribeAll(me); // unsubscribe from all events

    // make sure the AO is no longer in "ready set"
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QPSet_remove(&QF_readySet_, me->prio);
#ifndef Q_UNSAFE
    QPSet_update_(&QF_readySet_, &QF_readySet_dis_);
#endif
    QF_CRIT_X_();

    QActive_unregister_(me); // un-register this active object
}
#endif
//..........................................................................
void QActive_setAttr(QActive *const me, uint32_t attr1, void const *attr2) {
    Q_UNUSED_PAR(me);
    Q_UNUSED_PAR(attr1);
    Q_UNUSED_PAR(attr2);
    Q_ERROR_NOCRIT_(900); // should not be called in this QP port
}

//==========================================================================
static DWORD WINAPI ticker_thread(LPVOID arg) { // for CreateThread()
    Q_UNUSED_PAR(arg);

    int threadPrio = THREAD_PRIORITY_NORMAL;

    // set the ticker thread priority according to selection made in
    // QF_setTickRate()
    //
    if (l_tickPrio < 33) {
        threadPrio = THREAD_PRIORITY_BELOW_NORMAL;
    }
    else if (l_tickPrio > 66) {
        threadPrio = THREAD_PRIORITY_ABOVE_NORMAL;
    }

    SetThreadPriority(GetCurrentThread(), threadPrio);

    while (l_isRunning) {
        Sleep(l_tickMsec); // wait for the tick interval
        QF_onClockTick();  // callback (must call QTIMEEVT_TICK_X())
    }

    return 0U; // return success
}

