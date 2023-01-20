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
#ifndef QF_PORT_H_
#define QF_PORT_H_

// Win32 event queue and thread types
#define QF_EQUEUE_TYPE       QEQueue
// QF_OS_OBJECT_TYPE    not used
// QF_THREAD_TYPE       not used

// The maximum number of active objects in the application
#define QF_MAX_ACTIVE        64U

// The number of system clock tick rates
#define QF_MAX_TICK_RATE     2U

// Activate the QF QActive_stop() API
#define QF_ACTIVE_STOP       1

// various QF object sizes configuration for this port
#define QF_EVENT_SIZ_SIZE    4U
#define QF_EQUEUE_CTR_SIZE   4U
#define QF_MPOOL_SIZ_SIZE    4U
#define QF_MPOOL_CTR_SIZE    4U
#define QF_TIMEEVT_CTR_SIZE  4U

// Win32 critical section, see NOTE1
#define QF_CRIT_STAT_
#define QF_CRIT_E_()         QF_enterCriticalSection_()
#define QF_CRIT_X_()         QF_leaveCriticalSection_()

// QF_LOG2 not defined -- use the internal LOG2() implementation

#include "qep_port.h"  // QEP port
#include "qequeue.h"   // Win32-QV needs the native event-queue
#include "qmpool.h"    // Win32-QV needs the native memory-pool
#include "qf.h"        // QF platform-independent public interface

// internal functions for critical section management
void QF_enterCriticalSection_(void);
void QF_leaveCriticalSection_(void);

// set clock tick rate (NOTE ticksPerSec==0 disables the "ticker thread")
void QF_setTickRate(uint32_t ticksPerSec, int_t tickPrio);

// clock tick callback (NOTE not called when "ticker thread" is not running)
void QF_onClockTick(void);

// special adaptations for QWIN GUI applications...
#ifdef QWIN_GUI
    // replace main() with main_gui() as the entry point to a GUI app.
    #define main() main_gui()
    int_t main_gui(void); // prototype of the GUI application entry point
#endif

// abstractions for console access...
void QF_consoleSetup(void);
void QF_consoleCleanup(void);
int QF_consoleGetKey(void);
int QF_consoleWaitForKey(void);

//==========================================================================
// interface used only inside QF implementation, but not in applications
#ifdef QP_IMPL

    extern QPSet QF_readySet_;
    extern QPSet QF_readySet_dis_;

    // Win32-QV specific scheduler locking, see NOTE2
    #define QF_SCHED_STAT_
    #define QF_SCHED_LOCK_(dummy) ((void)0)
    #define QF_SCHED_UNLOCK_()    ((void)0)

    // Win32-QV active object event queue customization...
    #define QACTIVE_EQUEUE_WAIT_(me_) \
        Q_ASSERT_NOCRIT_(302, (me_)->eQueue.frontEvt != (QEvt *)0)
#ifndef Q_UNSAFE
    #define QACTIVE_EQUEUE_SIGNAL_(me_) \
        QPSet_insert(&QF_readySet_, (me_)->prio); \
        QPSet_update_(&QF_readySet_, &QF_readySet_dis_); \
        (void)SetEvent(QV_win32Event_)
#else
    #define QACTIVE_EQUEUE_SIGNAL_(me_) \
        QPSet_insert(&QF_readySet_, (me_)->prio); \
        (void)SetEvent(QV_win32Event_)
#endif

    // native QF event pool operations
    #define QF_EPOOL_TYPE_            QMPool
    #define QF_EPOOL_INIT_(p_, poolSto_, poolSize_, evtSize_) \
        (QMPool_init(&(p_), (poolSto_), (poolSize_), (evtSize_)))
    #define QF_EPOOL_EVENT_SIZE_(p_)  ((uint_fast16_t)(p_).blockSize)
    #define QF_EPOOL_GET_(p_, e_, m_, qs_id_) \
        ((e_) = (QEvt *)QMPool_get(&(p_), (m_), (qs_id_)))
    #define QF_EPOOL_PUT_(p_, e_, qs_id_) \
        (QMPool_put(&(p_), (e_), (qs_id_)))

    // Minimum required Windows version is Windows-XP or newer (0x0501)
    #ifdef WINVER
    #undef WINVER
    #endif
    #ifdef _WIN32_WINNT
    #undef _WIN32_WINNT
    #endif

    #define WINVER _WIN32_WINNT_WINXP
    #define _WIN32_WINNT _WIN32_WINNT_WINXP

    #define WIN32_LEAN_AND_MEAN
    #include <windows.h> // Win32 API

    extern HANDLE  QV_win32Event_; // Win32 event to signal events

#endif // QP_IMPL

// NOTES: ==================================================================
//
// NOTE1:
// QF, like all real-time frameworks, needs to execute certain sections of
// code exclusively, meaning that only one thread can execute the code at
// the time. Such sections of code are called "critical sections".
//
// This port uses a pair of functions QF_enterCriticalSection_() /
// QF_leaveCriticalSection_() to enter/leave the critical section,
// respectively.
//
// These functions are implemented in the qf_port.c module, where they
// manipulate the file-scope Win32 critical section object l_win32CritSect
// to protect all critical sections. Using the single critical section
// object for all critical section guarantees that only one thread at a time
// can execute inside a critical section. This prevents race conditions and
// data corruption.
//
// Please note, however, that the Win32 critical section implementation
// behaves differently than interrupt disabling. A common Win32 critical
// section ensures that only one thread at a time can execute a critical
// section, but it does not guarantee that a context switch cannot occur
// within the critical section. In fact, such context switches probably
// will happen, but they should not cause concurrency hazards because the
// critical section eliminates all race conditions.
//
// Unlike simply disabling and enabling interrupts, the critical section
// approach is also subject to priority inversions. Various versions of
// Windows handle priority inversions differently, but it seems that most of
// them recognize priority inversions and dynamically adjust the priorities of
// threads to prevent it. Please refer to the MSN articles for more
// information.
//
// NOTE2:
// Scheduler locking (used inside QF_publish_()) is not needed in the single-
// threaded Win32-QV port, because event multicasting is already atomic.
//

#endif // QF_PORT_H_
