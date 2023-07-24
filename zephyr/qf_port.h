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
// ============================================================================
//!
//! @date Last updated on: 2023-05-23
//! @version Last updated for: Zephyr 3.2.0 and @ref qpc_7_3_0
//!
//! @file
//! @brief QF/C port to Zephyr RTOS
//!
#ifndef QF_PORT_H_
#define QF_PORT_H_

// event queue and thread types
#define QF_EQUEUE_TYPE       struct k_msgq
#define QF_THREAD_TYPE       struct k_thread

// The maximum number of active objects in the application, NOTE1
#define QF_MAX_ACTIVE        (CONFIG_NUM_PREEMPT_PRIORITIES - 1U)

// The number of system clock tick rates
#define QF_MAX_TICK_RATE     2U

// QF critical section entry/exit for Zephyr, see NOTE1
#define QF_CRIT_STAT_        k_spinlock_key_t key_;
#define QF_CRIT_E_()         ((key_) = k_spin_lock(&QF_spinlock))
#define QF_CRIT_X_()         k_spin_unlock(&QF_spinlock, key_)

#include <zephyr/kernel.h>   // Zephyr kernel API
#include "qep_port.h"        // QEP port
#include "qequeue.h"         // used for event deferral
#include "qmpool.h"          // this QP port uses the native QF memory pool
#include "qf.h"              // QF platform-independent public interface

// Zephyr spinlock for QF critical section
extern struct k_spinlock QF_spinlock;

// Q_PRINTK() macro to avoid conflicts with Zephyr's printk()
// when Q_SPY configuation is used
//
#ifndef Q_SPY
#define Q_PRINTK(fmt_, ...)  printk(fmt_, ##__VA_ARGS__)
#else
#define Q_PRINTK(dummy, ...) ((void)0)
#endif

//==========================================================================
// interface used only inside QF implementation, but not in applications
#ifdef QP_IMPL

    // scheduler locking, see NOTE2
    #define QF_SCHED_STAT_
    #define QF_SCHED_LOCK_(dummy) do { \
        if (!k_is_in_isr()) {       \
            k_sched_lock();         \
        } \
    } while (false)
    #define QF_SCHED_UNLOCK_() do { \
        if (!k_is_in_isr()) {       \
            k_sched_unlock();       \
        } \
    } while (false)

    // native QF event pool customization
    #define QF_EPOOL_TYPE_            QMPool
    #define QF_EPOOL_INIT_(p_, poolSto_, poolSize_, evtSize_) \
        (QMPool_init(&(p_), (poolSto_), (poolSize_), (evtSize_)))
    #define QF_EPOOL_EVENT_SIZE_(p_)  ((uint_fast16_t)(p_).blockSize)
    #define QF_EPOOL_GET_(p_, e_, m_, qs_id_) \
        ((e_) = (QEvt *)QMPool_get(&(p_), (m_), (qs_id_)))
    #define QF_EPOOL_PUT_(p_, e_, qs_id_) \
        (QMPool_put(&(p_), (e_), (qs_id_)))

#endif // QP_IMPL

//============================================================================
// NOTE1:
// This QP port to Zephyr assumes that Active Objects will use only the
// preemptive Zephyr priorities [0..(CONFIG_NUM_PREEMPT_PRIORITIES - 1U)].
// In this priority numbering, the QP AO priority QF_MAX_ACTIVE (highest)
// maps to Zephyr priority 0 (highest). The QP AO priority 1 (lowest) maps
// to Zephyr priority (CONFIG_NUM_PREEMPT_PRIORITIES - 2U).
//
// NOTE2:
// Zephyr does not support selective scheduler locking up to a given
// priority ceiling. Therefore, this port uses global Zephyr scheduler lock.
//

#endif // QF_PORT_H_

