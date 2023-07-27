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
//! @date Last updated on: 2023-05-24
//! @version Last updated for: @ref qpc_7_3_0
//!
//! @file
//! @brief QF/C port, preemptive QK kernel, MPLAB-X XC32 compiler
//!
#ifndef QF_PORT_H_
#define QF_PORT_H_

// The maximum number of system clock tick rates
#define QF_MAX_TICK_RATE        2U

// The maximum number of active objects in the application, see NOTE1
#define QF_MAX_ACTIVE           32U

// QF interrupt disable/enable, see NOTE02
#define QF_INT_DISABLE()        __builtin_disable_interrupts()
#define QF_INT_ENABLE()         __builtin_enable_interrupts()

// QF critical section entry/exit, see NOTE02
#define QF_CRIT_STAT_
#define QF_CRIT_E_()            QF_INT_DISABLE()
#define QF_CRIT_X_()            QF_INT_ENABLE()

// fast log-base-2 with CLZ instruction
#define QF_LOG2(n_) ((uint8_t)(32U - _clz(n_)))

#include <xc.h>         // for _clz()

#include "qep_port.h"   // QEP port
#include "qk_port.h"    // QK preemptive kernel port
#include "qf.h"         // QF platform-independent public interface

//============================================================================
// NOTE1:
// The maximum number of active objects QF_MAX_ACTIVE can be increased
// up to 64, if necessary. Here it is set to a lower level to save some RAM.
//
// NOTE2:
// The DI (disable interrupts) instruction is used for fast, unconditional
// disabling and enabling of interrupts. The DI instruction only disables
// interrupts with priority levels 1-6. Priority level 7 interrupts and all
// trap events still have the ability to interrupt the CPU when the DI
// instruction is active. This means that from the perspective of QP, the
// level 7 interrupts are treated as non-maskable interrupts (NMIs). Such
// non-maskable interrupts _cannot_ call any QP services. In particular,
// they cannot post events.
//
// CAUTION: This QP port assumes that interrupt nesting is _enabled_,
// which is the default in the PIC32 processors. Interrupt nesting should
// never be disabled by setting the NSTDIS control bit (INTCON1<15>). If you
// don't want interrupts to nest, you can always prioritize them at the same
// level. For example, the default priority level for all interrupts is 4 out
// of reset. If you don't change this level for any interrupt the nesting of
// interrupt will not occur.
//

#endif // QF_PORT_H_

