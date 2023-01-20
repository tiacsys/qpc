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
//! @date Last updated on: 2023-07-18
//! @version Last updated for: @ref qpc_7_3_0
//!
//! @file
//! @brief QK/C port example for a generic C compiler.
#ifndef QK_PORT_H_
#define QK_PORT_H_

//============================================================================
//! enable the context-switch callback
#define QF_ON_CONTEXT_SW   1

//! Check if the code executes in the ISR context
#define QK_ISR_CONTEXT_() (QK_priv_.intNest != 0U)

//! Define the ISR entry sequence
#define QK_ISR_ENTRY()                               \
do {                                                 \
    QF_INT_DISABLE();                                \
    ++QK_priv_.intNest;                              \
    QF_QS_ISR_ENTRY(QK_priv_.intNest, QK_currPrio_); \
    QF_INT_ENABLE();                            \
} while (false)

//! Define the ISR exit sequence
#define QK_ISR_EXIT()             \
do {                              \
    QF_INT_DISABLE();             \
    --QK_priv_.intNest;           \
    if (QK_priv_.intNest == 0U) { \
        if (QK_sched_() != 0U) {  \
            QK_activate_();       \
        }                         \
    }                             \
    QF_INT_ENABLE();              \
} while (false)

#include "qk.h" // QK platform-independent public interface

uint32_t get_IPSR(void);

#endif // QK_PORT_H_

