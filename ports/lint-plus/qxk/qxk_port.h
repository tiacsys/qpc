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
//! @brief QXK/C port example for a generic C99 compiler.
//! @description
//! This is an example of a QP/C port with the documentation for the
//! configuration macros and includes.
#ifndef QXK_PORT_H_
#define QXK_PORT_H_

//! Check if the code executes in the ISR context
#define QXK_ISR_CONTEXT_() (QXK_get_IPSR() != 0U)

//! Trigger context switch (used internally in QXK only)
#define QXK_CONTEXT_SWITCH_() (QXK_trigPendSV())

//! Define the ISR entry sequence
#define QXK_ISR_ENTRY() ((void)0)

//! Define the ISR exit sequence
#define QXK_ISR_EXIT()  do {                                  \
    QF_INT_DISABLE();                                         \
    if (QXK_sched_() != 0U) {                                 \
        *Q_UINT2PTR_CAST(uint32_t, 0xE000ED04U) = (1U << 28U);\
    }                                                         \
    QF_INT_ENABLE();                                          \
    QXK_ARM_ERRATUM_838869();                                 \
} while (false)

#include "qxk.h" // QXK platform-independent public interface

uint32_t QXK_get_IPSR(void);
void QXK_trigPendSV(void);

#endif // QXK_PORT_H_

