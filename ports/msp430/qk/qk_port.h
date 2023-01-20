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
//! @date Last updated on: 2023-07-18
//! @version Last updated for: @ref qpc_7_3_0
//!
//! @file
//! @brief QK/C port port to MSP430
//!
#ifndef QK_PORT_H_
#define QK_PORT_H_

//! Check if the code executes in the ISR context
#define QK_ISR_CONTEXT_() (QK_priv_.intNest != 0U)

// QK interrupt entry and exit...
#define QK_ISR_ENTRY()    (++QK_priv_.intNest)

#define QK_ISR_EXIT()     do {    \
    --QK_priv_.intNest;           \
    if (QK_priv_.intNest == 0U) { \
        if (QK_sched_() != 0U) {  \
            QK_activate_();       \
        }                         \
    }                             \
    else {                        \
        Q_ERROR();                \
    }                             \
} while (false)

#include "qk.h"  // QK platform-independent public interface

#endif // QK_PORT_H_
