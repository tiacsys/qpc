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
//! @date Last updated on: 2023-05-23
//! @version Last updated for: @ref qpc_7_3_0
//!
//! @file
//! @brief QF/C port example for QV, generic C99 compiler.
#ifndef QF_PORT_H_
#define QF_PORT_H_

// interrupt disabling mechanism
#define QF_INT_DISABLE()    intDisable()
#define QF_INT_ENABLE()     intEnable()

void intDisable(void);
void intEnable(void);

// QF critical section mechanism
#define QF_CRIT_STAT        crit_stat_t crit_stat_;
#define QF_CRIT_ENTRY()     (crit_stat_ = critEntry())
#define QF_CRIT_EXIT()      critExit(crit_stat_)

typedef unsigned int crit_stat_t;
crit_stat_t critEntry(void);
void critExit(crit_stat_t stat);

#include "qep_port.h"  // QEP port
#include "qv_port.h"   // QV port

#endif // QF_PORT_H_

