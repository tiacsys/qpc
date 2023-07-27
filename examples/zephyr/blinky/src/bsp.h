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
//! @date Last updated on: 2022-08-24
//! @version Last updated for: Zephyr 3.1.99 and @ref qpc_7_1_0
//!
//! @file
//! @brief BSP for Zephyr, Blinky example
//!
#ifndef BSP_H
#define BSP_H

#define BSP_TICKS_PER_SEC    1000U

void BSP_init(void);
void BSP_ledOn(void);
void BSP_ledOff(void);

#endif // BSP_H

