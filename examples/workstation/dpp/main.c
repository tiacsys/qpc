//============================================================================
// Product: DPP example for Windows
// Last updated for version 7.3.0
// Last updated on  2023-07-17
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
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

//Q_DEFINE_THIS_FILE

// "fudge factor" for Windows, see NOTE1
enum { WIN_FUDGE_FACTOR = 10 };

//............................................................................
int main(int argc, char *argv[]) {
    QF_init();    // initialize the framework and the underlying RT kernel
    BSP_init(argc, argv); // initialize the Board Support Package

    // initialize publish-subscribe...
    static QSubscrList subscrSto[MAX_PUB_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    // initialize event pools...
    static QF_MPOOL_EL(TableEvt) smlPoolSto[2*N_PHILO*WIN_FUDGE_FACTOR];
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

    // start the active objects...
    Philo_ctor(); // instantiate all Philosopher active objects
    static QEvt const *philoQueueSto[N_PHILO][N_PHILO*WIN_FUDGE_FACTOR];
    for (uint8_t n = 0U; n < N_PHILO; ++n) {
        QACTIVE_START(AO_Philo[n],           // AO to start
                      n + 2U,                // QF-priority
                      philoQueueSto[n],      // event queue storage
                      Q_DIM(philoQueueSto[n]), // queue length [events]
                      (void *)0,             // stack storage (not used)
                      0U,                    // size of the stack [bytes]
                      (void *)0);            // initialization param
    }

    Table_ctor(); // instantiate the Table active object
    static QEvt const *tableQueueSto[N_PHILO*WIN_FUDGE_FACTOR];
    QACTIVE_START(AO_Table,                  // AO to start
                  N_PHILO + 2U,              // QF-priority
                  tableQueueSto,             // event queue storage
                  Q_DIM(tableQueueSto),      // queue length [events]
                  (void *)0,                 // stack storage (not used)
                  0U,                        // size of the stack [bytes]
                  (void *)0);                // initialization param

    return QF_run(); // run the QF application
}

//============================================================================
// NOTE1:
// Windows is not a deterministic real-time system, which means that the
// system can occasionally and unexpectedly "choke and freeze" for a number
// of seconds. The designers of Windows have dealt with these sort of issues
// by massively over-sizing the resources available to the applications. For
// example, the default Windows GUI message queues size is 10,000 entries,
// which can dynamically grow to an even larger number. Also the stacks of
// Win32 threads can dynamically grow to several megabytes.
//
// In contrast, the event queues, event pools, and stack size inside the
// real-time embedded (RTE) systems can be (and must be) much smaller,
// because you typically can put an upper bound on the real-time behavior
// and the resulting delays.
//
// To be able to run the unmodified applications designed originally for
// RTE systems on Windows, and to reduce the odds of resource shortages in
// this case, the generous WIN_FUDGE_FACTOR is used to oversize the
// event queues and event pools.
//
