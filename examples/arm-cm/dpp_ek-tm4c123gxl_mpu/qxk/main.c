//============================================================================
// Product: DPP example extended for QXK
// Last updated for version 7.3.0
// Last updated on  2023-06-28
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
// ============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

//Q_DEFINE_THIS_FILE

//............................................................................
int main() {

    QF_init();    // initialize the framework and the underlying RT kernel
    BSP_init();   // initialize the Board Support Package

    // signal dictionaries...
    QS_SIG_DICTIONARY(DONE_SIG,      (void *)0); // global signals
    QS_SIG_DICTIONARY(EAT_SIG,       (void *)0);
    QS_SIG_DICTIONARY(PAUSE_SIG,     (void *)0);
    QS_SIG_DICTIONARY(SERVE_SIG,     (void *)0);
    QS_SIG_DICTIONARY(TEST_SIG,      (void *)0);
    QS_SIG_DICTIONARY(TIMEOUT_SIG,   (void *)0);
    QS_SIG_DICTIONARY(HUNGRY_SIG,    AO_Table ); // signal just for Table

    // initialize publish-subscribe...
    static QSubscrList subscrSto[MAX_PUB_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    // NOTE: QF_poolInit() was called in BSP_init()

    // start the extended thread
    // NOTE: the ctor was called in BSP_init()
    static void const *test1QueueSto[5];
    QXTHREAD_START(XT_Test1,       // Thread to start
        1U,                        // QP priority of the thread
        test1QueueSto,             // event queue storage
        Q_DIM(test1QueueSto),      // event length [events]
        XT_Test1_stackSto,         // stack storage
        XT_Test1_stackSize,        // stack size [bytes]
        (void *)0);                // initialization param

    // NOTE: leave priority 2 free for a mutex

    // start the Philo active objects...
    static QEvt const *philoQueueSto[N_PHILO][N_PHILO];
    for (uint8_t n = 0U; n < N_PHILO; ++n) {
        // NOTE: the ctor was called in BSP_init()
        QACTIVE_START(AO_Philo[n],     // AO to start
            n + 3U,                    // QP priority of the AO
            philoQueueSto[n],          // event queue storage
            Q_DIM(philoQueueSto[n]),   // queue length [events]
            (void *)0,                 // stack storage (not used)
            0U,                        // size of the stack [bytes]
            (void *)0);                // initialization param
    }

    // example of prioritizing the Ticker0 active object
    // NOTE: the ctor was called in BSP_init()
    //QACTIVE_START(the_Ticker0, N_PHILO + 3U,
    //              0, 0, 0, 0, 0);

    // NOTE: leave priority (N_PHILO + 4) free for mutex

    // NOTE: the ctor was called in BSP_init()
    static void const *test2QueueSto[5];
    QXTHREAD_START(XT_Test2,       // Thread to start
        N_PHILO + 5U,              // QP priority of the thread
        test2QueueSto,             // event queue storage
        Q_DIM(test2QueueSto),      // event length [events]
        XT_Test2_stackSto,         // stack storage
        XT_Test2_stackSize,        // stack size [bytes]
        (void *)0);                // initialization param

    // NOTE: leave priority (N_PHILO + 6) free for mutex

    // NOTE: the ctor was called in BSP_init()
    static QEvt const *tableQueueSto[N_PHILO];
    QACTIVE_START(AO_Table,        // AO to start
        N_PHILO + 7U,              // QP priority of the AO
        tableQueueSto,             // event queue storage
        Q_DIM(tableQueueSto),      // queue length [events]
        (void *)0,                 // stack storage (not used)
        0U,                        // size of the stack [bytes]
        (void *)0);                // initialization param

    return QF_run(); // run the QF application
}

