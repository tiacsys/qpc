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
//============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

//Q_DEFINE_THIS_FILE

static QTicker l_ticker0;
QActive * const the_Ticker0 = &l_ticker0.super;

//............................................................................
int main() {
    static QEvt const *tableQueueSto[N_PHILO];
    static QEvt const *philoQueueSto[N_PHILO][N_PHILO];
    static QSubscrList subscrSto[MAX_PUB_SIG];
    static QF_MPOOL_EL(TableEvt) smlPoolSto[2*N_PHILO]; // small pool

    // stacks and queues for the extended test threads
    static void const *test1QueueSto[5];
    static uint64_t test1StackSto[64];
    static void const *test2QueueSto[5];
    static uint64_t test2StackSto[64];

    QF_init();    // initialize the framework and the underlying RT kernel
    BSP_init();   // initialize the Board Support Package

    // initialize publish-subscribe...
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    // initialize event pools...
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

    // start the extended thread
    Test1_ctor(); // instantiate the Test1 extended thread
    QXTHREAD_START(XT_Test1,            // Thread to start
                  1U,                   // QF-prio/pre-thre.
                  test1QueueSto,        // message queue storage
                  Q_DIM(test1QueueSto), // message length [events]
                  test1StackSto,        // stack storage
                  sizeof(test1StackSto),// stack size [bytes]
                  (void *)0);           // initialization param

    // NOTE: leave priority 2 free for a mutex

    // start the Philo active objects...
    Philo_ctor(); // instantiate all Philo AOs
    for (uint8_t n = 0U; n < N_PHILO; ++n) {
        QACTIVE_START(AO_Philo[n],       // AO to start
                      n + 3U,            // QF-prio/pre-thre.
                      philoQueueSto[n],  // event queue storage
                      Q_DIM(philoQueueSto[n]), // queue length [events]
                      (void *)0,         // stack storage (not used)
                      0U,                // size of the stack [bytes]
                      (void *)0);        // initialization param
    }

    // example of prioritizing the Ticker0 active object
    QTicker_ctor(&l_ticker0, 0U); // ticker AO for tick rate 0
    QACTIVE_START(the_Ticker0,
                  N_PHILO + 3U,          // QF-prio/pre-thre.
                  0, 0U, 0, 0U, (void *)0);

    // NOTE: leave priority (N_PHILO + 4) free for mutex

    Test2_ctor(); // instantiate the Test2 extended thread
    QXTHREAD_START(XT_Test2,             // Thread to start
                  N_PHILO + 5U,          // QF-prio/pre-thre.
                  test2QueueSto,         // message queue storage
                  Q_DIM(test2QueueSto),  // message length [events]
                  test2StackSto,         // stack storage
                  sizeof(test2StackSto), // stack size [bytes]
                  (void *)0);            // initialization param

    // NOTE: leave priority (N_PHILO + 6) free for mutex

    Table_ctor(); // instantiate the Table active object
    QACTIVE_START(AO_Table,              // AO to start
                  N_PHILO + 7U,          // QF-prio/pre-thre.
                  tableQueueSto,         // event queue storage
                  Q_DIM(tableQueueSto),  // queue length [events]
                  (void *)0,             // stack storage (not used)
                  0U,                    // size of the stack [bytes]
                  (void *)0);            // initialization param

    return QF_run(); // run the QF application
}

