//============================================================================
// Last updated for: @ref qpc_7_0_0
// Last updated on  2021-12-05
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

// Local-scope objects -----------------------------------------------------
static QEvt const *l_tableQueueSto[N_PHILO];
static QEvt const *l_philoQueueSto[N_PHILO][N_PHILO];
static QSubscrList l_subscrSto[MAX_PUB_SIG];

static union SmallEvents {
    void   *e0; // minimum event size
    uint8_t e1[sizeof(TableEvt)];
    // ... other event types to go into this pool
} l_smlPoolSto[2*N_PHILO + 10]; // storage for the small event pool

static ULONG l_philoStk[N_PHILO][256]; // stacks for the Philosophers
static ULONG l_tableStk[256];          // stack for the Table

//............................................................................
int main() {
    Philo_ctor(); // instantiate all Philosopher active objects
    Table_ctor(); // instantiate the Table active object
    tx_kernel_enter();
    return 0; // tx_kernel_enter() does not return
}
//............................................................................
void tx_application_define(void *first_unused_memory) {
    uint8_t n;

    (void)first_unused_memory; // unused parameter

    BSP_init(); // initialize the Board Support Package
    QF_init(); // initialize the framework and the underlying RT kernel

    QActive_psInit(l_subscrSto, Q_DIM(l_subscrSto)); // init publish-subscribe

    // initialize event pools...
    QF_poolInit(l_smlPoolSto, sizeof(l_smlPoolSto), sizeof(l_smlPoolSto[0]));

    QS_OBJ_DICTIONARY(l_smlPoolSto);
    QS_OBJ_DICTIONARY(AO_Table);
    QS_OBJ_DICTIONARY(AO_Philo[0]);
    QS_OBJ_DICTIONARY(AO_Philo[1]);
    QS_OBJ_DICTIONARY(AO_Philo[2]);
    QS_OBJ_DICTIONARY(AO_Philo[3]);
    QS_OBJ_DICTIONARY(AO_Philo[4]);

    for (n = 0; n < N_PHILO; ++n) { // start the active objects...
        QActive_setAttr(AO_Philo[n], THREAD_NAME_ATTR, "Philo");
        QACTIVE_START(AO_Philo[n], (uint8_t)(n + 1),
                      l_philoQueueSto[n], Q_DIM(l_philoQueueSto[n]),
                      l_philoStk[n], sizeof(l_philoStk[n]), (QEvt *)0);
    }

    QActive_setAttr(AO_Table, THREAD_NAME_ATTR, "Table");
    QACTIVE_START(AO_Table, (uint8_t)(N_PHILO + 1),
                  l_tableQueueSto, Q_DIM(l_tableQueueSto),
                  l_tableStk, sizeof(l_tableStk), (QEvt *)0);

    (void)QF_run();
}
