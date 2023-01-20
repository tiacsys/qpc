//============================================================================
// Product: DPP example
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
// along with this program. If not, see <www.gnu.org/licenses>.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
// ============================================================================
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

// local communication mechanisms ..........................................
static QXMutex l_mutex;
static QXSemaphore l_sema;

//----------------------------------------------------------------------------
// Test1 eXtended thread...
static QXThread l_test1;
QXThread * const XT_Test1 = &l_test1; // global opaque pointer

//............................................................................
static void Thread1_run(QXThread * const me) {

    QS_OBJ_DICTIONARY(XT_Test1);
    QS_OBJ_DICTIONARY(&XT_Test1->timeEvt);

    // subscribe to the EAT signal (from the DPP part of the application)
    QActive_subscribe(&me->super, EAT_SIG);

    for (;;) {
        QEvt const *e = QXThread_queueGet(BSP_TICKS_PER_SEC/4U);
        if (e != (QEvt *)0) {
            (void)QXSemaphore_signal(&l_sema); // signal Thread2
            QF_gc(e); // must explicitly recycle the received event!
        }

        QXMutex_lock(&l_mutex, QXTHREAD_NO_TIMEOUT); // lock the mutex
        BSP_ledOn();
        if (QXMutex_tryLock(&l_mutex)) { // nested mutex lock
            QXThread_delay(1U);  // BLOCK while holding a mutex
            QXMutex_unlock(&l_mutex);
        }
        QXMutex_unlock(&l_mutex);
        BSP_ledOff();
    }
}
//............................................................................
void Test1_ctor(void) {
    QXThread_ctor(&l_test1, &Thread1_run, 0U);
}

//----------------------------------------------------------------------------
// Test2 eXtended thread...
static QXThread l_test2;
QXThread * const XT_Test2 = &l_test2;  // global opaque pointer

//............................................................................
static void Thread2_run(QXThread * const me) {

    QS_OBJ_DICTIONARY(XT_Test2);
    QS_OBJ_DICTIONARY(&XT_Test2->timeEvt);
    QS_OBJ_DICTIONARY(&l_sema);
    QS_OBJ_DICTIONARY(&l_mutex);

    // initialize the semaphore before using it
    // NOTE: the semaphore is initialized in the highest-priority thread
    // that uses it. Alternatively, the semaphore can be initialized
    // before any thread runs.
    //
    QXSemaphore_init(&l_sema,
                     0U,  // count==0 (signaling semaphore)
                     1U); // max_count==1 (binary semaphore)

    // initialize the mutex before using it
    // NOTE: Here the mutex is initialized in the highest-priority thread
    // that uses it. Alternatively, the mutex can be initialized
    // before any thread runs.
    //
    //QXMutex_init(&l_mutex, 0U); // priority-ceiling NOT used
    QXMutex_init(&l_mutex, N_PHILO + 6U); //priority-ceiling protocol used

    for (;;) {
        // wait on a semaphore (BLOCK indefinitely)
        QXSemaphore_wait(&l_sema, QXTHREAD_NO_TIMEOUT);

        QXMutex_lock(&l_mutex, QXTHREAD_NO_TIMEOUT); // lock the mutex
        QXThread_delay(5U);  // BLOCK while holding a mutex
        QXMutex_unlock(&l_mutex);
    }
}
//............................................................................
void Test2_ctor(void) {
    QXThread_ctor(&l_test2, &Thread2_run, 0U);
}
