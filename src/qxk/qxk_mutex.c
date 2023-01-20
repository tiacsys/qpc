//$file${src::qxk::qxk_mutex.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${src::qxk::qxk_mutex.c}
//
// This code has been generated by QM 5.3.0 <www.state-machine.com/qm>.
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This code is covered by the following QP license:
// License #    : LicenseRef-QL-dual
// Issued to    : Any user of the QP/C real-time embedded framework
// Framework(s) : qpc
// Support ends : 2023-12-31
// License scope:
//
// Copyright (C) 2005 Quantum Leaps, LLC <state-machine.com>.
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
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//
//$endhead${src::qxk::qxk_mutex.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//! @file
//! @brief ::QXMutex class definition.
#define QP_IMPL           // this is QP implementation
#include "qf_port.h"      // QF port
#include "qf_pkg.h"       // QF package-scope interface
#include "qassert.h"      // QP Functional Safety (FuSa) Subsystem
#ifdef Q_SPY              // QS software tracing enabled?
    #include "qs_port.h"  // QS port
    #include "qs_pkg.h"   // QS facilities for pre-defined trace records
#else
    #include "qs_dummy.h" // disable the QS software tracing
#endif // Q_SPY

// protection against including this source file in a wrong project
#ifndef QXK_H_
    #error "Source file included in a project NOT based on the QXK kernel"
#endif // QXK_H_

Q_DEFINE_THIS_MODULE("qxk_mutex")

//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 700U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.0.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QXK::QXMutex} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QXK::QXMutex} ............................................................

//${QXK::QXMutex::init} ......................................................
void QXMutex_init(QXMutex * const me,
    QPrioSpec const prioSpec)
{
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    Q_REQUIRE_NOCRIT_(100, (prioSpec & 0xFF00U) == 0U);

    me->ao.prio  = (uint8_t)(prioSpec & 0xFFU); // QF-priority
    me->ao.pthre = 0U;   // preemption-threshold (not used)
    QActive * const ao = &me->ao;

    QF_MEM_APP_();
    QF_CRIT_X_();

    QActive_register_(ao); // register this mutex as AO
}

//${QXK::QXMutex::lock} ......................................................
bool QXMutex_lock(QXMutex * const me,
    uint_fast16_t const nTicks)
{
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    QXThread * const curr = QXK_PTR_CAST_(QXThread*, QXK_priv_.curr);

    // precondition, this mutex operation must:
    // - NOT be called from an ISR;
    // - be called from an eXtended thread;
    // - the mutex-priority must be in range;
    // - the thread must NOT be already blocked on any object.
    Q_REQUIRE_NOCRIT_(200, (!QXK_ISR_CONTEXT_())
        && (curr != (QXThread *)0)
        && (me->ao.prio <= QF_MAX_ACTIVE)
        && (curr->super.super.temp.obj == (QMState *)0));
    // also: the thread must NOT be holding a scheduler lock.
    Q_REQUIRE_NOCRIT_(201, QXK_priv_.lockHolder != curr->super.prio);

    // is the mutex available?
    bool locked = true; // assume that the mutex will be locked
    if (me->ao.eQueue.nFree == 0U) {
        me->ao.eQueue.nFree = 1U; // mutex lock nesting

        // also: the newly locked mutex must have no holder yet
        Q_REQUIRE_NOCRIT_(202, me->ao.thread == (void *)0);

        // set the new mutex holder to the curr thread and
        // save the thread's prio in the mutex
        // NOTE: reuse the otherwise unused eQueue data member.
        me->ao.thread = curr;
        me->ao.eQueue.head = (QEQueueCtr)curr->super.prio;

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_LOCK, curr->super.prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
            QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
        QS_END_NOCRIT_PRE_()

        if (me->ao.prio != 0U) { // priority-ceiling protocol used?
            // the holder priority must be lower than that of the mutex
            // and the priority slot must be occupied by this mutex
            Q_ASSERT_NOCRIT_(210, (curr->super.prio < me->ao.prio)
                && (QActive_registry_[me->ao.prio] == &me->ao));

            // remove the thread's original prio from the ready set
            // and insert the mutex's prio into the ready set
            QPSet_remove(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.eQueue.head);
            QPSet_insert(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.prio);
    #ifndef Q_UNSAFE
            QPSet_update_(&QXK_priv_.readySet, &QXK_priv_.readySet_dis);
    #endif
            // put the thread into the AO registry in place of the mutex
            QActive_registry_[me->ao.prio] = &curr->super;

            // set thread's prio to that of the mutex
            curr->super.prio  = me->ao.prio;
        }
    }
    // is the mutex locked by this thread already (nested locking)?
    else if (me->ao.thread == &curr->super) {

        // the nesting level beyond the arbitrary but high limit
        // most likely means cyclic or recursive locking of a mutex.
        Q_ASSERT_NOCRIT_(220, me->ao.eQueue.nFree < 0xFFU);

        ++me->ao.eQueue.nFree; // lock one more level

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_LOCK, curr->super.prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
            QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
        QS_END_NOCRIT_PRE_()
    }
    else { // the mutex is already locked by a different thread
        // the mutex holder must be valid
        Q_ASSERT_NOCRIT_(230, me->ao.thread != (void *)0);

        if (me->ao.prio != 0U) { // priority-ceiling protocol used?
            // the prio slot must be occupied by the thr. holding the mutex
            Q_ASSERT_NOCRIT_(240, QActive_registry_[me->ao.prio]
                             == QACTIVE_CAST_(me->ao.thread));
        }

        // remove the curr thread's prio from the ready set (will block)
        // and insert it to the waiting set on this mutex
        uint_fast8_t const p = (uint_fast8_t)curr->super.prio;
        QPSet_remove(&QXK_priv_.readySet, p);
    #ifndef Q_UNSAFE
        QPSet_update_(&QXK_priv_.readySet, &QXK_priv_.readySet_dis);
    #endif
        QPSet_insert(&me->waitSet,  p);

        // set the blocking object (this mutex)
        curr->super.super.temp.obj = QXK_PTR_CAST_(QMState*, me);
        QXThread_teArm_(curr, (enum_t)QXK_TIMEOUT_SIG, nTicks);

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_BLOCK, curr->super.prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_2U8_PRE_((uint8_t)me->ao.eQueue.head, // holder prio
                        curr->super.prio); // blocked thread prio
        QS_END_NOCRIT_PRE_()

        // schedule the next thread if multitasking started
        (void)QXK_sched_(); // schedule other threads

        QF_MEM_APP_();
        QF_CRIT_X_();
        QF_CRIT_EXIT_NOP(); // BLOCK here !!!

        // AFTER unblocking...
        QF_CRIT_E_();
        QF_MEM_SYS_();
        // the blocking object must be this mutex
        Q_ASSERT_NOCRIT_(240, curr->super.super.temp.obj
                         == QXK_PTR_CAST_(QMState*, me));

        // did the blocking time-out? (signal of zero means that it did)
        if (curr->timeEvt.super.sig == 0U) {
            if (QPSet_hasElement(&me->waitSet, p)) { // still waiting?
                QPSet_remove(&me->waitSet, p); // remove unblocked thread
                locked = false; // the mutex was NOT locked
            }
        }
        else { // blocking did NOT time out
            // the thread must NOT be waiting on this mutex
            Q_ASSERT_NOCRIT_(250, !QPSet_hasElement(&me->waitSet, p));
        }
        curr->super.super.temp.obj = (QMState *)0; // clear blocking obj.
    }
    QF_MEM_APP_();
    QF_CRIT_X_();

    return locked;
}

//${QXK::QXMutex::tryLock} ...................................................
bool QXMutex_tryLock(QXMutex * const me) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    QActive *curr = QXK_priv_.curr;
    if (curr == (QActive *)0) { // called from a basic thread?
        curr = QActive_registry_[QXK_priv_.actPrio];
    }

    // precondition, this mutex must:
    // - NOT be called from an ISR;
    // - the calling thread must be valid;
    // - the mutex-priority must be in range
    Q_REQUIRE_NOCRIT_(300, (!QXK_ISR_CONTEXT_())
        && (curr != (QActive *)0)
        && (me->ao.prio <= QF_MAX_ACTIVE));
    // also: the thread must NOT be holding a scheduler lock.
    Q_REQUIRE_NOCRIT_(301, QXK_priv_.lockHolder != curr->prio);

    // is the mutex available?
    if (me->ao.eQueue.nFree == 0U) {
        me->ao.eQueue.nFree = 1U;  // mutex lock nesting

        // also the newly locked mutex must have no holder yet
        Q_REQUIRE_NOCRIT_(302, me->ao.thread == (void *)0);

        // set the new mutex holder to the curr thread and
        // save the thread's prio in the mutex
        // NOTE: reuse the otherwise unused eQueue data member.
        me->ao.thread = curr;
        me->ao.eQueue.head = (QEQueueCtr)curr->prio;

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_LOCK, curr->prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
            QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
        QS_END_NOCRIT_PRE_()

        if (me->ao.prio != 0U) { // priority-ceiling protocol used?
            // the holder priority must be lower than that of the mutex
            // and the priority slot must be occupied by this mutex
            Q_ASSERT_NOCRIT_(210, (curr->prio < me->ao.prio)
                && (QActive_registry_[me->ao.prio] == &me->ao));

            // remove the thread's original prio from the ready set
            // and insert the mutex's prio into the ready set
            QPSet_remove(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.eQueue.head);
            QPSet_insert(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.prio);
    #ifndef Q_UNSAFE
            QPSet_update_(&QXK_priv_.readySet, &QXK_priv_.readySet_dis);
    #endif
            // put the thread into the AO registry in place of the mutex
            QActive_registry_[me->ao.prio] = curr;

            // set thread's prio to that of the mutex
            curr->prio  = me->ao.prio;
        }
    }
    // is the mutex locked by this thread already (nested locking)?
    else if (me->ao.thread == curr) {
        // the nesting level must not exceed the specified limit
        Q_ASSERT_NOCRIT_(320, me->ao.eQueue.nFree < 0xFFU);

        ++me->ao.eQueue.nFree; // lock one more level

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_LOCK, curr->prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
            QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
        QS_END_NOCRIT_PRE_()
    }
    else { // the mutex is already locked by a different thread
        if (me->ao.prio != 0U) {  // priority-ceiling protocol used?
            // the prio slot must be occupied by the thr. holding the mutex
            Q_ASSERT_NOCRIT_(340, QActive_registry_[me->ao.prio]
                             == QACTIVE_CAST_(me->ao.thread));
        }

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_BLOCK_ATTEMPT, curr->prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_2U8_PRE_((uint8_t)me->ao.eQueue.head, // holder prio
                        curr->prio); // trying thread prio
        QS_END_NOCRIT_PRE_()

        curr = (QActive *)0; // means that mutex is NOT available
    }
    QF_MEM_APP_();
    QF_CRIT_X_();

    return curr != (QActive *)0;
}

//${QXK::QXMutex::unlock} ....................................................
void QXMutex_unlock(QXMutex * const me) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    QActive *curr = QXK_priv_.curr;
    if (curr == (QActive *)0) { // called from a basic thread?
        curr = QActive_registry_[QXK_priv_.actPrio];
    }

    Q_REQUIRE_NOCRIT_(400, (!QXK_ISR_CONTEXT_())
        && (curr != (QActive *)0));
    Q_REQUIRE_NOCRIT_(401, me->ao.eQueue.nFree > 0U);
    Q_REQUIRE_NOCRIT_(402, me->ao.thread == curr);

    // is this the last nesting level?
    if (me->ao.eQueue.nFree == 1U) {

        if (me->ao.prio != 0U) { // priority-ceiling protocol used?

            // restore the holding thread's prio from the mutex
            curr->prio  = (uint8_t)me->ao.eQueue.head;

            // put the mutex back into the AO registry
            QActive_registry_[me->ao.prio] = &me->ao;

            // remove the mutex' prio from the ready set
            // and insert the original thread's priority
            QPSet_remove(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.prio);
            QPSet_insert(&QXK_priv_.readySet,
                         (uint_fast8_t)me->ao.eQueue.head);
    #ifndef Q_UNSAFE
            QPSet_update_(&QXK_priv_.readySet, &QXK_priv_.readySet_dis);
    #endif
        }

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_UNLOCK, curr->prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_2U8_PRE_((uint8_t)me->ao.eQueue.head, // holder prio
                        0U); // nesting
        QS_END_NOCRIT_PRE_()

        // are any other threads waiting on this mutex?
        if (QPSet_notEmpty(&me->waitSet)) {
            // find the highest-priority thread waiting on this mutex
            uint_fast8_t const p = QPSet_findMax(&me->waitSet);

            // remove this thread from waiting on the mutex
            // and insert it into the ready set.
            QPSet_remove(&me->waitSet,  p);
            QPSet_insert(&QXK_priv_.readySet, p);
    #ifndef Q_UNSAFE
            QPSet_update_(&QXK_priv_.readySet, &QXK_priv_.readySet_dis);
    #endif

            QXThread * const thr =
                QXK_PTR_CAST_(QXThread*, QActive_registry_[p]);

            // the waiting thread must:
            // - be registered in QF
            // - have the priority corresponding to the registration
            // - be an extended thread
            // - be blocked on this mutex
            Q_ASSERT_NOCRIT_(410, (thr != (QXThread *)0)
                && (thr->super.prio == (uint8_t)p)
                && (thr->super.super.state.act == Q_ACTION_CAST(0))
                && (thr->super.super.temp.obj
                    == QXK_PTR_CAST_(QMState*, me)));

            // disarm the internal time event
            (void)QXThread_teDisarm_(thr);

            // set the new mutex holder to the curr thread and
            // save the thread's prio in the mutex
            // NOTE: reuse the otherwise unused eQueue data member.
            me->ao.thread = thr;
            me->ao.eQueue.head = (QEQueueCtr)thr->super.prio;

            QS_BEGIN_NOCRIT_PRE_(QS_MTX_LOCK, thr->super.prio)
                QS_TIME_PRE_();  // timestamp
                QS_OBJ_PRE_(me); // this mutex
                QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
                QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
            QS_END_NOCRIT_PRE_()

            if (me->ao.prio != 0U) { // priority-ceiling protocol used?
                // the holder priority must be lower than that of the mutex
                Q_ASSERT_NOCRIT_(410, thr->super.prio < me->ao.prio);

                // put the thread into AO registry in place of the mutex
                QActive_registry_[me->ao.prio] = &thr->super;
            }
        }
        else { // no threads are waiting for this mutex
            me->ao.eQueue.nFree = 0U; // free up the nesting count

            // the mutex no longer held by any thread
            me->ao.thread = (void *)0;
            me->ao.eQueue.head = 0U;

            if (me->ao.prio != 0U) { // priority-ceiling protocol used?
                // put the mutex back at the original mutex slot
                QActive_registry_[me->ao.prio] =
                    QXK_PTR_CAST_(QActive*, me);
            }
        }

        // schedule the next thread if multitasking started
        if (QXK_sched_() != 0U) { // activation needed?
            QXK_activate_(); // synchronously activate basic-thred(s)
        }
    }
    else { // releasing one level of nested mutex lock
        Q_ASSERT_NOCRIT_(420, me->ao.eQueue.nFree > 0U);
        --me->ao.eQueue.nFree; // unlock one level

        QS_BEGIN_NOCRIT_PRE_(QS_MTX_UNLOCK_ATTEMPT, curr->prio)
            QS_TIME_PRE_();  // timestamp
            QS_OBJ_PRE_(me); // this mutex
            QS_U8_PRE_((uint8_t)me->ao.eQueue.head); // holder prio
            QS_U8_PRE_((uint8_t)me->ao.eQueue.nFree); // nesting
        QS_END_NOCRIT_PRE_()
    }
    QF_MEM_APP_();
    QF_CRIT_X_();
}
//$enddef${QXK::QXMutex} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
