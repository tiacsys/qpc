//$file${src::qf::qf_ps.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${src::qf::qf_ps.c}
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
//$endhead${src::qf::qf_ps.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//! @file
//! @brief ::QActive Publish-Subscribe services
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

Q_DEFINE_THIS_MODULE("qf_ps")

//============================================================================
//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 700U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.0.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QF::QActive::subscrList_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
QSubscrList * QActive_subscrList_;
//$enddef${QF::QActive::subscrList_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::maxPubSignal_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
enum_t QActive_maxPubSignal_;
//$enddef${QF::QActive::maxPubSignal_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QF::QActive::psInit} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::psInit} .....................................................
//! @static @public @memberof QActive
void QActive_psInit(
    QSubscrList * const subscrSto,
    enum_t const maxSignal)
{
    QActive_subscrList_   = subscrSto;
    QActive_maxPubSignal_ = maxSignal;

    // initialize the subscriber list
    for (enum_t sig = 0; sig < maxSignal; ++sig) {
        QPSet_setEmpty(&subscrSto[sig].set);
    #ifndef Q_UNSAFE
        QPSet_update_(&subscrSto[sig].set, &subscrSto[sig].set_dis);
    #endif
    }
}
//$enddef${QF::QActive::psInit} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::publish_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::publish_} ...................................................
//! @static @private @memberof QActive
void QActive_publish_(
    QEvt const * const e,
    void const * const sender,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(sender);
    Q_UNUSED_PAR(qs_id);
    #endif

    QSignal const sig = e->sig;

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    Q_REQUIRE_NOCRIT_(200, sig < (QSignal)QActive_maxPubSignal_);
    Q_REQUIRE_NOCRIT_(202,
        QPSet_verify_(&QActive_subscrList_[sig].set,
                      &QActive_subscrList_[sig].set_dis));

    QS_BEGIN_PRE_(QS_QF_PUBLISH, qs_id)
        QS_TIME_PRE_();          // the timestamp
        QS_OBJ_PRE_(sender);     // the sender object
        QS_SIG_PRE_(sig);        // the signal of the event
        QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_); // poolId & refCtr
    QS_END_PRE_()

    // is it a mutable event?
    if (QEVT_POOLID_(e) != 0U) {
        // NOTE: The reference counter of a mutable event is incremented to
        // prevent premature recycling of the event while the multicasting
        // is still in progress. At the end of the function, the garbage
        // collector step (QF_gc()) decrements the reference counter and
        // recycles the event if the counter drops to zero. This covers the
        // case when the event was published without any subscribers.
        QEvt_refCtr_inc_(e);
    }

    // make a local, modifiable copy of the subscriber set
    QPSet subscrSet = QActive_subscrList_[sig].set;

    QF_MEM_APP_();
    QF_CRIT_X_();

    if (QPSet_notEmpty(&subscrSet)) { // any subscribers?
        // highest-prio subscriber
        uint_fast8_t p = QPSet_findMax(&subscrSet);

        QF_CRIT_E_();
        QF_MEM_SYS_();

        QActive *a = QActive_registry_[p];
        // the AO must be registered with the framework
        Q_ASSERT_NOCRIT_(210, a != (QActive *)0);

        QF_MEM_APP_();
        QF_CRIT_X_();

        QF_SCHED_STAT_
        QF_SCHED_LOCK_(p); // lock the scheduler up to AO's prio
        do { // loop over all subscribers

            // QACTIVE_POST() asserts internally if the queue overflows
            QACTIVE_POST(a, e, sender);

            QPSet_remove(&subscrSet, p); // remove the handled subscriber
            if (QPSet_notEmpty(&subscrSet)) {  // still more subscribers?
                p = QPSet_findMax(&subscrSet); // highest-prio subscriber

                QF_CRIT_E_();
                QF_MEM_SYS_();

                a = QActive_registry_[p];
                // the AO must be registered with the framework
                Q_ASSERT_NOCRIT_(220, a != (QActive *)0);

                QF_MEM_APP_();
                QF_CRIT_X_();
            }
            else {
                p = 0U; // no more subscribers
            }
        } while (p != 0U);
        QF_SCHED_UNLOCK_(); // unlock the scheduler
    }

    // The following garbage collection step decrements the reference counter
    // and recycles the event if the counter drops to zero. This covers both
    // cases when the event was published with or without any subscribers.
    #if (QF_MAX_EPOOL > 0U)
    QF_gc(e); // recycle the event to avoid a leak
    #endif
}
//$enddef${QF::QActive::publish_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::subscribe} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::subscribe} ..................................................
//! @protected @memberof QActive
void QActive_subscribe(QActive const * const me,
    enum_t const sig)
{
    uint_fast8_t const p = (uint_fast8_t)me->prio;

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    Q_REQUIRE_NOCRIT_(300, ((enum_t)Q_USER_SIG <= sig)
        && (sig < QActive_maxPubSignal_)
        && (0U < p) && (p <= QF_MAX_ACTIVE)
        && (QActive_registry_[p] == me));
    Q_REQUIRE_NOCRIT_(302,
        QPSet_verify_(&QActive_subscrList_[sig].set,
                      &QActive_subscrList_[sig].set_dis));

    QS_BEGIN_PRE_(QS_QF_ACTIVE_SUBSCRIBE, p)
        QS_TIME_PRE_();    // timestamp
        QS_SIG_PRE_(sig);  // the signal of this event
        QS_OBJ_PRE_(me);   // this active object
    QS_END_PRE_()

    // insert the priority into the subscriber set
    QPSet_insert(&QActive_subscrList_[sig].set, p);
    #ifndef Q_UNSAFE
    QPSet_update_(&QActive_subscrList_[sig].set,
                  &QActive_subscrList_[sig].set_dis);
    #endif

    QF_MEM_APP_();
    QF_CRIT_X_();
}
//$enddef${QF::QActive::subscribe} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::unsubscribe} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::unsubscribe} ................................................
//! @protected @memberof QActive
void QActive_unsubscribe(QActive const * const me,
    enum_t const sig)
{
    uint_fast8_t const p = (uint_fast8_t)me->prio;

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    Q_REQUIRE_NOCRIT_(400, ((enum_t)Q_USER_SIG <= sig)
        && (sig < QActive_maxPubSignal_)
        && (0U < p) && (p <= QF_MAX_ACTIVE)
        && (QActive_registry_[p] == me));
    Q_REQUIRE_NOCRIT_(402,
        QPSet_verify_(&QActive_subscrList_[sig].set,
                      &QActive_subscrList_[sig].set_dis));

    QS_BEGIN_PRE_(QS_QF_ACTIVE_UNSUBSCRIBE, p)
        QS_TIME_PRE_();    // timestamp
        QS_SIG_PRE_(sig);  // the signal of this event
        QS_OBJ_PRE_(me);   // this active object
    QS_END_PRE_()

    // remove the priority from the subscriber set
    QPSet_remove(&QActive_subscrList_[sig].set, p);
    #ifndef Q_UNSAFE
    QPSet_update_(&QActive_subscrList_[sig].set,
                  &QActive_subscrList_[sig].set_dis);
    #endif

    QF_MEM_APP_();
    QF_CRIT_X_();
}
//$enddef${QF::QActive::unsubscribe} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::unsubscribeAll} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::unsubscribeAll} .............................................
//! @protected @memberof QActive
void QActive_unsubscribeAll(QActive const * const me) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    uint_fast8_t const p = (uint_fast8_t)me->prio;

    Q_REQUIRE_NOCRIT_(500, (0U < p) && (p <= QF_MAX_ACTIVE)
                           && (QActive_registry_[p] == me));
    enum_t const maxPubSig = QActive_maxPubSignal_;

    QF_MEM_APP_();
    QF_CRIT_X_();

    for (enum_t sig = (enum_t)Q_USER_SIG; sig < maxPubSig; ++sig) {
        QF_CRIT_E_();
        QF_MEM_SYS_();

        if (QPSet_hasElement(&QActive_subscrList_[sig].set, p)) {
            QPSet_remove(&QActive_subscrList_[sig].set, p);
    #ifndef Q_UNSAFE
            QPSet_update_(&QActive_subscrList_[sig].set,
                          &QActive_subscrList_[sig].set_dis);
    #endif
            QS_BEGIN_PRE_(QS_QF_ACTIVE_UNSUBSCRIBE, p)
                QS_TIME_PRE_();    // timestamp
                QS_SIG_PRE_(sig);  // the signal of this event
                QS_OBJ_PRE_(me);   // this active object
            QS_END_PRE_()
        }
        QF_MEM_APP_();
        QF_CRIT_X_();

        QF_CRIT_EXIT_NOP(); // prevent merging critical sections
    }
}
//$enddef${QF::QActive::unsubscribeAll} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
