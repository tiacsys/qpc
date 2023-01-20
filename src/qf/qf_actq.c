//$file${src::qf::qf_actq.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${src::qf::qf_actq.c}
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
//$endhead${src::qf::qf_actq.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//! @file
//! @brief ::QActive native queue operations (based on ::QEQueue)
//!
//! @note
//! This `qf_actq.c` source file needs to be included in the application
//! build only when the native ::QEQueue queue is used for ::QActive objects
//! (instead of a message queue of an RTOS).
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

Q_DEFINE_THIS_MODULE("qf_actq")

//============================================================================
//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 700U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.0.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QF::QActive::post_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::post_} ......................................................
bool QActive_post_(QActive * const me,
    QEvt const * const e,
    uint_fast16_t const margin,
    void const * const sender)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(sender);
    #endif

    #ifdef Q_UTEST // test?
    #if Q_UTEST != 0 // testing QP-stub?
    if (me->super.temp.fun == Q_STATE_CAST(0)) { // QActiveDummy?
        return QActiveDummy_post_(me, e, margin, sender);
    }
    #endif
    #endif

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    Q_REQUIRE_NOCRIT_(102, QEvt_verify_(e));

    QEQueueCtr nFree = me->eQueue.nFree; // get volatile into temporary

    // test-probe#1 for faking queue overflow
    QS_TEST_PROBE_DEF(&QActive_post_)
    QS_TEST_PROBE_ID(1,
        nFree = 0U;
    )

    bool status;
    if (margin == QF_NO_MARGIN) {
        if (nFree > 0U) {
            status = true; // can post
        }
        else {
            status = false; // cannot post
            Q_ERROR_NOCRIT_(190); // must be able to post the event
        }
    }
    else if (nFree > (QEQueueCtr)margin) {
        status = true; // can post
    }
    else {
        status = false; // cannot post, but don't assert
    }

    // is it a mutable event?
    if (QEVT_POOLID_(e) != 0U) {
        QEvt_refCtr_inc_(e); // increment the reference counter
    }

    if (status) { // can post the event?

        --nFree; // one free entry just used up
        me->eQueue.nFree = nFree; // update the original
        if (me->eQueue.nMin > nFree) {
            me->eQueue.nMin = nFree; // increase minimum so far
        }

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST, me->prio)
            QS_TIME_PRE_();       // timestamp
            QS_OBJ_PRE_(sender);  // the sender object
            QS_SIG_PRE_(e->sig);  // the signal of the event
            QS_OBJ_PRE_(me);      // this active object (recipient)
            QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_); // poolId & refCount
            QS_EQC_PRE_(nFree);   // # free entries
            QS_EQC_PRE_(me->eQueue.nMin); // min # free entries
        QS_END_NOCRIT_PRE_()

    #ifdef Q_UTEST
        // callback to examine the posted event under the same conditions
        // as producing the #QS_QF_ACTIVE_POST trace record, which are:
        // the local filter for this AO ('me->prio') is set
        if (QS_LOC_CHECK_(me->prio)) {
            // callback to examine the posted event
            QS_onTestPost(sender, me, e, status);
        }
    #endif

        // empty queue?
        if (me->eQueue.frontEvt == (QEvt *)0) {
            me->eQueue.frontEvt = e;    // deliver event directly

    #ifdef QXK_H_
            if (me->super.state.act == Q_ACTION_CAST(0)) { // eXtended?
                QXTHREAD_EQUEUE_SIGNAL_(me); // signal the event queue
            }
            else {
                QACTIVE_EQUEUE_SIGNAL_(me); // signal the event queue
            }
    #else
            QACTIVE_EQUEUE_SIGNAL_(me); // signal the event queue
    #endif
        }
        // queue is not empty, insert event into the ring-buffer
        else {
            // insert event into the ring buffer (FIFO)
            me->eQueue.ring[me->eQueue.head] = e;

            if (me->eQueue.head == 0U) { // need to wrap head?
                me->eQueue.head = me->eQueue.end;   // wrap around
            }
            --me->eQueue.head; // advance the head (counter clockwise)
        }

        QF_MEM_APP_();
        QF_CRIT_X_();
    }
    else { // cannot post the event

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST_ATTEMPT, me->prio)
            QS_TIME_PRE_();       // timestamp
            QS_OBJ_PRE_(sender);  // the sender object
            QS_SIG_PRE_(e->sig);  // the signal of the event
            QS_OBJ_PRE_(me);      // this active object (recipient)
            QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_); // poolId & refCount
            QS_EQC_PRE_(nFree);   // # free entries
            QS_EQC_PRE_(margin);  // margin requested
        QS_END_NOCRIT_PRE_()

    #ifdef Q_UTEST
        // callback to examine the posted event under the same conditions
        // as producing the #QS_QF_ACTIVE_POST trace record, which are:
        // the local filter for this AO ('me->prio') is set
        if (QS_LOC_CHECK_(me->prio)) {
            QS_onTestPost(sender, me, e, status);
        }
    #endif

        QF_MEM_APP_();
        QF_CRIT_X_();

    #if (QF_MAX_EPOOL > 0U)
        QF_gc(e); // recycle the event to avoid a leak
    #endif
    }

    return status;
}
//$enddef${QF::QActive::post_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::postLIFO_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::postLIFO_} ..................................................
void QActive_postLIFO_(QActive * const me,
    QEvt const * const e)
{
    #ifdef Q_UTEST // test?
    #if Q_UTEST != 0 // testing QP-stub?
    if (me->super.temp.fun == Q_STATE_CAST(0)) { // QActiveDummy?
        QActiveDummy_postLIFO_(me, e);
        return;
    }
    #endif
    #endif

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    #ifdef QXK_H_
    Q_REQUIRE_NOCRIT_(200, me->super.state.act != Q_ACTION_CAST(0));
    #endif

    QEQueueCtr nFree = me->eQueue.nFree; // get volatile into temporary

    // test-probe#1 for faking queue overflow
    QS_TEST_PROBE_DEF(&QActive_postLIFO_)
    QS_TEST_PROBE_ID(1,
        nFree = 0U;
    )

    Q_REQUIRE_NOCRIT_(201, nFree != 0U);

    // is it a mutable event?
    if (QEVT_POOLID_(e) != 0U) {
        QEvt_refCtr_inc_(e); // increment the reference counter
    }

    --nFree; // one free entry just used up
    me->eQueue.nFree = nFree; // update the original
    if (me->eQueue.nMin > nFree) {
        me->eQueue.nMin = nFree; // update minimum so far
    }

    QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST_LIFO, me->prio)
        QS_TIME_PRE_();      // timestamp
        QS_SIG_PRE_(e->sig); // the signal of this event
        QS_OBJ_PRE_(me);     // this active object
        QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_);// poolId & refCount
        QS_EQC_PRE_(nFree);  // # free entries
        QS_EQC_PRE_(me->eQueue.nMin); // min # free entries
    QS_END_NOCRIT_PRE_()

    #ifdef Q_UTEST
    // callback to examine the posted event under the same conditions
    // as producing the #QS_QF_ACTIVE_POST trace record, which are:
    // the local filter for this AO ('me->prio') is set
    if (QS_LOC_CHECK_(me->prio)) {
        QS_onTestPost((QActive *)0, me, e, true);
    }
    #endif

    QEvt const * const frontEvt  = me->eQueue.frontEvt;
    me->eQueue.frontEvt = e; // deliver the event directly to the front

    // was the queue empty?
    if (frontEvt == (QEvt *)0) {
        QACTIVE_EQUEUE_SIGNAL_(me); // signal the event queue
    }
    // queue was not empty, leave the event in the ring-buffer
    else {
        ++me->eQueue.tail;
        // need to wrap the tail?
        if (me->eQueue.tail == me->eQueue.end) {
            me->eQueue.tail = 0U; // wrap around
        }

        me->eQueue.ring[me->eQueue.tail] = frontEvt;
    }

    QF_MEM_APP_();
    QF_CRIT_X_();
}
//$enddef${QF::QActive::postLIFO_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QActive::get_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QActive::get_} .......................................................
QEvt const * QActive_get_(QActive * const me) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    QACTIVE_EQUEUE_WAIT_(me);  // wait for event to arrive directly

    // always remove event from the front
    QEvt const * const e = me->eQueue.frontEvt;
    QEQueueCtr const nFree = me->eQueue.nFree + 1U; // get volatile into tmp
    me->eQueue.nFree = nFree; // update the # free

    // any events in the ring buffer?
    if (nFree <= me->eQueue.end) {

        // remove event from the tail
        me->eQueue.frontEvt = me->eQueue.ring[me->eQueue.tail];
        if (me->eQueue.tail == 0U) { // need to wrap the tail?
            me->eQueue.tail = me->eQueue.end;   // wrap around
        }
        --me->eQueue.tail;

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_GET, me->prio)
            QS_TIME_PRE_();      // timestamp
            QS_SIG_PRE_(e->sig); // the signal of this event
            QS_OBJ_PRE_(me);     // this active object
            QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_); // poolId & refCount
            QS_EQC_PRE_(nFree);  // # free entries
        QS_END_NOCRIT_PRE_()
    }
    else {
        me->eQueue.frontEvt = (QEvt *)0; // queue becomes empty

        // all entries in the queue must be free (+1 for fronEvt)
        Q_ASSERT_NOCRIT_(310, nFree == (me->eQueue.end + 1U));

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_GET_LAST, me->prio)
            QS_TIME_PRE_();      // timestamp
            QS_SIG_PRE_(e->sig); // the signal of this event
            QS_OBJ_PRE_(me);     // this active object
            QS_2U8_PRE_(QEVT_POOLID_(e), e->refCtr_); // poolId & refCount
        QS_END_NOCRIT_PRE_()
    }

    QF_MEM_APP_();
    QF_CRIT_X_();

    return e;
}
//$enddef${QF::QActive::get_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QF::QF-base::getQueueMin} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QF-base::getQueueMin} ................................................
uint_fast16_t QF_getQueueMin(uint_fast8_t const prio) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    Q_REQUIRE_NOCRIT_(400, (prio <= QF_MAX_ACTIVE)
                      && (QActive_registry_[prio] != (QActive *)0));
    uint_fast16_t const min =
         (uint_fast16_t)QActive_registry_[prio]->eQueue.nMin;
    QF_CRIT_X_();

    return min;
}
//$enddef${QF::QF-base::getQueueMin} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${QF::QTicker} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QTicker} .............................................................

//${QF::QTicker::ctor} .......................................................
void QTicker_ctor(QTicker * const me,
    uint_fast8_t const tickRate)
{
    static struct QHsmVtable const vtable = { // QTicker virtual table
        &QTicker_init_,
        &QTicker_dispatch_
    #ifdef Q_SPY
        ,&QHsm_getStateHandler_
    #endif
    };

    QActive_ctor(&me->super, Q_STATE_CAST(0)); // superclass' ctor
    me->super.super.vptr = &vtable; // hook the vptr

    // reuse eQueue.head for tick-rate
    me->super.eQueue.head = (QEQueueCtr)tickRate;
}

//${QF::QTicker::init_} ......................................................
void QTicker_init_(
    QHsm * const me,
    void const * const par,
    uint_fast8_t const qs_id)
{
    Q_UNUSED_PAR(me);
    Q_UNUSED_PAR(par);
    Q_UNUSED_PAR(qs_id);

    QACTIVE_CAST_(me)->eQueue.tail = 0U;
}

//${QF::QTicker::dispatch_} ..................................................
void QTicker_dispatch_(
    QHsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id)
{
    Q_UNUSED_PAR(e);
    Q_UNUSED_PAR(qs_id);

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QEQueueCtr nTicks = QACTIVE_CAST_(me)->eQueue.tail; // save # of ticks
    QACTIVE_CAST_(me)->eQueue.tail = 0U; // clear # ticks
    QF_CRIT_X_();

    for (; nTicks > 0U; --nTicks) {
        QTimeEvt_tick_((uint_fast8_t)QACTIVE_CAST_(me)->eQueue.head, me);
    }
}

//${QF::QTicker::tick_} ......................................................
void QTicker_tick_(
    QActive * const me,
    void const * const sender)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(sender);
    #endif

    QF_CRIT_STAT_
    QF_CRIT_E_();
    QF_MEM_SYS_();

    if (me->eQueue.frontEvt == (QEvt *)0) {

        static QEvt const tickEvt = QEVT_INITIALIZER(0);
        me->eQueue.frontEvt = &tickEvt; // deliver event directly
        --me->eQueue.nFree; // one less free event

        QACTIVE_EQUEUE_SIGNAL_(me); // signal the event queue
    }

    ++me->eQueue.tail; // account for one more tick event

    QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST, me->prio)
        QS_TIME_PRE_();      // timestamp
        QS_OBJ_PRE_(sender); // the sender object
        QS_SIG_PRE_(0U);     // the signal of the event
        QS_OBJ_PRE_(me);     // this active object
        QS_2U8_PRE_(0U, 0U); // pool Id & refCtr of the evt
        QS_EQC_PRE_(0U);     // # free entries
        QS_EQC_PRE_(0U);     // min # free entries
    QS_END_NOCRIT_PRE_()

    QF_MEM_APP_();
    QF_CRIT_X_();
}
//$enddef${QF::QTicker} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
