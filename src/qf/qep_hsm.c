//$file${src::qf::qep_hsm.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${src::qf::qep_hsm.c}
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
//$endhead${src::qf::qep_hsm.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//! @file
//! @brief ::QHsm implementation
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

Q_DEFINE_THIS_MODULE("qep_hsm")

//============================================================================
//! @cond INTERNAL

// Immutable events corresponding to the reserved signals.
static QEvt const l_reservedEvt_[] = {
    QEVT_INITIALIZER(Q_EMPTY_SIG),
    QEVT_INITIALIZER(Q_ENTRY_SIG),
    QEVT_INITIALIZER(Q_EXIT_SIG),
    QEVT_INITIALIZER(Q_INIT_SIG)
};

enum {
    // maximum depth of state nesting in a HSM (including the top level),
    // must be >= 3
    QHSM_MAX_NEST_DEPTH_ = 6
};

// helper macro to handle reserved event in an QHsm
#define QHSM_RESERVED_EVT_(state_, sig_) \
    ((*(state_))(me, &l_reservedEvt_[(sig_)]))

// helper macro to trace state entry
#define QS_STATE_ENTRY_(state_, qs_id_)         \
    QF_CRIT_ENTRY();                            \
    QF_MEM_SYS();                              \
    QS_BEGIN_PRE_(QS_QEP_STATE_ENTRY, (qs_id_)) \
        QS_OBJ_PRE_(me);                        \
        QS_FUN_PRE_(state_);                    \
    QS_END_PRE_()                               \
    QF_MEM_APP();                              \
    QF_CRIT_EXIT()

// helper macro to trace state exit
#define QS_STATE_EXIT_(state_, qs_id_)          \
    QF_CRIT_ENTRY();                            \
    QF_MEM_SYS();                              \
    QS_BEGIN_PRE_(QS_QEP_STATE_EXIT, (qs_id_))  \
        QS_OBJ_PRE_(me);                        \
        QS_FUN_PRE_(state_);                    \
    QS_END_PRE_()                               \
    QF_MEM_APP();                              \
    QF_CRIT_EXIT()

//! @endcond

//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 730U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.3.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QEP::QP_versionStr[8]} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QEP::QP_versionStr[8]} ...................................................
char const QP_versionStr[8] = QP_VERSION_STR;
//$enddef${QEP::QP_versionStr[8]} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${QEP::QHsm} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QEP::QHsm} ...............................................................

//${QEP::QHsm::ctor} .........................................................
//! @protected @memberof QHsm
void QHsm_ctor(QHsm * const me,
    QStateHandler const initial)
{
    static struct QAsmVtable const vtable = { // QAsm virtual table
        &QHsm_init_,
        &QHsm_dispatch_
    #ifdef Q_SPY
        ,&QHsm_getStateHandler_
    #endif
    };
    // do not call the QAsm_ctor() here
    me->super.vptr      = &vtable;
    me->super.state.fun = Q_STATE_CAST(&QHsm_top);
    me->super.temp.fun  = initial;
}

//${QEP::QHsm::init_} ........................................................
//! @private @memberof QHsm
void QHsm_init_(
    QAsm * const me,
    void const * const e,
    uint_fast8_t const qs_id)
{
    QF_CRIT_STAT

    #ifdef Q_SPY
    QF_CRIT_ENTRY();
    QF_MEM_SYS();
    if ((QS_priv_.flags & 0x01U) == 0U) {
        QS_priv_.flags |= 0x01U;
        QS_FUN_DICTIONARY(&QHsm_top);
    }
    QF_MEM_APP();
    QF_CRIT_EXIT();
    #else
    Q_UNUSED_PAR(qs_id);
    #endif

    QStateHandler t = me->state.fun;

    QF_CRIT_ENTRY();
    Q_REQUIRE_INCRIT(200, (me->vptr != (struct QAsmVtable *)0)
                      && (me->temp.fun != Q_STATE_CAST(0))
                      && (t == Q_STATE_CAST(&QHsm_top)));
    QF_CRIT_EXIT();

    // execute the top-most initial transition
    QState r = (*me->temp.fun)(me, Q_EVT_CAST(QEvt));

    QF_CRIT_ENTRY();
    // the top-most initial transition must be taken
    Q_ASSERT_INCRIT(210, r == Q_RET_TRAN);

    QF_MEM_SYS();
    QS_BEGIN_PRE_(QS_QEP_STATE_INIT, qs_id)
        QS_OBJ_PRE_(me);           // this state machine object
        QS_FUN_PRE_(t);            // the source state
        QS_FUN_PRE_(me->temp.fun); // the target of the initial tran.
    QS_END_PRE_()
    QF_MEM_APP();
    QF_CRIT_EXIT();

    // drill down into the state hierarchy with initial transitions...
    do {
        QStateHandler path[QHSM_MAX_NEST_DEPTH_]; // tran entry path array
        int_fast8_t ip = 0; // tran entry path index

        path[0] = me->temp.fun;
        (void)QHSM_RESERVED_EVT_(me->temp.fun, Q_EMPTY_SIG);
        while (me->temp.fun != t) {
            ++ip;
            QF_CRIT_ENTRY();
            Q_ASSERT_INCRIT(220, ip < QHSM_MAX_NEST_DEPTH_);
            QF_CRIT_EXIT();
            path[ip] = me->temp.fun;
            (void)QHSM_RESERVED_EVT_(me->temp.fun, Q_EMPTY_SIG);
        }
        me->temp.fun = path[0];

        // nested initial transition, drill into the target hierarchy...
        do {
            // enter path[ip]
            if (QHSM_RESERVED_EVT_(path[ip], Q_ENTRY_SIG)
                == Q_RET_HANDLED)
            {
                QS_STATE_ENTRY_(path[ip], qs_id);
            }
            --ip;
        } while (ip >= 0);

        t = path[0]; // current state becomes the new source

        r = QHSM_RESERVED_EVT_(t, Q_INIT_SIG); // execute initial tran.

    #ifdef Q_SPY
        if (r == Q_RET_TRAN) {
            QF_CRIT_ENTRY();
            QF_MEM_SYS();
            QS_BEGIN_PRE_(QS_QEP_STATE_INIT, qs_id)
                QS_OBJ_PRE_(me);           // this state machine object
                QS_FUN_PRE_(t);            // the source state
                QS_FUN_PRE_(me->temp.fun); // the target of the initial tran.
            QS_END_PRE_()
            QF_MEM_APP();
            QF_CRIT_EXIT();
        }
    #endif // Q_SPY

    } while (r == Q_RET_TRAN);

    QF_CRIT_ENTRY();
    QF_MEM_SYS();
    QS_BEGIN_PRE_(QS_QEP_INIT_TRAN, qs_id)
        QS_TIME_PRE_();  // time stamp
        QS_OBJ_PRE_(me); // this state machine object
        QS_FUN_PRE_(t);  // the new active state
    QS_END_PRE_()
    QF_MEM_APP();
    QF_CRIT_EXIT();

    me->state.fun = t;   // change the current active state
    #ifndef Q_UNSAFE
    me->temp.uint = ~me->state.uint;
    #endif
}

//${QEP::QHsm::dispatch_} ....................................................
//! @private @memberof QHsm
void QHsm_dispatch_(
    QAsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    QStateHandler s = me->state.fun;
    QStateHandler t = s;
    QF_CRIT_STAT

    QF_CRIT_ENTRY();
    Q_REQUIRE_INCRIT(302, (QEvt_verify_(e))
                      && (s != Q_STATE_CAST(0))
                      && (me->state.uint == (uintptr_t)(~me->temp.uint)));
    QF_MEM_SYS();
    QS_BEGIN_PRE_(QS_QEP_DISPATCH, qs_id)
        QS_TIME_PRE_();      // time stamp
        QS_SIG_PRE_(e->sig); // the signal of the event
        QS_OBJ_PRE_(me);     // this state machine object
        QS_FUN_PRE_(s);      // the current state
    QS_END_PRE_()
    QF_MEM_APP();
    QF_CRIT_EXIT();

    // process the event hierarchically...
    QState r;
    me->temp.fun = s;
    do {
        s = me->temp.fun;
        r = (*s)(me, e); // invoke state handler s

        if (r == Q_RET_UNHANDLED) { // unhandled due to a guard?

            QF_CRIT_ENTRY();
            QF_MEM_SYS();
            QS_BEGIN_PRE_(QS_QEP_UNHANDLED, qs_id)
                QS_SIG_PRE_(e->sig); // the signal of the event
                QS_OBJ_PRE_(me);     // this state machine object
                QS_FUN_PRE_(s);      // the current state
            QS_END_PRE_()
            QF_MEM_APP();
            QF_CRIT_EXIT();

            r = QHSM_RESERVED_EVT_(s, Q_EMPTY_SIG); // superstate of s
        }
    } while (r == Q_RET_SUPER);

    // regular transition taken?
    if (r >= Q_RET_TRAN) {
        QStateHandler path[QHSM_MAX_NEST_DEPTH_];

        path[0] = me->temp.fun; // transition target
        path[1] = t; // current state
        path[2] = s; // transition source

        // exit current state to transition source s...
        for (; t != s; t = me->temp.fun) {
            // exit from t
            if (QHSM_RESERVED_EVT_(t, Q_EXIT_SIG) == Q_RET_HANDLED) {
                QS_STATE_EXIT_(t, qs_id);
                // find superstate of t
                (void)QHSM_RESERVED_EVT_(t, Q_EMPTY_SIG);
            }
        }
        int_fast8_t ip = QHsm_tran_(me, path, qs_id); // take the tran.

    #ifdef Q_SPY
        if (r == Q_RET_TRAN_HIST) {
            QF_CRIT_ENTRY();
            QF_MEM_SYS();
            QS_BEGIN_PRE_(QS_QEP_TRAN_HIST, qs_id)
                QS_OBJ_PRE_(me);      // this state machine object
                QS_FUN_PRE_(t);       // the source of the transition
                QS_FUN_PRE_(path[0]); // the target of the tran. to history
            QS_END_PRE_()
            QF_MEM_APP();
            QF_CRIT_EXIT();
        }
    #endif // Q_SPY

        // execute state entry actions in the desired order...
        for (; ip >= 0; --ip) {
            // enter path[ip]
            if (QHSM_RESERVED_EVT_(path[ip], Q_ENTRY_SIG)
                == Q_RET_HANDLED)
            {
                QS_STATE_ENTRY_(path[ip], qs_id);
            }
        }
        t = path[0];      // stick the target into register
        me->temp.fun = t; // update the next state

        // drill into the target hierarchy...
        while (QHSM_RESERVED_EVT_(t, Q_INIT_SIG) == Q_RET_TRAN) {

            QF_CRIT_ENTRY();
            QF_MEM_SYS();
            QS_BEGIN_PRE_(QS_QEP_STATE_INIT, qs_id)
                QS_OBJ_PRE_(me);           // this state machine object
                QS_FUN_PRE_(t);            // the source (pseudo)state
                QS_FUN_PRE_(me->temp.fun); // the target of the tran.
            QS_END_PRE_()
            QF_MEM_APP();
            QF_CRIT_EXIT();

            ip = 0;
            path[0] = me->temp.fun;

            // find superstate
            (void)QHSM_RESERVED_EVT_(me->temp.fun, Q_EMPTY_SIG);

            while (me->temp.fun != t) {
                ++ip;
                path[ip] = me->temp.fun;
                // find superstate
                (void)QHSM_RESERVED_EVT_(me->temp.fun, Q_EMPTY_SIG);
            }
            me->temp.fun = path[0];

            // entry path must not overflow
            QF_CRIT_ENTRY();
            Q_ASSERT_INCRIT(410, ip < QHSM_MAX_NEST_DEPTH_);
            QF_CRIT_EXIT();

            // retrace the entry path in reverse (correct) order...
            do {
                // enter path[ip]
                if (QHSM_RESERVED_EVT_(path[ip], Q_ENTRY_SIG)
                    == Q_RET_HANDLED)
                {
                    QS_STATE_ENTRY_(path[ip], qs_id);
                }
                --ip;
            } while (ip >= 0);

            t = path[0]; // current state becomes the new source
        }

        QF_CRIT_ENTRY();
        QF_MEM_SYS();
        QS_BEGIN_PRE_(QS_QEP_TRAN, qs_id)
            QS_TIME_PRE_();      // time stamp
            QS_SIG_PRE_(e->sig); // the signal of the event
            QS_OBJ_PRE_(me);     // this state machine object
            QS_FUN_PRE_(s);      // the source of the tran.
            QS_FUN_PRE_(t);      // the new active state
        QS_END_PRE_()
        QF_MEM_APP();
        QF_CRIT_EXIT();
    }

    #ifdef Q_SPY
    else if (r == Q_RET_HANDLED) {
        QF_CRIT_ENTRY();
        QF_MEM_SYS();
        QS_BEGIN_PRE_(QS_QEP_INTERN_TRAN, qs_id)
            QS_TIME_PRE_();      // time stamp
            QS_SIG_PRE_(e->sig); // the signal of the event
            QS_OBJ_PRE_(me);     // this state machine object
            QS_FUN_PRE_(s);      // the source state
        QS_END_PRE_()
        QF_MEM_APP();
        QF_CRIT_EXIT();
    }
    else {
        QF_CRIT_ENTRY();
        QF_MEM_SYS();
        QS_BEGIN_PRE_(QS_QEP_IGNORED, qs_id)
            QS_TIME_PRE_();      // time stamp
            QS_SIG_PRE_(e->sig); // the signal of the event
            QS_OBJ_PRE_(me);     // this state machine object
            QS_FUN_PRE_(me->state.fun); // the current state
        QS_END_PRE_()
        QF_MEM_APP();
        QF_CRIT_EXIT();
    }
    #endif // Q_SPY

    me->state.fun = t; // change the current active state
    #ifndef Q_UNSAFE
    me->temp.uint = ~me->state.uint;
    #endif
}

//${QEP::QHsm::getStateHandler_} .............................................
#ifdef Q_SPY
//! @private @memberof QHsm
QStateHandler QHsm_getStateHandler_(QAsm * const me) {
    return me->state.fun;
}
#endif // def Q_SPY

//${QEP::QHsm::isIn} .........................................................
//! @public @memberof QHsm
bool QHsm_isIn(QHsm * const me,
    QStateHandler const state)
{
    QF_CRIT_STAT
    QF_CRIT_ENTRY();
    Q_REQUIRE_INCRIT(602, me->super.state.uint
                           == (uintptr_t)(~me->super.temp.uint));
    QF_CRIT_EXIT();

    bool inState = false; // assume that this HSM is not in 'state'

    // scan the state hierarchy bottom-up
    QState r;
    do {
        // do the states match?
        if (me->super.temp.fun == state) {
            inState = true;    // 'true' means that match found
            r = Q_RET_IGNORED; // cause breaking out of the loop
        }
        else {
            r = QHSM_RESERVED_EVT_(me->super.temp.fun, Q_EMPTY_SIG);
        }
    } while (r != Q_RET_IGNORED); // QHsm_top() state not reached

    #ifndef Q_UNSAFE
    me->super.temp.uint = ~me->super.state.uint;
    #endif

    return inState; // return the status
}

//${QEP::QHsm::childState} ...................................................
//! @public @memberof QHsm
QStateHandler QHsm_childState(QHsm * const me,
    QStateHandler const parent)
{
    QStateHandler child = me->super.state.fun; // start with current state
    bool isFound = false; // start with the child not found

    // establish stable state configuration
    me->super.temp.fun = child;
    QState r;
    do {
        // is this the parent of the current child?
        if (me->super.temp.fun == parent) {
            isFound = true; // child is found
            r = Q_RET_IGNORED; // break out of the loop
        }
        else {
            child = me->super.temp.fun;
            r = QHSM_RESERVED_EVT_(me->super.temp.fun, Q_EMPTY_SIG);
        }
    } while (r != Q_RET_IGNORED); // the top state not reached

    #ifndef Q_UNSAFE
    me->super.temp.uint = ~me->super.state.uint;
    #endif

    QF_CRIT_STAT
    QF_CRIT_ENTRY();
    Q_ASSERT_INCRIT(890, isFound);
    QF_CRIT_EXIT();

    #ifdef Q_UNSAFE
    Q_UNUSED_PAR(isFound);
    #endif

    return child; // return the child
}

//${QEP::QHsm::tran_} ........................................................
//! @private @memberof QHsm
int_fast8_t QHsm_tran_(
    QAsm * const me,
    QStateHandler * const path,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    int_fast8_t ip = -1; // transition entry path index
    QStateHandler t = path[0];
    QStateHandler const s = path[2];
    QF_CRIT_STAT

    // (a) check source==target (transition to self)...
    if (s == t) {
        // exit source s
        if (QHSM_RESERVED_EVT_(s, Q_EXIT_SIG) == Q_RET_HANDLED) {
            QS_STATE_EXIT_(s, qs_id);
        }
        ip = 0; // enter the target
    }
    else {
        // find superstate of target
        (void)QHSM_RESERVED_EVT_(t, Q_EMPTY_SIG);

        t = me->temp.fun;

        // (b) check source==target->super...
        if (s == t) {
            ip = 0; // enter the target
        }
        else {
            // find superstate of src
            (void)QHSM_RESERVED_EVT_(s, Q_EMPTY_SIG);

            // (c) check source->super==target->super...
            if (me->temp.fun == t) {
                // exit source s
                if (QHSM_RESERVED_EVT_(s, Q_EXIT_SIG) == Q_RET_HANDLED) {
                    QS_STATE_EXIT_(s, qs_id);
                }
                ip = 0; // enter the target
            }
            else {
                // (d) check source->super==target...
                if (me->temp.fun == path[0]) {
                    // exit source s
                    if (QHSM_RESERVED_EVT_(s, Q_EXIT_SIG) == Q_RET_HANDLED) {
                        QS_STATE_EXIT_(s, qs_id);
                    }
                }
                else {
                    // (e) check rest of source==target->super->super..
                    // and store the entry path along the way
                    int_fast8_t iq = 0; // indicate that LCA not found
                    ip = 1; // enter target and its superstate
                    path[1] = t;      // save the superstate of target
                    t = me->temp.fun; // save source->super

                    // find target->super->super...
                    QState r = QHSM_RESERVED_EVT_(path[1], Q_EMPTY_SIG);
                    while (r == Q_RET_SUPER) {
                        ++ip;
                        path[ip] = me->temp.fun; // store the entry path
                        if (me->temp.fun == s) { // is it the source?
                            iq = 1; // indicate that the LCA found

                            // entry path must not overflow
                            QF_CRIT_ENTRY();
                            Q_ASSERT_INCRIT(510, ip < QHSM_MAX_NEST_DEPTH_);
                            QF_CRIT_EXIT();
                            --ip; // do not enter the source
                            r = Q_RET_HANDLED; // terminate the loop
                        }
                        else { // it is not the source, keep going up
                            r = QHSM_RESERVED_EVT_(me->temp.fun, Q_EMPTY_SIG);
                        }
                    }

                    // the LCA not found yet?
                    if (iq == 0) {
                        // entry path must not overflow
                        QF_CRIT_ENTRY();
                        Q_ASSERT_INCRIT(520, ip < QHSM_MAX_NEST_DEPTH_);
                        QF_CRIT_EXIT();

                        // exit source s
                        if (QHSM_RESERVED_EVT_(s, Q_EXIT_SIG)
                            == Q_RET_HANDLED)
                        {
                            QS_STATE_EXIT_(s, qs_id);
                        }

                        // (f) check the rest of source->super
                        //                  == target->super->super...
                        iq = ip;
                        r = Q_RET_IGNORED; // indicate that the LCA NOT found
                        do {
                            if (t == path[iq]) { // is this the LCA?
                                r = Q_RET_HANDLED; // indicate the LCA found
                                ip = iq - 1; // do not enter the LCA
                                iq = -1; // cause termination of the loop
                            }
                            else {
                                --iq; // try lower superstate of target
                            }
                        } while (iq >= 0);

                        // the LCA not found yet?
                        if (r != Q_RET_HANDLED) {
                            // (g) check each source->super->...
                            // for each target->super...
                            r = Q_RET_IGNORED; // keep looping
                            do {
                                // exit from t
                                if (QHSM_RESERVED_EVT_(t, Q_EXIT_SIG)
                                    == Q_RET_HANDLED)
                                {
                                    QS_STATE_EXIT_(t, qs_id);
                                    // find superstate of t
                                    (void)QHSM_RESERVED_EVT_(t, Q_EMPTY_SIG);
                                }
                                t = me->temp.fun; // set to super of t
                                iq = ip;
                                do {
                                    // is this the LCA?
                                    if (t == path[iq]) {
                                        ip = iq - 1; // do not enter the LCA
                                        iq = -1;     // break out of inner loop
                                        r = Q_RET_HANDLED; // break outer loop
                                    }
                                    else {
                                        --iq;
                                    }
                                } while (iq >= 0);
                            } while (r != Q_RET_HANDLED);
                        }
                    }
                }
            }
        }
    }
    return ip;
}

//${QEP::QHsm::top} ..........................................................
//! @protected @memberof QAsm
QState QHsm_top(QHsm const * const me,
    QEvt const * const e)
{
    Q_UNUSED_PAR(me);
    Q_UNUSED_PAR(e);
    return Q_RET_IGNORED; // the top state ignores all events
}
//$enddef${QEP::QHsm} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
