//$file${include::qep.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${include::qep.h}
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
//$endhead${include::qep.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef QEP_H_
#define QEP_H_

//============================================================================
#define QP_VERSION     730U
#define QP_VERSION_STR "7.3.0"
#define QP_RELEASE     0x766E8D15U

//============================================================================
//! @cond INTERNAL

#ifndef Q_SIGNAL_SIZE
#define Q_SIGNAL_SIZE 2U
#endif // ndef Q_SIGNAL_SIZE

//! @endcond
//============================================================================

//$declare${glob-types} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${glob-types::int_t} .......................................................
typedef int int_t;

//${glob-types::enum_t} ......................................................
typedef int enum_t;

//${glob-types::float32_t} ...................................................
typedef float float32_t;

//${glob-types::float64_t} ...................................................
typedef double float64_t;
//$enddecl${glob-types} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$declare${QEP} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QEP::QP_versionStr[8]} ...................................................
//! the current QP version number string in ROM, based on #QP_VERSION_STR
extern char const QP_versionStr[8];

//${QEP::QSignal} ............................................................
#if (Q_SIGNAL_SIZE == 1U)
typedef uint8_t QSignal;
#endif //  (Q_SIGNAL_SIZE == 1U)

//${QEP::QSignal} ............................................................
#if (Q_SIGNAL_SIZE == 2U)
typedef uint16_t QSignal;
#endif //  (Q_SIGNAL_SIZE == 2U)

//${QEP::QSignal} ............................................................
#if (Q_SIGNAL_SIZE == 4U)
typedef uint32_t QSignal;
#endif //  (Q_SIGNAL_SIZE == 4U)

//${QEP::QEVT_MARKER} ........................................................
#define QEVT_MARKER 0xE0U

//${QEP::QEvt} ...............................................................
// ! @class QEvt
typedef struct {
// public:

    //! @public @memberof QEvt
    QSignal sig;

// private:

    //! @private @memberof QEvt
    uint8_t poolId_;

    //! @private @memberof QEvt
    uint8_t volatile refCtr_;
} QEvt;

// public:

//! @public @memberof QEvt
static inline void QEvt_ctor(QEvt * const me,
    enum_t const sig)
{
    me->sig     = (QSignal)sig;
    me->poolId_ = QEVT_MARKER;
    me->refCtr_ = 0U;
}

//! @private @memberof QEvt
static inline bool QEvt_verify_(QEvt const * const me) {
    return (me != (QEvt const *)0)
           && ((me->poolId_ & 0xF0U) == QEVT_MARKER);
}

//${QEP::QStateRet} ..........................................................
//! All possible values returned from state/action handlers
//! @note
//! The order of enumeration matters for algorithmic correctness.
enum QStateRet {
    // unhandled and need to "bubble up"
    Q_RET_SUPER,     //!< event passed to superstate to handle
    Q_RET_SUPER_SUB, //!< event passed to submachine superstate
    Q_RET_UNHANDLED, //!< event unhandled due to a guard

    // handled and do not need to "bubble up"
    Q_RET_HANDLED,   //!< event handled (internal transition)
    Q_RET_IGNORED,   //!< event silently ignored (bubbled up to top)

    // entry/exit
    Q_RET_ENTRY,     //!< state entry action executed
    Q_RET_EXIT,      //!< state exit  action executed

    // no side effects
    Q_RET_NULL,      //!< return value without any effect

    // transitions need to execute transition-action table in ::QMsm
    Q_RET_TRAN,      //!< regular transition
    Q_RET_TRAN_INIT, //!< initial transition in a state or submachine
    Q_RET_TRAN_EP,   //!< entry-point transition into a submachine

    // transitions that additionally clobber me->state
    Q_RET_TRAN_HIST, //!< transition to history of a given state
    Q_RET_TRAN_XP    //!< exit-point transition out of a submachine
};

//${QEP::QState} .............................................................
typedef enum QStateRet QState;

//${QEP::QStateHandler} ......................................................
typedef QState (* QStateHandler )(void * const me, QEvt const * const e);

//${QEP::QActionHandler} .....................................................
typedef QState (* QActionHandler )(void * const me);

//${QEP::QXThread} ...........................................................
// forward declaration
struct QXThread;

//${QEP::QXThreadHandler} ....................................................
typedef void (* QXThreadHandler )(struct QXThread * const me);

//${QEP::QMState} ............................................................
typedef struct QMState {
    struct QMState const *superstate; //!< @private @memberof QMState
    QStateHandler const stateHandler; //!< @private @memberof QMState
    QActionHandler const entryAction; //!< @private @memberof QMState
    QActionHandler const exitAction;  //!< @private @memberof QMState
    QActionHandler const initAction;  //!< @private @memberof QMState
} QMState;

//${QEP::QMTranActTable} .....................................................
typedef struct QMTranActTable {
    QMState const *target;       //!< @private @memberof QMTranActTable
    QActionHandler const act[1]; //!< @private @memberof QMTranActTable
} QMTranActTable;

//${QEP::QAsmAttr} ...........................................................
union QAsmAttr {
    QStateHandler   fun;         //!< @private @memberof QAsmAttr
    QActionHandler  act;         //!< @private @memberof QAsmAttr
    QXThreadHandler thr;         //!< @private @memberof QAsmAttr
    QMTranActTable const *tatbl; //!< @private @memberof QAsmAttr
    struct QMState const *obj;   //!< @private @memberof QAsmAttr
#ifndef Q_UNSAFE
    uintptr_t      uint;         //!< @private @memberof QAsmAttr
#endif
};

//${QEP::QReservedSig} .......................................................
//! Reserved signals by the QHsm-style state machine implementation
enum QReservedSig {
    Q_EMPTY_SIG,     //!< signal to execute the default case
    Q_ENTRY_SIG,     //!< signal for coding entry actions
    Q_EXIT_SIG,      //!< signal for coding exit actions
    Q_INIT_SIG,      //!< signal for coding initial transitions
    Q_USER_SIG       //!< offset for the user signals (QP Application)
};

//${QEP::QAsm} ...............................................................
//! @class QAsm
typedef struct {
// private:

    //! @protected @memberof QAsm
    struct QAsmVtable const * vptr;

// protected:

    //! @protected @memberof QAsm
    union QAsmAttr state;

    //! @protected @memberof QAsm
    union QAsmAttr temp;
} QAsm;

// protected:

//! @protected @memberof QAsm
void QAsm_ctor(QAsm * const me);

//${QEP::QAsmVtable} .........................................................
struct QAsmVtable {
    void (*init)(QAsm * const me, void const * const e,
                 uint_fast8_t const qs_id);
    void (*dispatch)(QAsm * const me, QEvt const * const e,
                     uint_fast8_t const qs_id);
#ifdef Q_SPY
    QStateHandler (*getStateHandler)(QAsm * const me);
#endif // Q_SPY
};

//${QEP::QHsm} ...............................................................
//! @class QHsm
//! @extends QAsm
typedef struct {
// protected:
    QAsm super;
} QHsm;

// protected:

//! @protected @memberof QHsm
void QHsm_ctor(QHsm * const me,
    QStateHandler const initial);

// private:

//! @private @memberof QHsm
void QHsm_init_(
    QAsm * const me,
    void const * const e,
    uint_fast8_t const qs_id);

//! @private @memberof QHsm
void QHsm_dispatch_(
    QAsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id);

#ifdef Q_SPY
//! @private @memberof QHsm
QStateHandler QHsm_getStateHandler_(QAsm * const me);
#endif // def Q_SPY

// public:

//! @public @memberof QHsm
bool QHsm_isIn(QHsm * const me,
    QStateHandler const state);

//! @public @memberof QHsm
static inline QStateHandler QHsm_state(QHsm const * const me) {
    return me->super.state.fun;
}

//! @public @memberof QHsm
QStateHandler QHsm_childState(QHsm * const me,
    QStateHandler const parent);

// private:

//! @private @memberof QHsm
int_fast8_t QHsm_tran_(
    QAsm * const me,
    QStateHandler * const path,
    uint_fast8_t const qs_id);

// protected:

//! @protected @memberof QAsm
QState QHsm_top(QHsm const * const me,
    QEvt const * const e);

//${QEP::QMsm} ...............................................................
//! @class QMsm
//! @extends QAsm
typedef struct {
// protected:
    QAsm super;
} QMsm;

// protected:

//! @protected @memberof QMsm
void QMsm_ctor(QMsm * const me,
    QStateHandler const initial);

// public:

//! @private @memberof QMsm
void QMsm_init_(
    QAsm * const me,
    void const * const e,
    uint_fast8_t const qs_id);

// private:

//! @private @memberof QMsm
void QMsm_dispatch_(
    QAsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id);

// public:

#ifdef Q_SPY
//! @public @memberof QMsm
static inline QStateHandler QMsm_getStateHandler_(QAsm * const me) {
    return me->state.obj->stateHandler;
}
#endif // def Q_SPY

//! @public @memberof QMsm
bool QMsm_isInState(QMsm const * const me,
    QMState const * const stateObj);

//! @public @memberof QMsm
static inline QMState const * QMsm_stateObj(QMsm * const me) {
    return me->super.state.obj;
}

//! @public @memberof QMsm
QMState const * QMsm_childStateObj(QMsm const * const me,
    QMState const * const parent);

// private:

//! @private @memberof QMsm
QState QMsm_execTatbl_(
    QAsm * const me,
    QMTranActTable const * const tatbl,
    uint_fast8_t const qs_id);

//! @private @memberof QMsm
void QMsm_exitToTranSource_(
    QAsm * const me,
    QMState const * const cs,
    QMState const * const ts,
    uint_fast8_t const qs_id);

//! @private @memberof QMsm
QState QMsm_enterHistory_(
    QAsm * const me,
    QMState const *const hist,
    uint_fast8_t const qs_id);
//$enddecl${QEP} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$declare${QEP-macros} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QEP-macros::QEVT_INITIALIZER} ............................................
#define QEVT_INITIALIZER(sig_) { (QSignal)(sig_), QEVT_MARKER, 0U }

//${QEP-macros::QEVT_POOLID_} ................................................
#define QEVT_POOLID_(e_) ((uint_fast8_t)(e_)->poolId_ & 0x0FU)

//${QEP-macros::QASM_INIT} ...................................................
#ifdef Q_SPY
#define QASM_INIT(me_, par_, qs_id_) do { \
    Q_ASSERT(((QAsm *)(me_))->vptr); \
    (*((QAsm *)(me_))->vptr->init)((QAsm *)(me_), (par_), (qs_id_)); \
} while (false)
#endif // def Q_SPY

//${QEP-macros::QASM_INIT} ...................................................
#ifndef Q_SPY
#define QASM_INIT(me_, par_, dummy) do { \
    Q_ASSERT(((QAsm *)(me_))->vptr); \
    (*((QAsm *)(me_))->vptr->init)((QAsm *)(me_), (par_), 0); \
} while (false)
#endif // ndef Q_SPY

//${QEP-macros::QASM_DISPATCH} ...............................................
#ifdef Q_SPY
#define QASM_DISPATCH(me_, e_, qs_id_) \
    (*((QAsm *)(me_))->vptr->dispatch)((QAsm *)(me_), (e_), (qs_id_))
#endif // def Q_SPY

//${QEP-macros::QASM_DISPATCH} ...............................................
#ifndef Q_SPY
#define QASM_DISPATCH(me_, e_, dummy) \
    (*((QAsm *)(me_))->vptr->dispatch)((QAsm *)(me_), (e_), 0U)
#endif // ndef Q_SPY

//${QEP-macros::Q_ASM_UPCAST} ................................................
#define Q_ASM_UPCAST(ptr_) ((QAsm *)(ptr_))

//${QEP-macros::Q_HSM_UPCAST} ................................................
#define Q_HSM_UPCAST(ptr_) ((QHsm *)(ptr_))

//${QEP-macros::Q_MSM_UPCAST} ................................................
#define Q_MSM_UPCAST(ptr_) ((QMsm *)(ptr_))

//${QEP-macros::Q_TRAN} ......................................................
#define Q_TRAN(target_) \
    ((Q_ASM_UPCAST(me))->temp.fun = Q_STATE_CAST(target_), \
     (QState)Q_RET_TRAN)

//${QEP-macros::Q_TRAN_HIST} .................................................
#define Q_TRAN_HIST(hist_) \
    ((Q_ASM_UPCAST(me))->temp.fun = (hist_), \
     (QState)Q_RET_TRAN_HIST)

//${QEP-macros::Q_SUPER} .....................................................
#define Q_SUPER(super_) \
    ((Q_ASM_UPCAST(me))->temp.fun = Q_STATE_CAST(super_), \
     (QState)Q_RET_SUPER)

//${QEP-macros::Q_HANDLED} ...................................................
#define Q_HANDLED() ((QState)Q_RET_HANDLED)

//${QEP-macros::Q_UNHANDLED} .................................................
#define Q_UNHANDLED() ((QState)Q_RET_UNHANDLED)

//${QEP-macros::Q_ACTION_NULL} ...............................................
#define Q_ACTION_NULL ((QActionHandler)0)

//${QEP-macros::Q_EVT_CAST} ..................................................
#define Q_EVT_CAST(class_) ((class_ const *)(e))

//${QEP-macros::Q_STATE_CAST} ................................................
#define Q_STATE_CAST(handler_) ((QStateHandler)(handler_))

//${QEP-macros::Q_ACTION_CAST} ...............................................
#define Q_ACTION_CAST(action_) ((QActionHandler)(action_))

//${QEP-macros::Q_UNUSED_PAR} ................................................
#define Q_UNUSED_PAR(par_) ((void)(par_))

//${QEP-macros::Q_DIM} .......................................................
#define Q_DIM(array_) (sizeof(array_) / sizeof((array_)[0U]))

//${QEP-macros::Q_UINT2PTR_CAST} .............................................
#define Q_UINT2PTR_CAST(type_, uint_) ((type_ *)(uint_))

//${QEP-macros::QM_ENTRY} ....................................................
#ifdef Q_SPY
#define QM_ENTRY(state_) \
    ((Q_ASM_UPCAST(me))->temp.obj = (state_), \
     (QState)Q_RET_ENTRY)
#endif // def Q_SPY

//${QEP-macros::QM_ENTRY} ....................................................
#ifndef Q_SPY
#define QM_ENTRY(dummy) ((QState)Q_RET_ENTRY)
#endif // ndef Q_SPY

//${QEP-macros::QM_EXIT} .....................................................
#ifdef Q_SPY
#define QM_EXIT(state_) \
    ((Q_ASM_UPCAST(me))->temp.obj = (state_), \
     (QState)Q_RET_EXIT)
#endif // def Q_SPY

//${QEP-macros::QM_EXIT} .....................................................
#ifndef Q_SPY
#define QM_EXIT(dummy) ((QState)Q_RET_EXIT)
#endif // ndef Q_SPY

//${QEP-macros::QM_SM_EXIT} ..................................................
#define QM_SM_EXIT(state_) \
    ((Q_ASM_UPCAST(me))->temp.obj = (state_), \
     (QState)Q_RET_EXIT)

//${QEP-macros::QM_TRAN} .....................................................
#define QM_TRAN(tatbl_) ((Q_ASM_UPCAST(me))->temp.tatbl \
    = (struct QMTranActTable const *)(tatbl_), \
 (QState)Q_RET_TRAN)

//${QEP-macros::QM_TRAN_INIT} ................................................
#define QM_TRAN_INIT(tatbl_) ((Q_ASM_UPCAST(me))->temp.tatbl \
    = (struct QMTranActTable const *)(tatbl_), \
 (QState)Q_RET_TRAN_INIT)

//${QEP-macros::QM_TRAN_HIST} ................................................
#define QM_TRAN_HIST(history_, tatbl_) \
    ((((Q_ASM_UPCAST(me))->state.obj  = (history_)), \
      ((Q_ASM_UPCAST(me))->temp.tatbl = \
          (struct QMTranActTable const *)(tatbl_))), \
     (QState)Q_RET_TRAN_HIST)

//${QEP-macros::QM_TRAN_EP} ..................................................
#define QM_TRAN_EP(tatbl_) ((Q_ASM_UPCAST(me))->temp.tatbl \
    = (struct QMTranActTable const *)(tatbl_), \
 (QState)Q_RET_TRAN_EP)

//${QEP-macros::QM_TRAN_XP} ..................................................
#define QM_TRAN_XP(xp_, tatbl_) \
    ((((Q_ASM_UPCAST(me))->state.act  = (xp_)), \
      ((Q_ASM_UPCAST(me))->temp.tatbl = \
          (struct QMTranActTable const *)(tatbl_))), \
     (QState)Q_RET_TRAN_XP)

//${QEP-macros::QM_HANDLED} ..................................................
#define QM_HANDLED() ((QState)Q_RET_HANDLED)

//${QEP-macros::QM_UNHANDLED} ................................................
#define QM_UNHANDLED() ((QState)Q_RET_UNHANDLED)

//${QEP-macros::QM_SUPER} ....................................................
#define QM_SUPER() ((QState)Q_RET_SUPER)

//${QEP-macros::QM_SUPER_SUB} ................................................
#define QM_SUPER_SUB(host_) \
    ((Q_ASM_UPCAST(me))->temp.obj = (host_), \
     (QState)Q_RET_SUPER_SUB)

//${QEP-macros::QM_STATE_NULL} ...............................................
#define QM_STATE_NULL ((QMState *)0)
//$enddecl${QEP-macros} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif // QEP_H_
