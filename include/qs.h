//$file${include::qs.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${include::qs.h}
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
//$endhead${include::qs.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef QS_H_
#define QS_H_

#ifndef Q_SPY
#error "Q_SPY must be defined to include qs.h"
#endif

//============================================================================
//! @cond INTERNAL

#ifndef QS_CTR_SIZE
#define QS_CTR_SIZE 2U
#endif

#ifndef QS_TIME_SIZE
#define QS_TIME_SIZE 4U
#endif

//! @endcond
//============================================================================

//$declare${QS-types} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QS-types::QS} ............................................................
// @class QS
typedef struct QS {
    //! @cond INTERNAL
    uint8_t dummy;
    //! @endcond
} QS;

//${QS-types::QSpyPre} .......................................................
//! @static @public @memberof QS
//! pre-defined QS record IDs
enum QSpyPre {
    // [0] QS session (not maskable)
    QS_EMPTY,             //!< QS record for cleanly starting a session

    // [1] SM records
    QS_QEP_STATE_ENTRY,   //!< a state was entered
    QS_QEP_STATE_EXIT,    //!< a state was exited
    QS_QEP_STATE_INIT,    //!< an initial transition was taken in a state
    QS_QEP_INIT_TRAN,     //!< the top-most initial transition was taken
    QS_QEP_INTERN_TRAN,   //!< an internal transition was taken
    QS_QEP_TRAN,          //!< a regular transition was taken
    QS_QEP_IGNORED,       //!< an event was ignored (silently discarded)
    QS_QEP_DISPATCH,      //!< an event was dispatched (begin of RTC step)
    QS_QEP_UNHANDLED,     //!< an event was un-handled due to a guard

    // [10] Active Object (AO) records
    QS_QF_ACTIVE_DEFER,   //!< AO deferred an event
    QS_QF_ACTIVE_RECALL,  //!< AO recalled an event
    QS_QF_ACTIVE_SUBSCRIBE,   //!< an AO subscribed to an event
    QS_QF_ACTIVE_UNSUBSCRIBE, //!< an AO unsubscribed to an event
    QS_QF_ACTIVE_POST,      //!< an event was posted (FIFO) directly to AO
    QS_QF_ACTIVE_POST_LIFO, //!< an event was posted (LIFO) directly to AO
    QS_QF_ACTIVE_GET,     //!< AO got an event and its queue is not empty
    QS_QF_ACTIVE_GET_LAST,//!< AO got an event and its queue is empty
    QS_QF_ACTIVE_RECALL_ATTEMPT, //!< AO attempted to recall an event

    // [19] Event Queue (EQ) records
    QS_QF_EQUEUE_POST,      //!< an event was posted (FIFO) to a raw queue
    QS_QF_EQUEUE_POST_LIFO, //!< an event was posted (LIFO) to a raw queue
    QS_QF_EQUEUE_GET,     //!< get an event and queue still not empty
    QS_QF_EQUEUE_GET_LAST,//!< get the last event from the queue

    // [23] Framework (QF) records
    QS_QF_NEW_ATTEMPT,   //!< an attempt to allocate an event failed

    // [24] Memory Pool (MP) records
    QS_QF_MPOOL_GET,      //!< a memory block was removed from memory pool
    QS_QF_MPOOL_PUT,      //!< a memory block was returned to memory pool

    // [26] Additional Framework (QF) records
    QS_QF_PUBLISH,        //!< an event was published to active objects
    QS_QF_NEW_REF,        //!< new event reference was created
    QS_QF_NEW,            //!< new event was created
    QS_QF_GC_ATTEMPT,     //!< garbage collection attempt
    QS_QF_GC,             //!< garbage collection
    QS_QF_TICK,           //!< QTimeEvt_tick_() was called

    // [32] Time Event (TE) records
    QS_QF_TIMEEVT_ARM,    //!< a time event was armed
    QS_QF_TIMEEVT_AUTO_DISARM, //!< a time event expired and was disarmed
    QS_QF_TIMEEVT_DISARM_ATTEMPT,//!< attempt to disarm a disarmed QTimeEvt
    QS_QF_TIMEEVT_DISARM, //!< true disarming of an armed time event
    QS_QF_TIMEEVT_REARM,  //!< rearming of a time event
    QS_QF_TIMEEVT_POST,   //!< a time event posted itself directly to an AO

    // [38] Additional Framework (QF) records
    QS_QF_DELETE_REF,     //!< an event reference is about to be deleted
    QS_QF_CRIT_ENTRY,     //!< critical section was entered
    QS_QF_CRIT_EXIT,      //!< critical section was exited
    QS_QF_ISR_ENTRY,      //!< an ISR was entered
    QS_QF_ISR_EXIT,       //!< an ISR was exited
    QS_QF_INT_DISABLE,    //!< interrupts were disabled
    QS_QF_INT_ENABLE,     //!< interrupts were enabled

    // [45] Additional Active Object (AO) records
    QS_QF_ACTIVE_POST_ATTEMPT,//!< attempt to post an evt to AO failed

    // [46] Additional Event Queue (EQ) records
    QS_QF_EQUEUE_POST_ATTEMPT,//!< attempt to post evt to QEQueue failed

    // [47] Additional Memory Pool (MP) records
    QS_QF_MPOOL_GET_ATTEMPT,  //!< attempt to get a memory block failed

    // [48] Scheduler (SC) records
    QS_SCHED_PREEMPT,     //!< scheduler asynchronously preempted a task
    QS_SCHED_RESTORE,     //!< scheduler restored preempted task
    QS_SCHED_LOCK,        //!< scheduler was locked
    QS_SCHED_UNLOCK,      //!< scheduler was unlocked
    QS_SCHED_NEXT,        //!< scheduler started new task
    QS_SCHED_IDLE,        //!< scheduler restored the idle task

    // [54] Miscellaneous QS records (not maskable)
    QS_ENUM_DICT,         //!< enumeration dictionary entry

    // [55] Additional QEP records
    QS_QEP_TRAN_HIST,     //!< a tran to history was taken
    QS_QEP_TRAN_EP,       //!< a tran to entry point into a submachine
    QS_QEP_TRAN_XP,       //!< a tran to exit  point out of a submachine

    // [58] Miscellaneous QS records (not maskable)
    QS_TEST_PAUSED,       //!< test has been paused
    QS_TEST_PROBE_GET,    //!< reports that Test-Probe has been used
    QS_SIG_DICT,          //!< signal dictionary entry
    QS_OBJ_DICT,          //!< object dictionary entry
    QS_FUN_DICT,          //!< function dictionary entry
    QS_USR_DICT,          //!< user QS record dictionary entry
    QS_TARGET_INFO,       //!< reports the Target information
    QS_TARGET_DONE,       //!< reports completion of a user callback
    QS_RX_STATUS,         //!< reports QS data receive status
    QS_QUERY_DATA,        //!< reports the data from "current object" query
    QS_PEEK_DATA,         //!< reports the data from the PEEK query
    QS_ASSERT_FAIL,       //!< assertion failed in the code
    QS_QF_RUN,            //!< QF_run() was entered

    // [71] Semaphore (SEM) records
    QS_SEM_TAKE,          //!< a semaphore was taken by a thread
    QS_SEM_BLOCK,         //!< a semaphore blocked a thread
    QS_SEM_SIGNAL,        //!< a semaphore was signaled
    QS_SEM_BLOCK_ATTEMPT, //!< a semaphore blocked was attempted

    // [75] Mutex (MTX) records
    QS_MTX_LOCK,          //!< a mutex was locked
    QS_MTX_BLOCK,         //!< a mutex blocked a thread
    QS_MTX_UNLOCK,        //!< a mutex was unlocked
    QS_MTX_LOCK_ATTEMPT,  //!< a mutex lock was attempted
    QS_MTX_BLOCK_ATTEMPT, //!< a mutex blocking was attempted
    QS_MTX_UNLOCK_ATTEMPT,//!< a mutex unlock was attempted

    // [81]
    QS_PRE_MAX            //!< the # predefined signals
};

//${QS-types::QSpyGroups} ....................................................
//! @static @public @memberof QS
//! QS-TX record groups for QS_GLB_FILTER()
enum QSpyGroups {
    QS_ALL_RECORDS = 0xF0,//!< all maskable QS records
    QS_SM_RECORDS,        //!< State Machine QS records
    QS_AO_RECORDS,        //!< Active Object QS records
    QS_EQ_RECORDS,        //!< Event Queues QS records
    QS_MP_RECORDS,        //!< Memory Pools QS records
    QS_TE_RECORDS,        //!< Time Events QS records
    QS_QF_RECORDS,        //!< QF QS records
    QS_SC_RECORDS,        //!< Scheduler QS records
    QS_SEM_RECORDS,       //!< Semaphore QS records
    QS_MTX_RECORDS,       //!< Mutex QS records
    QS_U0_RECORDS,        //!< User Group 100-104 records
    QS_U1_RECORDS,        //!< User Group 105-109 records
    QS_U2_RECORDS,        //!< User Group 110-114 records
    QS_U3_RECORDS,        //!< User Group 115-119 records
    QS_U4_RECORDS,        //!< User Group 120-124 records
    QS_UA_RECORDS         //!< All User records
};

//${QS-types::QSpyUserOffsets} ...............................................
//! @static @public @memberof QS
//! QS user record group offsets for QS_GLB_FILTER()
enum QSpyUserOffsets {
    QS_USER  = 100,   //!< the first record available to QS users
    QS_USER0 = (enum_t)QS_USER,      //!< offset for User Group 0
    QS_USER1 = (enum_t)QS_USER0 + 5, //!< offset for User Group 1
    QS_USER2 = (enum_t)QS_USER1 + 5, //!< offset for User Group 2
    QS_USER3 = (enum_t)QS_USER2 + 5, //!< offset for User Group 3
    QS_USER4 = (enum_t)QS_USER3 + 5  //!< offset for User Group 4
};

//${QS-types::QSpyIdOffsets} .................................................
//! @static @public @memberof QS
//! QS ID offsets for QS_LOC_FILTER()
enum QSpyIdOffsets {
    QS_AO_ID = 0,  //!< offset for AO priorities
    QS_EP_ID = 64, //!< offset for event-pool IDs
    QS_EQ_ID = 80, //!< offset for event-queue IDs
    QS_AP_ID = 96  //!< offset for Application-specific IDs
};

//${QS-types::QSpyIdGroups} ..................................................
//! @static @public @memberof QS
//! QS ID groups for QS_LOC_FILTER()
enum QSpyIdGroups {
    QS_ALL_IDS = 0xF0,                      //!< all QS IDs
    QS_AO_IDS  = (0x80 + (enum_t)QS_AO_ID), //!< AO IDs (priorities)
    QS_EP_IDS  = (0x80 + (enum_t)QS_EP_ID), //!< event-pool IDs
    QS_EQ_IDS  = (0x80 + (enum_t)QS_EQ_ID), //!< event-queue IDs
    QS_AP_IDS  = (0x80 + (enum_t)QS_AP_ID)  //!< Application-specific IDs
};

//${QS-types::QSpyId} ........................................................
//! @struct QSpyId
typedef struct { uint8_t prio; } QSpyId;

//${QS-types::QSpyFilter} ....................................................
//! @struct QSpyFilter
typedef struct { uint8_t glb[16]; uint8_t loc[16]; } QSpyFilter;

//${QS-types::QSpyFunPtr} ....................................................
//! @static @private @memberof QS
typedef void (* QSpyFunPtr )(void);

//${QS-types::QSCtr} .........................................................
#if (QS_CTR_SIZE == 2U)
typedef uint16_t QSCtr;
#endif //  (QS_CTR_SIZE == 2U)

//${QS-types::QSCtr} .........................................................
#if (QS_CTR_SIZE == 4U)
typedef uint32_t QSCtr;
#endif //  (QS_CTR_SIZE == 4U)

//${QS-types::QSTimeCtr} .....................................................
#if (QS_TIME_SIZE == 2U)
typedef uint16_t QSTimeCtr;
#endif //  (QS_TIME_SIZE == 2U)

//${QS-types::QSTimeCtr} .....................................................
#if (QS_TIME_SIZE == 4U)
typedef uint32_t QSTimeCtr;
#endif //  (QS_TIME_SIZE == 4U)

//${QS-types::QSFun} .........................................................
#if (QS_FUN_PTR_SIZE == 2U)
typedef uint16_t QSFun;
#endif //  (QS_FUN_PTR_SIZE == 2U)

//${QS-types::QSFun} .........................................................
#if (QS_FUN_PTR_SIZE == 4U)
typedef uint32_t QSFun;
#endif //  (QS_FUN_PTR_SIZE == 4U)

//${QS-types::QSFun} .........................................................
#if (QS_FUN_PTR_SIZE == 8U)
typedef uint64_t QSFun;
#endif //  (QS_FUN_PTR_SIZE == 8U)
//$enddecl${QS-types} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${QS-macros} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QS-macros::QS_INIT} ......................................................
#define QS_INIT(arg_) (QS_onStartup(arg_))

//${QS-macros::QS_EXIT} ......................................................
#define QS_EXIT() (QS_onCleanup())

//${QS-macros::QS_OUTPUT} ....................................................
#define QS_OUTPUT() (QS_output())

//${QS-macros::QS_RX_INPUT} ..................................................
#define QS_RX_INPUT() (QS_rx_input())

//${QS-macros::QS_GLB_FILTER} ................................................
#define QS_GLB_FILTER(rec_) (QS_glbFilter_((int_fast16_t)(rec_)))

//${QS-macros::QS_LOC_FILTER} ................................................
#define QS_LOC_FILTER(qs_id_) (QS_locFilter_((int_fast16_t)(qs_id_)))

//${QS-macros::QS_BEGIN_ID} ..................................................
#define QS_BEGIN_ID(rec_, qs_id_) \
if (QS_GLB_CHECK_(rec_) && QS_LOC_CHECK_(qs_id_)) { \
    QS_CRIT_STAT_ \
    QS_CRIT_E_(); \
    QS_MEM_SYS_(); \
    QS_beginRec_((uint_fast8_t)(rec_)); \
    QS_TIME_PRE_(); {

//${QS-macros::QS_END} .......................................................
#define QS_END() \
    } QS_endRec_(); \
    QS_CRIT_X_(); \
    QS_MEM_APP_(); \
}

//${QS-macros::QS_FLUSH} .....................................................
#define QS_FLUSH() (QS_onFlush())

//${QS-macros::QS_BEGIN_NOCRIT} ..............................................
#define QS_BEGIN_NOCRIT(rec_, qs_id_) \
if (QS_GLB_CHECK_(rec_) && QS_LOC_CHECK_(qs_id_)) { \
    QS_beginRec_((uint_fast8_t)(rec_)); \
    QS_TIME_PRE_(); {

//${QS-macros::QS_END_NOCRIT} ................................................
#define QS_END_NOCRIT() } \
    QS_endRec_();\
}

//${QS-macros::QS_GLB_CHECK_} ................................................
#define QS_GLB_CHECK_(rec_) \
    (((uint_fast8_t)QS_filter_.glb[(uint_fast8_t)(rec_) >> 3U] \
          & ((uint_fast8_t)1U << ((uint_fast8_t)(rec_) & 7U))) != 0U)

//${QS-macros::QS_LOC_CHECK_} ................................................
#define QS_LOC_CHECK_(qs_id_) \
    (((uint_fast8_t)QS_filter_.loc[(uint_fast8_t)(qs_id_) >> 3U] \
          & ((uint_fast8_t)1U << ((uint_fast8_t)(qs_id_) & 7U))) != 0U)

//${QS-macros::QS_REC_DONE} ..................................................
#ifndef QS_REC_DONE
#define QS_REC_DONE() ((void)0)
#endif // ndef QS_REC_DONE

//${QS-macros::QS_I8} ........................................................
#define QS_I8(width_, data_) \
    (QS_u8_fmt_((uint8_t)(((width_) << 4U) & 0x7U) | (uint8_t)QS_I8_ENUM_T, \
                (data_)))

//${QS-macros::QS_U8} ........................................................
#define QS_U8(width_, data_) \
    (QS_u8_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_U8_T, (data_)))

//${QS-macros::QS_I16} .......................................................
#define QS_I16(width_, data_) \
    (QS_u16_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_I16_T, (data_)))

//${QS-macros::QS_U16} .......................................................
#define QS_U16(width_, data_) \
    (QS_u16_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_U16_T, (data_)))

//${QS-macros::QS_I32} .......................................................
#define QS_I32(width_, data_) \
    (QS_u32_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_I32_T, (data_)))

//${QS-macros::QS_U32} .......................................................
#define QS_U32(width_, data_) \
    (QS_u32_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_U32_T, (data_)))

//${QS-macros::QS_I64} .......................................................
#define QS_I64(width_, data_) \
    (QS_u64_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_I64_T, (data_)))

//${QS-macros::QS_U64} .......................................................
#define QS_U64(width_, data_) \
    (QS_u64_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_U64_T, (data_)))

//${QS-macros::QS_F32} .......................................................
#define QS_F32(width_, data_) \
    (QS_f32_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_F32_T, (data_)))

//${QS-macros::QS_F64} .......................................................
#define QS_F64(width_, data_) \
    (QS_f64_fmt_((uint8_t)(((width_) << 4)) | (uint8_t)QS_F64_T, (data_)))

//${QS-macros::QS_STR} .......................................................
#define QS_STR(str_) (QS_str_fmt_((str_)))

//${QS-macros::QS_MEM} .......................................................
#define QS_MEM(mem_, size_) (QS_mem_fmt_((mem_), (size_)))

//${QS-macros::QS_ENUM} ......................................................
#define QS_ENUM(group_, value_) \
    (QS_u8_fmt_((uint8_t)(0x80U | ((group_) << 4U)) | (uint8_t)QS_I8_ENUM_T,\
                (uint8_t)(value_)))

//${QS-macros::QS_TIME_PRE_} .................................................
#if (QS_TIME_SIZE == 4U)
#define QS_TIME_PRE_() (QS_u32_raw_(QS_onGetTime()))
#endif //  (QS_TIME_SIZE == 4U)

//${QS-macros::QS_TIME_PRE_} .................................................
#if (QS_TIME_SIZE == 2U)
#define QS_TIME_PRE_() (QS_u16_raw_(QS_onGetTime()))
#endif //  (QS_TIME_SIZE == 2U)

//${QS-macros::QS_TIME_PRE_} .................................................
#if (QS_TIME_SIZE == 1U)
#define QS_TIME_PRE_() (QS_u8_raw_(QS_onGetTime()))
#endif //  (QS_TIME_SIZE == 1U)

//${QS-macros::QS_OBJ} .......................................................
#if (QS_OBJ_PTR_SIZE == 4U)
#define QS_OBJ(obj_) (QS_u32_fmt_(QS_OBJ_T, (uint32_t)(obj_)))
#endif //  (QS_OBJ_PTR_SIZE == 4U)

//${QS-macros::QS_OBJ} .......................................................
#if (QS_OBJ_PTR_SIZE == 2U)
#define QS_OBJ(obj_) (QS_u16_fmt_(QS_OBJ_T, (uint16_t)(obj_)))
#endif //  (QS_OBJ_PTR_SIZE == 2U)

//${QS-macros::QS_OBJ} .......................................................
#if (QS_OBJ_PTR_SIZE == 1U)
#define QS_OBJ(obj_) (QS_u8_fmt_(QS_OBJ_T, (uint8_t)(obj_)))
#endif //  (QS_OBJ_PTR_SIZE == 1U)

//${QS-macros::QS_OBJ} .......................................................
#if (QS_OBJ_PTR_SIZE == 8U)
#define QS_OBJ(obj_) (QS_u64_fmt_(QS_OBJ_T, (uint64_t)(obj_)))
#endif //  (QS_OBJ_PTR_SIZE == 8U)

//${QS-macros::QS_FUN} .......................................................
#if (QS_FUN_PTR_SIZE == 4U)
#define QS_FUN(fun_) (QS_u32_fmt_(QS_FUN_T, (uint32_t)(fun_)))
#endif //  (QS_FUN_PTR_SIZE == 4U)

//${QS-macros::QS_FUN} .......................................................
#if (QS_FUN_PTR_SIZE == 2U)
#define QS_FUN(fun_) (QS_u16_fmt_(QS_FUN_T, (uint16_t)(fun_)))
#endif //  (QS_FUN_PTR_SIZE == 2U)

//${QS-macros::QS_FUN} .......................................................
#if (QS_FUN_PTR_SIZE == 1U)
#define QS_FUN(fun_) (QS_u8_fmt_(QS_FUN_T, (uint8_t)(fun_)))
#endif //  (QS_FUN_PTR_SIZE == 1U)

//${QS-macros::QS_FUN} .......................................................
#if (QS_FUN_PTR_SIZE == 8U)
#define QS_FUN(fun_) (QS_u64_fmt_(QS_FUN_T, (uint64_t)(fun_)))
#endif //  (QS_FUN_PTR_SIZE == 8U)

//${QS-macros::QS_SIG} .......................................................
#if (Q_SIGNAL_SIZE == 4U)
#define QS_SIG(sig_, obj_) \
        QS_u32_fmt_(QS_SIG_T, (sig_)); \
        QS_obj_raw_(obj_)
#endif //  (Q_SIGNAL_SIZE == 4U)

//${QS-macros::QS_SIG} .......................................................
#if (Q_SIGNAL_SIZE == 2U)
#define QS_SIG(sig_, obj_) \
        QS_u16_fmt_(QS_SIG_T, (sig_)); \
        QS_obj_raw_(obj_)
#endif //  (Q_SIGNAL_SIZE == 2U)

//${QS-macros::QS_SIG} .......................................................
#if (Q_SIGNAL_SIZE == 1U)
#define QS_SIG(sig_, obj_) \
        QS_u8_fmt_(QS_SIG_T, (sig_)); \
        QS_obj_raw_(obj_)
#endif //  (Q_SIGNAL_SIZE == 1U)

//${QS-macros::QS_SIG_DICTIONARY} ............................................
#define QS_SIG_DICTIONARY(sig_, obj_) \
    (QS_sig_dict_pre_((QSignal)(sig_), (obj_), #sig_))

//${QS-macros::QS_OBJ_DICTIONARY} ............................................
#define QS_OBJ_DICTIONARY(obj_) \
    (QS_obj_dict_pre_((obj_), #obj_))

//${QS-macros::QS_OBJ_ARR_DICTIONARY} ........................................
#define QS_OBJ_ARR_DICTIONARY(obj_, idx_) \
    (QS_obj_arr_dict_pre_((obj_), (idx_), #obj_))

//${QS-macros::QS_FUN_DICTIONARY} ............................................
#define QS_FUN_DICTIONARY(fun_) \
    (QS_fun_dict_pre_((void (*)(void))(fun_), #fun_))

//${QS-macros::QS_USR_DICTIONARY} ............................................
#define QS_USR_DICTIONARY(rec_) \
    (QS_usr_dict_pre_((rec_), #rec_))

//${QS-macros::QS_ENUM_DICTIONARY} ...........................................
#define QS_ENUM_DICTIONARY(value_, group_) \
    (QS_enum_dict_pre_((value_), (group_), #value_))

//${QS-macros::QF_QS_CRIT_ENTRY} .............................................
void QF_QS_CRIT_ENTRY(void);

//${QS-macros::QF_QS_CRIT_EXIT} ..............................................
void QF_QS_CRIT_EXIT(void);

//${QS-macros::QF_QS_ISR_ENTRY} ..............................................
void QF_QS_ISR_ENTRY(
    uint_fast8_t const isrnest,
    uint_fast8_t const prio_);

//${QS-macros::QF_QS_ISR_EXIT} ...............................................
void QF_QS_ISR_EXIT(
    uint_fast8_t isrnest,
    uint_fast8_t prio);

//${QS-macros::QF_QS_ACTION} .................................................
#define QF_QS_ACTION(act_) (act_)

//${QS-macros::QS_EOD} .......................................................
#define QS_EOD ((uint16_t)0xFFFFU)

//${QS-macros::QS_CMD} .......................................................
#define QS_CMD ((uint8_t)7U)

//${QS-macros::QS_HEX_FMT} ...................................................
#define QS_HEX_FMT ((uint8_t)0x0FU)

//${QS-macros::QS_CRIT_STAT_} ................................................
#ifndef QS_CRIT_STAT_
#define QS_CRIT_STAT_ QF_CRIT_STAT_
#endif // ndef QS_CRIT_STAT_

//${QS-macros::QS_CRIT_E_} ...................................................
#ifndef QS_CRIT_E_
#define QS_CRIT_E_() QF_CRIT_E_()
#endif // ndef QS_CRIT_E_

//${QS-macros::QS_CRIT_X_} ...................................................
#ifndef QS_CRIT_X_
#define QS_CRIT_X_() QF_CRIT_X_()
#endif // ndef QS_CRIT_X_

//${QS-macros::QS_MEM_SYS_} ..................................................
#ifndef QS_MEM_SYS_
#define QS_MEM_SYS_() QF_MEM_SYS_()
#endif // ndef QS_MEM_SYS_

//${QS-macros::QS_MEM_APP_} ..................................................
#ifndef QS_MEM_APP_
#define QS_MEM_APP_() QF_MEM_APP_()
#endif // ndef QS_MEM_APP_
//$enddecl${QS-macros} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${QS-filter} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QS-filter::filter_} ......................................................
//! @static @private @memberof QS
extern QSpyFilter QS_filter_;
//$enddecl${QS-filter} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//============================================================================
//! @cond INTERNAL

typedef struct {
    void const * locFilter_AP; //!< @deprecated
    uint8_t * buf;
    QSCtr end;
    QSCtr volatile head;
    QSCtr volatile tail;
    QSCtr volatile used;
    uint8_t volatile seq;
    uint8_t volatile chksum;
    uint8_t volatile critNest;
    uint8_t flags;
} QSAttr;

//! @static @private @memberof QS
extern QSAttr QS_priv_;

//! @static @private @memberof QS
void QS_glbFilter_(int_fast16_t const filter);

//! @static @private @memberof QS
void QS_locFilter_(int_fast16_t const filter);

//! @static @private @memberof QS
void QS_beginRec_(uint_fast8_t const rec);

//! @static @private @memberof QS
void QS_endRec_(void);

//! @static @private @memberof QS
void QS_u8_raw_(uint8_t const d);

//! @static @private @memberof QS
void QS_2u8_raw_(
    uint8_t const d1,
    uint8_t const d2);

//! @static @private @memberof QS
void QS_u16_raw_(uint16_t const d);

//! @static @private @memberof QS
void QS_u32_raw_(uint32_t const d);

//! @static @private @memberof QS
void QS_obj_raw_(void const * const obj);

//! @static @private @memberof QS
void QS_str_raw_(char const * const str);

//! @static @private @memberof QS
void QS_u8_fmt_(
    uint8_t const format,
    uint8_t const d);

//! @static @private @memberof QS
void QS_u16_fmt_(
    uint8_t const format,
    uint16_t const d);

//! @static @private @memberof QS
void QS_u32_fmt_(
    uint8_t const format,
    uint32_t const d);

//! @static @private @memberof QS
void QS_str_fmt_(char const * const str);

//! @static @private @memberof QS
void QS_mem_fmt_(
    uint8_t const * const blk,
    uint8_t const size);

//! @static @private @memberof QS
void QS_sig_dict_pre_(
    QSignal const sig,
    void const * const obj,
    char const * const name);

//! @static @private @memberof QS
void QS_obj_dict_pre_(
    void const * const obj,
    char const * const name);

//! @static @private @memberof QS
void QS_obj_arr_dict_pre_(
    void const * const obj,
    uint_fast16_t const idx,
    char const * const name);

//! @static @private @memberof QS
void QS_fun_dict_pre_(
    QSpyFunPtr const fun,
    char const * const name);

//! @static @private @memberof QS
void QS_usr_dict_pre_(
    enum_t const rec,
    char const * const name);

//! @static @private @memberof QS
void QS_enum_dict_pre_(
    enum_t const value,
    uint8_t const group,
    char const * const name);

//! @static @private @memberof QS
void QS_target_info_pre_(uint8_t const isReset);

//! @static @private @memberof QS
void QS_u64_raw_(uint64_t const d);

//! @static @private @memberof QS
void QS_u64_fmt_(
    uint8_t const format,
    uint64_t const d);

//! @static @private @memberof QS
void QS_f32_fmt_(
    uint8_t const format,
    float32_t const d);

//! @static @private @memberof QS
void QS_f64_fmt_(
    uint8_t const format,
    float64_t const d);

//! @endcond
//============================================================================

//$declare${QS-tx} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QS-tx::preType} ..........................................................
//! Enumerates data elements for app-specific trace records
enum QS_preType {
    QS_I8_ENUM_T, //!< signed 8-bit integer or enum format
    QS_U8_T,      //!< unsigned 8-bit integer format
    QS_I16_T,     //!< signed 16-bit integer format
    QS_U16_T,     //!< unsigned 16-bit integer format
    QS_I32_T,     //!< signed 32-bit integer format
    QS_U32_T,     //!< unsigned 32-bit integer format
    QS_F32_T,     //!< 32-bit floating point format
    QS_F64_T,     //!< 64-bit floating point format
    QS_STR_T,     //!< zero-terminated ASCII string format
    QS_MEM_T,     //!< up to 255-bytes memory block format
    QS_SIG_T,     //!< event signal format
    QS_OBJ_T,     //!< object pointer format
    QS_FUN_T,     //!< function pointer format
    QS_I64_T,     //!< signed 64-bit integer format
    QS_U64_T      //!< unsigned 64-bit integer format
};

//${QS-tx::initBuf} ..........................................................
//! @static @public @memberof QS
void QS_initBuf(
    uint8_t * const sto,
    uint_fast32_t const stoSize);

//${QS-tx::getByte} ..........................................................
//! @static @public @memberof QS
uint16_t QS_getByte(void);

//${QS-tx::getBlock} .........................................................
//! @static @public @memberof QS
uint8_t const * QS_getBlock(uint16_t * const pNbytes);

//${QS-tx::doOutput} .........................................................
//! @static @public @memberof QS
void QS_doOutput(void);

//${QS-tx::onStartup} ........................................................
//! @static @public @memberof QS
uint8_t QS_onStartup(void const * arg);

//${QS-tx::onCleanup} ........................................................
//! @static @public @memberof QS
void QS_onCleanup(void);

//${QS-tx::onFlush} ..........................................................
//! @static @public @memberof QS
void QS_onFlush(void);

//${QS-tx::onGetTime} ........................................................
//! @static @public @memberof QS
QSTimeCtr QS_onGetTime(void);

//${QS-tx::ASSERTION} ........................................................
//! @static @public @memberof QS
void QS_ASSERTION(
    char const * const module,
    int_t const id,
    uint32_t const delay);
//$enddecl${QS-tx} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//============================================================================
//! @cond INTERNAL

typedef struct {
    void * currObj[8];
    uint8_t * buf;
    QSCtr  end;
    QSCtr volatile head;
    QSCtr volatile tail;
#ifdef Q_UTEST
    bool inTestLoop;
#endif
} QSRxAttr;

//! @static @private @memberof QS
extern QSRxAttr QS_rxPriv_;

//! @endcond
//============================================================================

//$declare${QS-rx} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QS-rx::QSpyObjKind} ......................................................
//! @static @public @memberof QS
//! Kinds of objects used in QS::QS_setCurrObj() and QS::QS_queryCurrObj()
enum QS_QSpyObjKind {
    SM_OBJ,    //!< state machine object
    AO_OBJ,    //!< active object
    MP_OBJ,    //!< event pool object
    EQ_OBJ,    //!< raw queue object
    TE_OBJ,    //!< time event object
    AP_OBJ,    //!< generic Application-specific object
    MAX_OBJ
};

//${QS-rx::OSpyObjComb} ......................................................
//! @static @public @memberof QS
//! Object combinations for QS::QS_setCurrObj() and QS::QS_queryCurrObj()
enum QS_OSpyObjComb {
    SM_AO_OBJ = (enum_t)MAX_OBJ //!< combination of SM and AO
};

//${QS-rx::rxInitBuf} ........................................................
//! @static @public @memberof QS
void QS_rxInitBuf(
    uint8_t * const sto,
    uint16_t const stoSize);

//${QS-rx::rxPut} ............................................................
//! @static @public @memberof QS
static inline bool QS_rxPut(uint8_t const b) {
    QSCtr head = QS_rxPriv_.head + 1U;
    if (head == QS_rxPriv_.end) {
        head = 0U;
    }
    if (head != QS_rxPriv_.tail) { // buffer NOT full?
        QS_rxPriv_.buf[QS_rxPriv_.head] = b;
        QS_rxPriv_.head = head; // update the head to a *valid* index
        return true;  // byte placed in the buffer
    }
    else {
        return false; // byte NOT placed in the buffer
    }
}

//${QS-rx::rxGetNfree} .......................................................
//! @static @public @memberof QS
uint16_t QS_rxGetNfree(void);

//${QS-rx::doInput} ..........................................................
//! @static @public @memberof QS
void QS_doInput(void);

//${QS-rx::setCurrObj} .......................................................
//! @static @public @memberof QS
void QS_setCurrObj(
    uint8_t const obj_kind,
    void * const obj_ptr);

//${QS-rx::queryCurrObj} .....................................................
//! @static @public @memberof QS
void QS_queryCurrObj(uint8_t const obj_kind);

//${QS-rx::rxParse} ..........................................................
//! @static @public @memberof QS
void QS_rxParse(void);

//${QS-rx::onReset} ..........................................................
//! @static @public @memberof QS
void QS_onReset(void);

//${QS-rx::onCommand} ........................................................
//! @static @public @memberof QS
void QS_onCommand(
    uint8_t cmdId,
    uint32_t param1,
    uint32_t param2,
    uint32_t param3);

//${QS-rx::RX_PUT} ...........................................................
//! @static @public @memberof QS
bool QS_RX_PUT(uint8_t const b);
//$enddecl${QS-rx} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//============================================================================
#ifdef Q_UTEST

//$declare${QUTest} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QUTest::QS::TProbe} ......................................................
// @struct TProbe
struct QS_TProbe {
    QSFun    addr;
    uint32_t data;
    uint8_t  idx;
};

//${QUTest::QS::onTestSetup} .................................................
//! @static @public @memberof QS
void QS_onTestSetup(void);

//${QUTest::QS::onTestTeardown} ..............................................
//! @static @public @memberof QS
void QS_onTestTeardown(void);

//${QUTest::QS::onTestEvt} ...................................................
//! @static @public @memberof QS
void QS_onTestEvt(QEvt * e);

//${QUTest::QS::onTestPost} ..................................................
//! @static @public @memberof QS
void QS_onTestPost(
    void const * sender,
    QActive * recipient,
    QEvt const * e,
    bool status);

//${QUTest::QS::onTestLoop} ..................................................
//! @static @public @memberof QS
void QS_onTestLoop(void);

//${QUTest::QUTEST_ON_POST} ..................................................
//! record-ID for posting events
#define QUTEST_ON_POST 124
//$enddecl${QUTest} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//============================================================================
//! @cond INTERNAL

typedef struct {
    uint_fast8_t intLock; //!< interrupt lock up-down counter
    struct QS_TProbe tpBuf[16]; //!< buffer of Test-Probes received so far
    uint8_t   tpNum;     //!< current # Test-Probes
    QSTimeCtr testTime;  //!< test time (tick counter)
} QSTestAttr;

extern QSTestAttr QS_tstPriv_;

void QS_test_pause_(void);
uint32_t QS_getTestProbe_(QSpyFunPtr const api);

//! @endcond
//============================================================================

// QP-stub for QUTest
// NOTE: The QP-stub is needed for unit testing QP applications,
// but might NOT be needed for testing QP itself.
#if Q_UTEST != 0

//! @cond INTERNAL
//$declare${QUTest-stub::QF::readySet_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QUTest-stub::QF::readySet_} ..............................................
//! @static @private @memberof QF
extern QPSet QF_readySet_;
//$enddecl${QUTest-stub::QF::readySet_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${QUTest-stub::QF::readySet_dis_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QUTest-stub::QF::readySet_dis_} ..........................................
#ifndef Q_UNSAFE
//! @static @private @memberof QF
extern QPSet QF_readySet_dis_;
#endif // ndef Q_UNSAFE
//$enddecl${QUTest-stub::QF::readySet_dis_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void QS_processTestEvts_(void);
//! @endcond

//$declare${QUTest-stub::QHsmDummy} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QUTest-stub::QHsmDummy} ..................................................
//! @class QHsmDummy
//! @extends QHsm
typedef struct {
// protected:
    QAsm super;
} QHsmDummy;

// public:

//! @public @memberof QHsmDummy
void QHsmDummy_ctor(QHsmDummy * const me);

//! @private @memberof QHsmDummy
void QHsmDummy_init_(
    QAsm * const me,
    void const * const par,
    uint_fast8_t const qs_id);

// private:

//! @private @memberof QHsmDummy
void QHsmDummy_dispatch_(
    QAsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id);
//$enddecl${QUTest-stub::QHsmDummy} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${QUTest-stub::QActiveDummy} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QUTest-stub::QActiveDummy} ...............................................
//! @class QActiveDummy
//! @extends QActive
typedef struct {
// protected:
    QActive super;
} QActiveDummy;

// public:

//! @public @memberof QActiveDummy
void QActiveDummy_ctor(QActiveDummy * const me);

// private:

//! @private @memberof QActiveDummy
void QActiveDummy_init_(
    QAsm * const me,
    void const * const par,
    uint_fast8_t const qs_id);

//! @private @memberof QActiveDummy
void QActiveDummy_dispatch_(
    QAsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id);

//! @private @memberof QActiveDummy
bool QActiveDummy_post_(
    QActive * const me,
    QEvt const * const e,
    uint_fast16_t const margin,
    void const * const sender);

//! @private @memberof QActiveDummy
void QActiveDummy_postLIFO_(
    QActive * const me,
    QEvt const * const e);
//$enddecl${QUTest-stub::QActiveDummy} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif // Q_UTEST != 0

#define QS_TEST_PROBE_DEF(fun_) \
    uint32_t const qs_tp_ = QS_getTestProbe_((void (*)(void))(fun_));

#define QS_TEST_PROBE(code_) \
    if (qs_tp_ != 0U) { code_ }

#define QS_TEST_PROBE_ID(id_, code_) \
    if (qs_tp_ == (uint32_t)(id_)) { code_ }

#define QS_TEST_PAUSE()  (QS_test_pause_())

#else // Q_UTEST not defined

// dummy definitions when not building for QUTEST
#define QS_TEST_PROBE_DEF(fun_)
#define QS_TEST_PROBE(code_)
#define QS_TEST_PROBE_ID(id_, code_)
#define QS_TEST_PAUSE()  ((void)0)

#endif // Q_UTEST

#endif // QS_H_
