//$file${include::qf_pkg.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: qpc.qm
// File:  ${include::qf_pkg.h}
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
//$endhead${include::qf_pkg.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef QF_PKG_H_
#define QF_PKG_H_

//$declare${QF::QF} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QF} ..................................................................
//! @class QF
typedef struct {
// private:

#if (QF_MAX_EPOOL > 0U)
    //! @private @memberof QF
    QF_EPOOL_TYPE_ ePool_[QF_MAX_EPOOL];
#endif //  (QF_MAX_EPOOL > 0U)

#if (QF_MAX_EPOOL > 0U)
    //! @private @memberof QF
    uint_fast8_t maxPool_;
#endif //  (QF_MAX_EPOOL > 0U)
} QF;
//$enddecl${QF::QF} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//============================================================================
//! @cond INTERNAL

//$declare${QF::QF-pkg} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${QF::QF-pkg::priv_} .......................................................
//! @static @private @memberof QF
extern QF QF_priv_;

//${QF::QF-pkg::bzero_} ......................................................
//! @static @private @memberof QF
//! @static @private @memberof QF
void QF_bzero_(
    void * const start,
    uint_fast16_t const len);
//$enddecl${QF::QF-pkg} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// Bitmasks are for the QTimeEvt::refCtr_ attribute (inherited from ::QEvt).
// In the ::QTimeEvt class, this attribute is NOT used for reference counting
// because the QEVT_POOLID_(e) check for time events is zero (time events
// are NOT allocated dynamically).
#define QTE_IS_LINKED      (1U << 7U)
#define QTE_WAS_DISARMED   (1U << 6U)
#define QTE_TICK_RATE      0x0FU

//! @endcond
//============================================================================

//! @private @memberof QEvt
static inline void QEvt_refCtr_inc_(QEvt const *me) {
    ++((QEvt *)me)->refCtr_;
}

//! @private @memberof QEvt
static inline void QEvt_refCtr_dec_(QEvt const *me) {
    --((QEvt *)me)->refCtr_;
}

#define QACTIVE_CAST_(ptr_) ((QActive *)(ptr_))
#define Q_UINTPTR_CAST_(ptr_) ((uintptr_t)(ptr_))

#endif // QF_PKG_H_
