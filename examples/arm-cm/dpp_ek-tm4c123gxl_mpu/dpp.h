//$file${.::dpp.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: dpp.qm
// File:  ${.::dpp.h}
//
// This code has been generated by QM 5.3.0 <www.state-machine.com/qm>.
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This generated code is open source software: you can redistribute it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation.
//
// This code is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// NOTE:
// Alternatively, this generated code may be distributed under the terms
// of Quantum Leaps commercial licenses, which expressly supersede the GNU
// General Public License and are specifically designed for licensees
// interested in retaining the proprietary status of their code.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//
//$endhead${.::dpp.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef DPP_H_
#define DPP_H_

enum DPPSignals {
    EAT_SIG = Q_USER_SIG, // published by Table to let a philosopher eat
    DONE_SIG,       // published by Philosopher when done eating
    PAUSE_SIG,      // published by BSP to pause serving forks
    SERVE_SIG,      // published by BSP to serve re-start serving forks
    TEST_SIG,       // published by BSP to test the application
    MAX_PUB_SIG,    // the last published signal

    HUNGRY_SIG,     // posted direclty to Table from hungry Philo
    TIMEOUT_SIG,    // used by Philosophers for time events
    MAX_SIG         // the last signal
};

//$declare${Events::TableEvt} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${Events::TableEvt} ........................................................
typedef struct {
// protected:
    QEvt super;

// public:
    uint8_t philoNum;
} TableEvt;
//$enddecl${Events::TableEvt} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

enum {
    N_PHILO = 5     // # Philos
};

//$declare${AOs::Philo_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::Philo_ctor} .........................................................
void Philo_ctor(
    QActive * const act,
    uint8_t const nPhilo,
    void const * const mpu);
//$enddecl${AOs::Philo_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${AOs::AO_Philo[N_PHILO]} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::AO_Philo[N_PHILO]} ..................................................
extern QActive * const AO_Philo[N_PHILO];
//$enddecl${AOs::AO_Philo[N_PHILO]} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$declare${AOs::Table_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::Table_ctor} .........................................................
void Table_ctor(
    QActive * const act,
    void const * const mpu);
//$enddecl${AOs::Table_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$declare${AOs::AO_Table} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${AOs::AO_Table} ...........................................................
extern QActive * const AO_Table;
//$enddecl${AOs::AO_Table} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#ifdef QXK_H_
void Test1_ctor(QXThread * const thr, void const * const mpu);
extern QXThread * const XT_Test1;
extern void * const XT_Test1_stackSto;
extern uint_fast16_t const XT_Test1_stackSize;

void Test2_ctor(QXThread * const thr, void const * const mpu);
extern QXThread * const XT_Test2;
extern void * const XT_Test2_stackSto;
extern uint_fast16_t const XT_Test2_stackSize;
#endif // QXK_H_

#endif // DPP_H_
