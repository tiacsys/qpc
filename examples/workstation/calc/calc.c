//$file${.::calc.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: calc.qm
// File:  ${.::calc.c}
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
//$endhead${.::calc.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include "qpc.h"   // QP/C
#include "bsp.h"   // board support package
#include "calc.h"  // application

Q_DEFINE_THIS_FILE

#define KEY_NULL    '\0'
#define KEY_PLUS    '+'
#define KEY_MINUS   '-'
#define KEY_MULT    '*'
#define KEY_DIVIDE  '/'

//$declare${SMs::Calc} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${SMs::Calc} ...............................................................
typedef struct Calc {
// protected:
    QHsm super;

// public:

// private:
    double op1;
    double op2;
    uint8_t oper1;
    uint8_t oper2;
} Calc;

// private:

// guard function to evaluate the current expression
static bool Calc_eval(Calc * const me,
    double op,
    uint8_t oper);
extern Calc Calc_inst;

// protected:
static QState Calc_initial(Calc * const me, void const * const par);
static QState Calc_on(Calc * const me, QEvt const * const e);
// Error state after evaluation of an expression.
static QState Calc_error(Calc * const me, QEvt const * const e);
static QState Calc_ready(Calc * const me, QEvt const * const e);
static QState Calc_result(Calc * const me, QEvt const * const e);
static QState Calc_begin(Calc * const me, QEvt const * const e);
static QState Calc_operand1(Calc * const me, QEvt const * const e);
static QState Calc_zero1(Calc * const me, QEvt const * const e);
static QState Calc_int1(Calc * const me, QEvt const * const e);
static QState Calc_frac1(Calc * const me, QEvt const * const e);
static QState Calc_negated1(Calc * const me, QEvt const * const e);
static QState Calc_opEntered(Calc * const me, QEvt const * const e);
static QState Calc_operand2(Calc * const me, QEvt const * const e);
static QState Calc_zero2(Calc * const me, QEvt const * const e);
static QState Calc_int2(Calc * const me, QEvt const * const e);
static QState Calc_frac2(Calc * const me, QEvt const * const e);
static QState Calc_negated2(Calc * const me, QEvt const * const e);
static QState Calc_final(Calc * const me, QEvt const * const e);
//$enddecl${SMs::Calc} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Check for the minimum required QP version
#if (QP_VERSION < 730U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.3.0 or higher required
#endif
//$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//$define${SMs::the_calc} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${SMs::the_calc} ...........................................................
QHsm * const the_calc = &Calc_inst.super;
//$enddef${SMs::the_calc} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${SMs::Calc_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${SMs::Calc_ctor} ..........................................................
void Calc_ctor(void) {
    Calc *me = &Calc_inst;
    QHsm_ctor(&me->super, Q_STATE_CAST(&Calc_initial));
}
//$enddef${SMs::Calc_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//$define${SMs::Calc} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

//${SMs::Calc} ...............................................................
Calc Calc_inst;

//${SMs::Calc::eval} .........................................................
static bool Calc_eval(Calc * const me,
    double op,
    uint8_t oper)
{
    double result;
    if ((oper == KEY_NULL) || (oper == KEY_PLUS) || (oper == KEY_MINUS)) {
        switch (me->oper2) {
            case KEY_MULT: {
                me->op2 *= op;
                break;
            }
            case KEY_DIVIDE: {
                if ((-1e-30 < op) && (op < 1e-30)) {
                    BSP_display_error(" Error 0 "); // divide by zero
                    return false;
                }
                me->op2 /= op;
                break;
             }
             default: { // no op2 yet
                me->op2 = op;
                me->oper2 = oper;
                break;
             }
        }
        switch (me->oper1) {
            case KEY_PLUS: {
                me->op1 += me->op2;
                break;
            }
            case KEY_MINUS: {
                me->op1 -= me->op2;
                break;
            }
            case KEY_MULT: {
                me->op1 *= me->op2;
                break;
            }
            case KEY_DIVIDE: {
                if ((-1e-30 < me->op2) && (me->op2 < 1e-30)) {
                    BSP_display_error(" Error 0 "); // divide by zero
                    return false;
                }
                me->op1 /= me->op2;
                break;
            }
            default: {
                Q_ERROR();
                break;
            }
        }
        me->oper1 = oper;
        me->oper2 = KEY_NULL;
        result = me->op1;
    }
    else { // (oper == KEY_MULT) || (oper == KEY_DIV)
        switch (me->oper2) {
            case KEY_MULT: {
                me->op2 *= op;
                break;
            }
            case KEY_DIVIDE: {
                if ((-1e-30 < op) && (op < 1e-30)) {
                    BSP_display_error(" Error 0 "); // divide by zero
                    return false;
                }
                me->op2 /= op;
                break;
            }
            default: { // oper2 not provided yet
                me->op2 = op;
                break;
            }
        }
        me->oper2 = oper;
        result = me->op2;
    }

    if ((result < -99999999.0) || (99999999.0 < result)) {
        BSP_display_error(" Error 1 "); // out of range
        return false;
    }
    if ((-0.0000001 < result) && (result < 0.0000001)) {
        result = 0.0;
    }
    BSP_display(result);

    return true;
}

//${SMs::Calc::SM} ...........................................................
static QState Calc_initial(Calc * const me, void const * const par) {
    //${SMs::Calc::SM::initial}
    (void)par; // unused parameter
    return Q_TRAN(&Calc_on);
}

//${SMs::Calc::SM::on} .......................................................
static QState Calc_on(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on}
        case Q_ENTRY_SIG: {
            BSP_message("on-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on}
        case Q_EXIT_SIG: {
            BSP_message("on-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::initial}
        case Q_INIT_SIG: {
            BSP_message("on-INIT;");
            BSP_clear();
            status_ = Q_TRAN(&Calc_ready);
            break;
        }
        //${SMs::Calc::SM::on::C}
        case C_SIG: {
            status_ = Q_TRAN(&Calc_on);
            break;
        }
        //${SMs::Calc::SM::on::OFF}
        case OFF_SIG: {
            status_ = Q_TRAN(&Calc_final);
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::error} ................................................
static QState Calc_error(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::error}
        case Q_ENTRY_SIG: {
            BSP_message("error-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::error}
        case Q_EXIT_SIG: {
            BSP_message("error-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_on);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::ready} ................................................
static QState Calc_ready(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::ready}
        case Q_ENTRY_SIG: {
            BSP_message("ready-ENTRY;");
            me->oper2 = KEY_NULL;
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::ready}
        case Q_EXIT_SIG: {
            BSP_message("ready-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::ready::initial}
        case Q_INIT_SIG: {
            BSP_message("ready-INIT;");
            status_ = Q_TRAN(&Calc_begin);
            break;
        }
        //${SMs::Calc::SM::on::ready::DIGIT_0}
        case DIGIT_0_SIG: {
            BSP_clear();
            status_ = Q_TRAN(&Calc_zero1);
            break;
        }
        //${SMs::Calc::SM::on::ready::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_clear();
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int1);
            break;
        }
        //${SMs::Calc::SM::on::ready::POINT}
        case POINT_SIG: {
            BSP_clear();
            BSP_insert((int)'0');
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac1);
            break;
        }
        //${SMs::Calc::SM::on::ready::OPER}
        case OPER_SIG: {
            me->op1 = BSP_get_value();
            me->oper1 = Q_EVT_CAST(CalcEvt)->key_code;
            status_ = Q_TRAN(&Calc_opEntered);
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_on);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::ready::result} ........................................
static QState Calc_result(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::ready::result}
        case Q_ENTRY_SIG: {
            BSP_message("result-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::ready::result}
        case Q_EXIT_SIG: {
            BSP_message("result-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_ready);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::ready::begin} .........................................
static QState Calc_begin(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::ready::begin}
        case Q_ENTRY_SIG: {
            BSP_message("begin-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::ready::begin}
        case Q_EXIT_SIG: {
            BSP_message("begin-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::ready::begin::OPER}
        case OPER_SIG: {
            //${SMs::Calc::SM::on::ready::begin::OPER::[e->key=='-']}
            if (Q_EVT_CAST(CalcEvt)->key_code == KEY_MINUS) {
                status_ = Q_TRAN(&Calc_negated1);
            }
            //${SMs::Calc::SM::on::ready::begin::OPER::[else]}
            else {
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_ready);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand1} .............................................
static QState Calc_operand1(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand1}
        case Q_ENTRY_SIG: {
            BSP_message("operand1-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1}
        case Q_EXIT_SIG: {
            BSP_message("operand1-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::CE}
        case CE_SIG: {
            BSP_clear();
            status_ = Q_TRAN(&Calc_ready);
            break;
        }
        //${SMs::Calc::SM::on::operand1::OPER}
        case OPER_SIG: {
            me->op1 = BSP_get_value();
            me->oper1 = Q_EVT_CAST(CalcEvt)->key_code;
            status_ = Q_TRAN(&Calc_opEntered);
            break;
        }
        //${SMs::Calc::SM::on::operand1::EQUALS}
        case EQUALS_SIG: {
            status_ = Q_TRAN(&Calc_result);
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_on);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand1::zero1} ......................................
static QState Calc_zero1(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand1::zero1}
        case Q_ENTRY_SIG: {
            BSP_message("zero1-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::zero1}
        case Q_EXIT_SIG: {
            BSP_message("zero1-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::zero1::DIGIT_0}
        case DIGIT_0_SIG: {
            ;
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::zero1::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int1);
            break;
        }
        //${SMs::Calc::SM::on::operand1::zero1::POINT}
        case POINT_SIG: {
            BSP_insert((int)'0');
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac1);
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand1);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand1::int1} .......................................
static QState Calc_int1(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand1::int1}
        case Q_ENTRY_SIG: {
            BSP_message("int1-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::int1}
        case Q_EXIT_SIG: {
            BSP_message("int1-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::int1::POINT}
        case POINT_SIG: {
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac1);
            break;
        }
        //${SMs::Calc::SM::on::operand1::int1::DIGIT_0, DIGIT_1_9}
        case DIGIT_0_SIG: // intentionally fall through
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand1);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand1::frac1} ......................................
static QState Calc_frac1(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand1::frac1}
        case Q_ENTRY_SIG: {
            BSP_message("frac1-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::frac1}
        case Q_EXIT_SIG: {
            BSP_message("frac1-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::frac1::POINT}
        case POINT_SIG: {
            ;
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::frac1::DIGIT_0, DIGIT_1_9}
        case DIGIT_0_SIG: // intentionally fall through
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand1);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand1::negated1} ...................................
static QState Calc_negated1(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand1::negated1}
        case Q_ENTRY_SIG: {
            BSP_message("negated1-ENTRY;");
            BSP_negate();
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::negated1}
        case Q_EXIT_SIG: {
            BSP_message("negated1-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand1::negated1::DIGIT_0}
        case DIGIT_0_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_zero1);
            break;
        }
        //${SMs::Calc::SM::on::operand1::negated1::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int1);
            break;
        }
        //${SMs::Calc::SM::on::operand1::negated1::POINT}
        case POINT_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_frac1);
            break;
        }
        //${SMs::Calc::SM::on::operand1::negated1::OPER}
        case OPER_SIG: {
            //${SMs::Calc::SM::on::operand1::negated1::OPER::[e->key=='-']}
            if (Q_EVT_CAST(CalcEvt)->key_code == KEY_MINUS) {
                ;
                status_ = Q_HANDLED();
            }
            //${SMs::Calc::SM::on::operand1::negated1::OPER::[else]}
            else {
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand1);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::opEntered} ............................................
static QState Calc_opEntered(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::opEntered}
        case Q_ENTRY_SIG: {
            BSP_message("opEntered-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::opEntered}
        case Q_EXIT_SIG: {
            BSP_message("opEntered-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::opEntered::DIGIT_0}
        case DIGIT_0_SIG: {
            BSP_clear();
            status_ = Q_TRAN(&Calc_zero2);
            break;
        }
        //${SMs::Calc::SM::on::opEntered::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_clear();
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int2);
            break;
        }
        //${SMs::Calc::SM::on::opEntered::POINT}
        case POINT_SIG: {
            BSP_clear();
            BSP_insert((int)'0');
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac2);
            break;
        }
        //${SMs::Calc::SM::on::opEntered::OPER}
        case OPER_SIG: {
            //${SMs::Calc::SM::on::opEntered::OPER::[e->key=='-']}
            if (Q_EVT_CAST(CalcEvt)->key_code == KEY_MINUS) {
                status_ = Q_TRAN(&Calc_negated2);
            }
            //${SMs::Calc::SM::on::opEntered::OPER::[else]}
            else {
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_on);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand2} .............................................
static QState Calc_operand2(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand2}
        case Q_ENTRY_SIG: {
            BSP_message("operand2-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2}
        case Q_EXIT_SIG: {
            BSP_message("operand2-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::CE}
        case CE_SIG: {
            BSP_clear();
            status_ = Q_TRAN(&Calc_opEntered);
            break;
        }
        //${SMs::Calc::SM::on::operand2::EQUALS}
        case EQUALS_SIG: {
            //${SMs::Calc::SM::on::operand2::EQUALS::[Calc_eval(me,BSP_get_value(),KE~}
            if (Calc_eval(me, BSP_get_value(), KEY_NULL)) {
                status_ = Q_TRAN(&Calc_result);
            }
            //${SMs::Calc::SM::on::operand2::EQUALS::[else]}
            else {
                status_ = Q_TRAN(&Calc_error);
            }
            break;
        }
        //${SMs::Calc::SM::on::operand2::OPER}
        case OPER_SIG: {
            //${SMs::Calc::SM::on::operand2::OPER::[Calc_eval(me,BSP_get_value(),Q_~}
            if (Calc_eval(me, BSP_get_value(), Q_EVT_CAST(CalcEvt)->key_code)) {
                status_ = Q_TRAN(&Calc_opEntered);
            }
            //${SMs::Calc::SM::on::operand2::OPER::[else]}
            else {
                status_ = Q_TRAN(&Calc_error);
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_on);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand2::zero2} ......................................
static QState Calc_zero2(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand2::zero2}
        case Q_ENTRY_SIG: {
            BSP_message("zero2-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::zero2}
        case Q_EXIT_SIG: {
            BSP_message("zero2-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::zero2::DIGIT_0}
        case DIGIT_0_SIG: {
            ;
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::zero2::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int2);
            break;
        }
        //${SMs::Calc::SM::on::operand2::zero2::POINT}
        case POINT_SIG: {
            BSP_insert((int)'0');
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac2);
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand2);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand2::int2} .......................................
static QState Calc_int2(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand2::int2}
        case Q_ENTRY_SIG: {
            BSP_message("int2-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::int2}
        case Q_EXIT_SIG: {
            BSP_message("int2-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::int2::POINT}
        case POINT_SIG: {
            BSP_insert((int)'.');
            status_ = Q_TRAN(&Calc_frac2);
            break;
        }
        //${SMs::Calc::SM::on::operand2::int2::DIGIT_0, DIGIT_1_9}
        case DIGIT_0_SIG: // intentionally fall through
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand2);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand2::frac2} ......................................
static QState Calc_frac2(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand2::frac2}
        case Q_ENTRY_SIG: {
            BSP_message("frac2-ENTRY;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::frac2}
        case Q_EXIT_SIG: {
            BSP_message("frac2-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::frac2::POINT}
        case POINT_SIG: {
            ;
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::frac2::DIGIT_0, DIGIT_1_9}
        case DIGIT_0_SIG: // intentionally fall through
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand2);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::on::operand2::negated2} ...................................
static QState Calc_negated2(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::on::operand2::negated2}
        case Q_ENTRY_SIG: {
            BSP_message("negated2-ENTRY;");
            BSP_negate();
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::negated2}
        case Q_EXIT_SIG: {
            BSP_message("negated2-EXIT;");
            status_ = Q_HANDLED();
            break;
        }
        //${SMs::Calc::SM::on::operand2::negated2::DIGIT_0}
        case DIGIT_0_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_zero2);
            break;
        }
        //${SMs::Calc::SM::on::operand2::negated2::DIGIT_1_9}
        case DIGIT_1_9_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_int2);
            break;
        }
        //${SMs::Calc::SM::on::operand2::negated2::POINT}
        case POINT_SIG: {
            BSP_insert(Q_EVT_CAST(CalcEvt)->key_code);
            status_ = Q_TRAN(&Calc_frac2);
            break;
        }
        //${SMs::Calc::SM::on::operand2::negated2::OPER}
        case OPER_SIG: {
            //${SMs::Calc::SM::on::operand2::negated2::OPER::[e->key=='-']}
            if (Q_EVT_CAST(CalcEvt)->key_code == KEY_MINUS) {
                ;
                status_ = Q_HANDLED();
            }
            //${SMs::Calc::SM::on::operand2::negated2::OPER::[else]}
            else {
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Calc_operand2);
            break;
        }
    }
    return status_;
}

//${SMs::Calc::SM::final} ....................................................
static QState Calc_final(Calc * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        //${SMs::Calc::SM::final}
        case Q_ENTRY_SIG: {
            BSP_message("final-ENTRY;");
            BSP_exit();
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
//$enddef${SMs::Calc} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
