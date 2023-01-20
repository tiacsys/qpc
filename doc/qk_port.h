/*! @file
* @brief Sample QK/C port
*
* @details
* This is just an example of a QK port for a generic C99 compiler.
* Other specific QK ports will define the QK facilities differently.
*/
#ifndef QK_PORT_H_
#define QK_PORT_H_

/*! Check if the code executes in the ISR context */
#define QK_ISR_CONTEXT_() (QK_priv_.intNest != 0U)

/*! Define the ISR entry sequence */
#define QK_ISR_ENTRY()                               \
do {                                                 \
    QF_INT_DISABLE();                                \
    ++QK_priv_.intNest;                              \
    QF_QS_ISR_ENTRY(QK_priv_.intNest, QK_currPrio_); \
    QF_INT_ENABLE();                            \
} while (false)

/*! Define the ISR exit sequence */
#define QK_ISR_EXIT()             \
do {                              \
    QF_INT_DISABLE();             \
    --QK_priv_.intNest;           \
    if (QK_priv_.intNest == 0U) { \
        if (QK_sched_() != 0U) {  \
            QK_activate_();       \
        }                         \
    }                             \
    QF_INT_ENABLE();              \
} while (false)

#include "qk.h" /* QK platform-independent public interface */

#endif /* QK_PORT_H_ */
