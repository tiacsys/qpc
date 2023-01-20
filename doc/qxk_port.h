/*! @file
* @brief Sample QXK/C port
*
* @details
* This is just an example of a QXK port for a generic C99 compiler.
* Other specific QXK ports will define the QXK facilities differently.
*/
#ifndef QXK_PORT_H_
#define QXK_PORT_H_

/*! Check if the code executes in the ISR context */
#define QXK_ISR_CONTEXT_() (QXK_get_IPSR() != 0U)

/*! Trigger context switch (used internally in QXK only) */
#define QXK_CONTEXT_SWITCH_() (trigPendSV())

/*! Define the ISR entry sequence */
#define QXK_ISR_ENTRY() ((void)0)

/*! Define the ISR exit sequence */
#define QXK_ISR_EXIT()  do {                                  \
    QF_INT_DISABLE();                                         \
    if (QXK_sched_() != 0U) {                                 \
        *Q_UINT2PTR_CAST(uint32_t, 0xE000ED04U) = (1U << 28U);\
    }                                                         \
    QF_INT_ENABLE();                                          \
    QXK_ARM_ERRATUM_838869();                                 \
} while (false)

#include "qxk.h" /* QXK platform-independent public interface */

#endif /* QXK_PORT_H_ */
