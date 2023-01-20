/*! @file
* @brief Sample QV/C port
*
* @details
* This is just an example of a QV port for a generic C99 compiler.
* Other specific QV ports will define the QV facilities differently.
*/
#ifndef QV_PORT_H_
#define QV_PORT_H_

/*! Macro to put the CPU to sleep **safely** in the cooperative
* QV kernel (to be called from QV_onIdle()).
*/
#define QV_CPU_SLEEP()     \
do {                       \
    __disable_interrupt(); \
    QF_INT_ENABLE();       \
    __WFI();               \
    __enable_interrupt();  \
} while (false)

#include "qv.h" /* QV platform-independent public interface */

#endif /* QV_PORT_H_ */
