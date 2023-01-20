/*! @file
* @brief Sample QS/C port
*
* @details
* This is just an example of a QS port for a 32-bit CPU.
* Other specific QS ports will define the QS facilities differently.
*
* @remark
* QS might be used with or without the other QP framework components,
* in which case the separate definitions of the macros QS_CRIT_STAT_,
* QS_CRIT_E_(), and QS_CRIT_X_() are needed. In this sample port QS is
* configured to be used with the other QP component, by simply including
* "qf_port.h" *before* "qs.h".
*/
#ifndef QS_PORT_H_
#define QS_PORT_H_

/*! QS time-stamp size in bytes */
#define QS_TIME_SIZE     4U

/*! object pointer size in bytes */
#define QS_OBJ_PTR_SIZE  4U

/*! function pointer size in bytes */
#define QS_FUN_PTR_SIZE  4U

/*! QS buffer-counters size in bytes. Valid values: 2U or 4U;
* default 2U.
*/
#define QS_CTR_SIZE      2U

/*! QS time stamp size in bytes. Valid values: 1U, 2U, or 4U;
* default 4U.
*/
#define QS_TIME_SIZE     4U

#ifndef QF_PORT_H_
#include "qf_port.h" /* use QS with QF */
#endif

#include "qs.h" /* QS platform-independent public interface */

#endif /* QS_PORT_H_ */
