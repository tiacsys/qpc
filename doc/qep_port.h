/*! @file
* @brief Sample QEP/C port
*
* @details
* This is just an example of a QEP port for a generic C99 compiler.
* Other specific QEP ports will define the QEP facilities differently.
*/
#ifndef QEP_PORT_H_
#define QEP_PORT_H_

/*! @brief The size (in bytes) of the signal of an event. Valid values:
* 1U, 2U, or 4U; default 2U
*
* @details
* This macro can be defined in the QEP port file (qep_port.h) to
* configure the ::QSignal type. When the macro is undefined, the
* default of 2 bytes is applied in qep.h.
*/
#define Q_SIGNAL_SIZE  2U

/*! @brief No-return specifier for the Q_onAssert() callback function.
*
* @details
* If the `Q_NORETURN` macro is undefined, the default definition uses
* the C99 specifier `_Noreturn`.
*
* @note
* The `Q_NORETURN` macro can be defined in the QP port (typically in
* `qep_port.h`). If such definition is provided the default won't be used.
*/
#define Q_NORETURN  _Noreturn  void

#include <stdint.h>  /* Exact-width types. WG14/N843 C99 Standard */
#include <stdbool.h> /* Boolean type.      WG14/N843 C99 Standard */

#include "qep.h"     /* QEP platform-independent public interface */

#endif /* QEP_PORT_H_ */
