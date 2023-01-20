/*! @file
* @brief Sample QF/C port
*
* @details
* This is just an example of a QF port for a generic C99 compiler.
* Other specific QF ports will define the QF facilities differently.
*/
#ifndef QF_PORT_H_
#define QF_PORT_H_

/*! Macro to configure the maximum number of Active Objects to be managed
* by the QF framework. Range [1U..64U] (configurable in qf_port.h)
*/
#define QF_MAX_ACTIVE    64U

/*! Maximum number of clock rates (configurable in qf_port.h)
* Valid values: [0U..15U]; default 1U
*/
#define QF_MAX_TICK_RATE 1U

/*! Maximum number of event pools (configurable in qf_port.h)
* Valid values: [0U..15U]; default 3U
*
* @note
* #QF_MAX_EPOOL set to zero means that event pools are NOT configured
* and consequently mutable events should NOT be used in the application.
*/
#define QF_MAX_EPOOL 3U

/*! Size of the QTimeEvt counter (configurable in qf_port.h)
* Valid values: 1U, 2U, or 4U; default 4U
*/
#define QF_TIMEEVT_CTR_SIZE 4U

/*! Size of the event-size (configurable value in qf_port.h)
* Valid values: 1U, 2U, or 4U; default 2U
*/
#define QF_EVENT_SIZ_SIZE   2U

/*! Size of the ::QMPoolSize data type (configurable in qf_port.h).
* Valid values 1U, 2U, or 4U; default 2U
*/
#define QF_MPOOL_SIZ_SIZE 2U

/*! Size of the ::QMPoolCtr data type (configurable in qf_port.h).
* Valid values 1U, 2U, or 4U; default 2U
*/
#define QF_MPOOL_CTR_SIZE 2U

/* interrupt disabling mechanism -------------------------------------------*/
/*! Disable interrupts */
#define QF_INT_DISABLE()    intDisable()

/*! Enable interrupts */
#define QF_INT_ENABLE()     intEnable()

/* QF critical section mechanism -------------------------------------------*/
/*! Define the critical section status that was present before entering
* the critical section.
*
* @details
* For critical sections that are allowed to nest, the critical section
* status must be saved and restored at the end. This macro provides the
* storage for saving the status.
*
* @note
* This macro might be empty, in which case the critical section status
* is not saved or restored. Such critical sections won't be able to nest.
* Also, note that the macro should be invoked without the closing semicolon.
*/
#define QF_CRIT_STAT_       crit_stat_t crit_stat_;

/*! Enter the critical section
*
* @details
* If the critical section status is provided, the macro saves the
* critical section status from before entering the critical section.
* Otherwise, the macro just unconditionally enters the critical section
* without saving the status.
*/
#define QF_CRIT_E_()        (crit_stat_ = critEntry())

/*! Exit the critical section
*
* @details
* If the critical section status is provided, the macro restores the
* critical section status saved by QF_CRIT_E_(). Otherwise, the macro
* just unconditionally exits the critical section.
*/
#define QF_CRIT_X_()        critExit(crit_stat_)

typedef unsigned int crit_stat_t;
crit_stat_t critEntry(void);
void critExit(crit_stat_t stat);

#ifdef QF_MEM_ISOLATE

    /* Memory isolation requires the context-switch */
    #define QF_ON_CONTEXT_SW   1U

    /*! Memory System setting */
    #define QF_MEM_SYS_() QF_onMemSys()

    /*! Memory Application setting */
    #define QF_MEM_APP_() QF_onMemApp()

#endif /* def QF_MEM_ISOLATE */

#include "qep_port.h"  /* QEP port */
#include "qk_port.h"   /* QK port */

#endif /* QF_PORT_H_ */
