/*! @file
* @brief Various macros for configuring QP/C (typically used as
* command-line options)
*/

/*! Preprocessor switch to disable the QP Functional Safety (FuSa) Subsystem
*
* @details
* When defined, #Q_UNSAFE disables the QP Functional Safety (FuSa) Subsystem.
* This means that there is no CPU overhead for internal integrity checks
* and no memory overhead for duplicate storage, but there is
* **no protection** against Uncontrolled System Failures (USFs) either.
*
* @attention
* The QP Functional Safety (FuSa) Subsystem is enabled by default and
* explicit disabling it is **NOT** recommended, especially in
* safety-related applications.
*/
#define Q_UNSAFE

/*! @deprecated Preprocessor switch to disable QP assertions
*
* @description
* This macro has been superseded by #Q_UNSAFE.
*/
#define Q_NASSERT

/*! The preprocessor switch to activate the QS software tracing
* instrumentation in the code
*
* @details
* When defined, Q_SPY activates the QS software tracing instrumentation.
* When Q_SPY is not defined, the QS instrumentation in the code does
* not generate any code.
*/
#define Q_SPY

/*! The preprocessor switch to activate the QUTest unit testing
* instrumentation in the code
*
* @note
* This macro requires that #Q_SPY be defined as well.
*/
#define Q_UTEST

/*! The preprocessor switch to enable invoking event constructors
* for dynamically created events, see @ref QF::QF_newX_() "QF_newX_()"
*
* @trace
* - @tr{REQ-QP-01_40}
*/
#define Q_EVT_CTOR

/*! This macro enables calling the context-switch callback
* QF_onContextSw() in all build-in kernels (QV, QK, QXK)
*/
#define QF_ON_CONTEXT_SW

/*! Enable the QActive_stop() API in the QF port.
*
* @details
* Defining this macro enables the QActive_stop() API in a given port.
* This feature should be used with caution, as stopping and re-starting
* active objects **cleanly** can be tricky.
*/
#define QF_ACTIVE_STOP

/*! The preprocessor switch to activate the Memory Separation in QP.
*
* @details
* This macro requires hardware support, such as the Memory Protection
* Unit (MPU). Also usually requires the the context-switch callback
* QF_onContextSw(), so #QF_ON_CONTEXT_SW gets defined as well.
*/
#define QF_MEM_ISOLATE

/*! Macro defined only for the internal QP implementation. It should
* be not defined for the application-level code
*/
#define QP_IMPL
