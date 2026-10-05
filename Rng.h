/**
 * \author Mr.Nobody
 * \file Rng.h
 * \ingroup Rng
 * \brief Rng module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef RNG_RNG_H
#define RNG_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Rng_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief Execution state of the peripheral data processing */
typedef enum
{
    RNG_EXEC_STATE_IDLE = 0u, /**< Peripheral is idle (generator disabled)        */
    RNG_EXEC_STATE_BUSY,      /**< Peripheral is busy (generating data)           */
    RNG_EXEC_STATE_PAUSED,    /**< Peripheral is paused (data are not delivered)  */
    RNG_EXEC_STATE_CNT        /**< Count of available execution states            */
}   rng_ExecState_t;


/** \brief Conditioning configuration item */
typedef struct
{
    rng_FunctionState_t Supported;  /**< Configuration is supported by the device       */
    uint32_t            CrRegVal;   /**< CR register value (RNG_CONFIG1/2/3, NISTC)     */
    uint32_t            HtcrRegVal; /**< HTCR register value (health test configuration) */
    uint32_t            NscrRegVal; /**< NSCR register value (noise source control)     */
}   rng_ConfigurationList_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* RNG_RNG_H */
