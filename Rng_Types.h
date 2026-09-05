/**
 * \author Mr.Nobody
 * \file Rng_Types.h
 * \ingroup Rng
 * \brief Rng module global types definition
 *
 * This file contains the types definitions used across the module and are 
 * available for other modules through Port file.
 *
 */

#ifndef RNG_RNG_TYPES_H
#define RNG_RNG_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define RNG_NULL_PTR                        ( ( void* ) 0u )

/** Length of RNG data FIFO */
#define RNG_FIFO_LEN                        ( 4u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t rng_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t rng_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t rng_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    rng_MajorVersion_t Major; /**< Major version */
    rng_MinorVersion_t Minor; /**< Minor version */
    rng_PatchVersion_t Patch; /**< Patch version */
}   rng_ModuleVersion_t;


/** \brief Function status enumeration */
typedef enum
{
    RNG_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    RNG_FUNCTION_ACTIVE         /**< Function status is active   */
}   rng_FunctionState_t;


/** \brief Flag states enumeration */
typedef enum
{
    RNG_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    RNG_FLAG_ACTIVE         /**< Active flag state   */
}   rng_FlagState_t;


/** \brief Enumeration used to signal request processing state */
typedef enum
{
    RNG_REQUEST_ERROR = 0u, /**< Processing request failed  */
    RNG_REQUEST_OK          /**< Processing request succeed */
}   rng_RequestState_t;


/** \brief Peripheral identification */
typedef enum
{
    RNG_PERIPH_1 = 0u,
    RNG_PERIPH_CNT
}   rng_PeriphId_t;


/** \brief Random Number Generator (RNG) clock source selection */
typedef enum
{
    RNG_CLK_SOURCE_HSI48 = 0u, /**< HSI48 clock source              */
    RNG_CLK_SOURCE_HSI48_DIV2, /**< HSI48 divided by 2 clock source */
    RNG_CLK_SOURCE_HSI16,      /**< HSI16 clock source              */
    RNG_CLK_SOURCE_CNT         /**< Count of available options      */
}   rng_ClkSource_t;


/** \brief Random Number Generator (RNG) clock divider selection.
 *
 * The clock divider is applied to the selected clock source before it is used
 * by the RNG. The clock divider can be used to reduce the clock frequency
 * provided to the RNG. The maximum clock frequency allowed is 48MHz.
 * The minimal frequency is not specified however for NIST compliance it
 * should be at least 1.2 MHz to deliver 16bytes every 341 us.
 */
typedef enum
{
    RNG_CLK_DIV_1,     /**< Input clock is not divided      */
    RNG_CLK_DIV_2,     /**< Input clock is divided by 2     */
    RNG_CLK_DIV_4,     /**< Input clock is divided by 4     */
    RNG_CLK_DIV_8,     /**< Input clock is divided by 8     */
    RNG_CLK_DIV_16,    /**< Input clock is divided by 16    */
    RNG_CLK_DIV_32,    /**< Input clock is divided by 32    */
    RNG_CLK_DIV_64,    /**< Input clock is divided by 64    */
    RNG_CLK_DIV_128,   /**< Input clock is divided by 128   */
    RNG_CLK_DIV_256,   /**< Input clock is divided by 256   */
    RNG_CLK_DIV_512,   /**< Input clock is divided by 512   */
    RNG_CLK_DIV_1024,  /**< Input clock is divided by 1024  */
    RNG_CLK_DIV_2048,  /**< Input clock is divided by 2048  */
    RNG_CLK_DIV_4096,  /**< Input clock is divided by 4096  */
    RNG_CLK_DIV_8192,  /**< Input clock is divided by 8192  */
    RNG_CLK_DIV_16384, /**< Input clock is divided by 16384 */
    RNG_CLK_DIV_32768, /**< Input clock is divided by 32768 */
    RNG_CLK_DIV_CNT    /**< Count of available options      */
}   rng_ClkDivider_t;


/** \brief Random Number Generator (RNG) configuration selection.
 *
 * The RNG can be configured to operate in different modes. The modes
 * differ in the way the conditioning component is configured.
 * The conditioning component in the RNG is a deterministic function that
 * increases the entropy rate of the resulting fixed-length bitstrings output
 * (128-bit).
 * The NIST SP800-90B target is full entropy on the output (128-bit).
 * Other configurations may be used when full entropy is not required or
 * when a higher throughput is needed.
 */
typedef enum
{
    RNG_CONFIG_SELECTION_NIST = 0u,      /**< NIST SP800-90B compliant configuration                 */
    RNG_CONFIG_SELECTION_AIS_31_SPEED,   /**< AIS-31 T0 to T8 compliant configuration - high speed   */
    RNG_CONFIG_SELECTION_AIS_31_ENTROPY, /**< AIS-31 T0 to T8 compliant configuration - high entropy */
    RNG_CONFIG_SELECTION_CNT             /**< Count of available options                             */
}   rng_ConfigSelection_t;


/** \brief List of random number generator peripheral errors */
typedef enum
{
    RNG_ERROR_NONE = 0u, /**< No active error mask          */
    RNG_ERROR_SEED_ERR,  /**< Noise source error            */
    RNG_ERROR_CLOCK_ERR, /**< Clock frequency too low error */
    RNG_ERROR_ALL,       /**< All possible errors           */
    RNG_ERROR_CNT        /**< Count of available options    */
}   rng_Error_t;


/** \brief Data transfer handling style enumeration */
typedef enum
{
    RNG_TRANSFER_NONE = 0u, /**< No data transfer handling style is used. User must handle data transfer by himself. */
    RNG_TRANSFER_BLOCKING , /**< Blocking style is used for data transfer.                                           */
    RNG_TRANSFER_INTERRUPT, /**< Interrupts are used for data transfer.                                              */
    RNG_TRANSFER_CNT        /**< Count of available options                                                          */
}   rng_TransferStyle_t;


/**
 * \brief Random Number Generator (RNG) data output type.
 * The conditioning component in the RNG is a deterministic function that
 * increases the entropy rate of the resulting fixed-length bitstrings output
 * (128-bit).
 * The NIST SP800-90B target is full entropy on the output (128-bit). */
typedef struct
{
    uint32_t value[ RNG_FIFO_LEN ];
}   rng_RandomData_t;


/** \brief Interrupt priority type definition */
typedef uint32_t rng_IrqPrio_t;


/** \brief Data ready callback type definition. */
typedef void ( rng_DataRdyCallback_t )( rng_RandomData_t value );


/** \brief Error callback type definition. */
typedef void ( rng_ErrCallback_t )( rng_Error_t errMask );


typedef struct
{
    rng_PeriphId_t        PeriphId;         /**< Peripheral identification                                  */
    rng_ClkSource_t       ClkSource;        /**< Clock source selection                                     */
    rng_ConfigSelection_t ConfigSelection;  /**< RNG configuration selection                                */

    rng_TransferStyle_t   TransferStyle;    /**< Data transfer style (block/interrupt)                      */
    rng_IrqPrio_t         IrqPrio;          /**< Interrupt priority (in case interrupt transfer is used)    */
    rng_DataRdyCallback_t *DataRdyCallback; /**< Data ready callback (in case interrupt transfer is used)   */
    rng_ErrCallback_t     *ErrorCallback;   /**< Active error callback (in case interrupt transfer is used) */
}   rng_ConfigStruct_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* RNG_RNG_TYPES_H */
