/**
 * \author Mr.Nobody
 * \file Rng.h
 * \ingroup Rng
 * \brief Random Number Generator (RNG) module common functionality
 *
 * # Introduction #
 * The RNG is a true random number generator that provides full entropy outputs
 * to the application as 32-bit samples. It is composed of a live entropy source
 * (analog) and an internal conditioning component.
 * The RNG is a NIST SP 800-90B compliant entropy source that can be used to
 * construct a nondeterministic random bit generator (NDRBG).
 * The RNG true random number generator can be certified NIST SP800-90B. It can
 * also be tested using the German BSI statistical tests of AIS-31 (T0 to T8).
 *
 * # RNG main features #
 * • The RNG delivers 32-bit true random numbers, produced by an analog entropy
 *   source conditioned by a NIST SP800-90B approved conditioning stage.
 * • It can be used as the entropy source to construct a nondeterministic random
 *   bit generator (NDRBG).
 * • In the NIST configuration, it produces four 32-bit random samples every
 *   412 AHB clock cycles if fAHB < fthreshold (256 RNG clock cycles otherwise).
 * • It embeds startup and NIST SP800-90B approved continuous health tests
 *   (repetition count and adaptive proportion tests), associated with specific
 *   error management
 * • It can be disabled to reduce power consumption, or enabled with an automatic
 *   low power mode (default configuration).
 * • It has an AMBA® AHB slave peripheral, accessible through 32-bit word single
 *   accesses only (else an AHB bus error is generated, and the write accesses
 *   are ignored).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Rng.h"                            /* Self include                   */
#include "Rng_Port.h"                       /* Public functions include       */
#include "Rng_Types.h"                      /* Module types definitions       */
#include "Rcc_Port.h"                       /* RCC functionality              */
#include "Nvic_Port.h"                      /* NVIC functionality             */
#include "Stm32.h"                          /* MCU common functionality       */
#include "Stm32_rng.h"                      /* RNG peripheral functionality   */
/* ============================== TYPEDEFS ================================== */

/** Enumeration used to signal execution state of peripheral */
typedef enum
{
    RNG_EXEC_STATE_IDLE = 0u, /**< Peripheral is idle (not processing data)       */
    RNG_EXEC_STATE_BUSY,      /**< Peripheral is busy (processing data)           */
    RNG_EXEC_STATE_PAUSED,    /**< Peripheral is paused (processing is suspended) */
    RNG_EXEC_STATE_ERROR,     /**< Peripheral is in error state                   */
    RNG_EXEC_STATE_CNT
}   rng_ExecState_t;


/** Structure type used for USART/UART configuration array */
typedef struct
{
    RNG_TypeDef*          RngReg;  /**< RNG configuration register */
    rcc_PeriphId_t        RngRcc;  /**< RNG RCC configuration ID   */
    nvic_PeriphIrqList_t  RngNvic; /**< RNG NVIC configuration ID  */
    nvic_IsrCallback_t    RngIsr;  /**< RNG ISR callback routines  */
}   usart_ConfigStruct_t;


/** Structure type used to store users USART/UART ISR callback pointers */
typedef struct
{
    rng_PeriphId_t         RngPeriphId;     /**< Peripheral ID       */
    rng_TransferStyle_t    TransferStyle;   /**< Data transfer style */
    rng_ExecState_t        ExecState;       /**< Execution state     */

    rng_DataRdyCallback_t *DataRdyCallback; /**< Data ready callback */
    rng_ErrCallback_t     *ErrorCallback;   /**< Error callback      */

}   rng_RuntimeData_t;


/** Structure type used to store peripheral configuration values */
typedef struct
{
    rng_ConfigSelection_t ConfigSelection; /**< Configuration selection  */
    uint32_t              CrRegVal;        /**< CR Register value        */
    uint32_t              HtcrRegVal;      /**< HTCR Register value      */
    uint32_t              NscrRegVal;      /**< NSCR Register value      */
}   rng_ConfigurationList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static void Rng_IsrHandler( void );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define RNG_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define RNG_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define RNG_PATCH_VERSION           ( 0u )

/** Maximum wait time for configuration request confirmation */
#define RNG_TIMEOUT_RAW             ( 0x84FCB )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

static volatile rng_RuntimeData_t       rng_RuntimeData[ RNG_PERIPH_CNT ] =
{
    { .RngPeriphId = RNG_PERIPH_1, .TransferStyle = RNG_TRANSFER_BLOCKING, .ExecState = RNG_EXEC_STATE_IDLE, .DataRdyCallback = RNG_NULL_PTR, .ErrorCallback = RNG_NULL_PTR },
};

static usart_ConfigStruct_t const       rng_PeriphConf[ RNG_PERIPH_CNT ] =
{
    { .RngReg = RNG, .RngRcc = RCC_PERIPH_RNG_HSI48, .RngNvic = NVIC_PERIPH_IRQ_RNG, .RngIsr = Rng_IsrHandler },
};


static rng_ConfigurationList_t const    rng_ConfigList[ RNG_CONFIG_SELECTION_CNT ] =
{
    { .ConfigSelection = RNG_CONFIG_SELECTION_NIST          , .CrRegVal = RNG_CR_NIST_VALUE, .HtcrRegVal = RNG_HTCR_NIST_VALUE, .NscrRegVal = RNG_NSCR_NIST_VALUE  },
    { .ConfigSelection = RNG_CONFIG_SELECTION_AIS_31_SPEED  , .CrRegVal = 0x01801000       , .HtcrRegVal = 0x0000AAC7         , .NscrRegVal = 0x0003ffff           },
    { .ConfigSelection = RNG_CONFIG_SELECTION_AIS_31_ENTROPY, .CrRegVal = 0x00F00D00       , .HtcrRegVal = 0x0000AAC7         , .NscrRegVal = 0x0003ffff           },
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
rng_ModuleVersion_t Rng_Get_ModuleVersion( void )
{
    rng_ModuleVersion_t retVersion;

    retVersion.Major = RNG_MAJOR_VERSION;
    retVersion.Minor = RNG_MINOR_VERSION;
    retVersion.Patch = RNG_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initialization of Random Number Generator (RNG) peripheral
 *
 * \note The clock for RNG shall be configured to 48MHz. Otherwise error will
 *       be raised.
 *
 * \param rngConfig [in]: Peripheral configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Init( rng_ConfigStruct_t * const rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_NULL_PTR != rngConfig )
    {
        retState = Rng_Set_ClkSource( rngConfig->PeriphId, rngConfig->ClkSource );

        if( RNG_REQUEST_ERROR != retState )
        {
            retState = Rng_Set_Configuration( rngConfig->PeriphId, rngConfig->ConfigSelection );
        }

        /*----------------------- Configure peripheral -----------------------*/

        if( RNG_REQUEST_ERROR != retState )
        {
            retState = Rng_Set_DataRdyCallback( rngConfig->PeriphId, rngConfig->DataRdyCallback );
        }

        if( RNG_REQUEST_ERROR != retState )
        {
            retState = Rng_Set_ErrorCallback( rngConfig->PeriphId, rngConfig->ErrorCallback );
        }

        /*------------------- Transfer mode configuration --------------------*/

        if( RNG_TRANSFER_INTERRUPT == rngConfig->TransferStyle )
        {
            if( RNG_REQUEST_ERROR != retState )
            {
                retState = Rng_Set_IrqPriority( rngConfig->PeriphId, rngConfig->IrqPrio );
            }

            if( RNG_REQUEST_ERROR != retState )
            {
                retState = Rng_Set_IrqActive( rngConfig->PeriphId );
            }
        }
        else if( RNG_TRANSFER_BLOCKING == rngConfig->TransferStyle )
        {
            retState = RNG_REQUEST_OK;
        }
        else
        {
            /* None of the supported transfer styles was selected.
             * User will handle it by himself. */
        }
    }
    else
    {
        /* Null pointer assigned */
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes module Rng
 *
 * This function shall call every necessary sub-module deinitialization function 
 * and free all the resources allocated by the module. In case of failure, the 
 * function shall handle it by itself and shall not be transferred to AppMain 
 * layer.
 */
rng_RequestState_t Rng_Deinit( rng_ConfigStruct_t * const rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( rng_PeriphConf[ rngConfig->PeriphId ].RngRcc );

    if( RCC_REQUEST_ERROR == rccState )
    {
        /* Clock was not successfully deactivated */
        retState = RNG_REQUEST_ERROR;
    }
    else
    {
        retState = RNG_REQUEST_OK;
    }

    return (retState);
}


/**
 * \brief Main task of module Rng
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's 
 * requirements.
 */
void Rng_Task( void )
{
    rng_Error_t errorState = RNG_ERROR_NONE;

    for( uint32_t periphId = 0u; RNG_PERIPH_CNT > periphId; periphId++ )
    {
        if( ( RNG_TRANSFER_BLOCKING == rng_RuntimeData[ periphId ].TransferStyle ) &&
            ( RNG_EXEC_STATE_BUSY   == rng_RuntimeData[ periphId ].ExecState     )    )
        {
            /* In blocking mode, check if data is ready */
            rng_FunctionState_t dataReadyState = RNG_FUNCTION_INACTIVE;

            Rng_Get_DataReadyState( (rng_PeriphId_t) periphId, &dataReadyState );

            if( RNG_FUNCTION_ACTIVE == dataReadyState )
            {
                rng_RandomData_t generatedData;

                (void)Rng_Get_Data( (rng_PeriphId_t) periphId, &generatedData );

                if ( RNG_NULL_PTR != rng_RuntimeData[ periphId ].DataRdyCallback )
                {
                    rng_RuntimeData[ periphId ].DataRdyCallback( generatedData );
                }
                else
                {
                    /* No callback assigned, data will be lost */
                }
            }
            else
            {
                /* No data ready, nothing to do. */
            }

            (void)Rng_Get_Error( (rng_PeriphId_t) periphId, &errorState );

            if( RNG_ERROR_NONE != errorState )
            {
                if ( RNG_NULL_PTR != rng_RuntimeData[ periphId ].ErrorCallback )
                {
                    rng_RuntimeData[ periphId ].ErrorCallback( errorState );
                }
            }
            else
            {
                /* No error detected, nothing to do. */
            }
        }
        else
        {
            /* In interrupt mode, nothing to do here */
        }
    }
}


/**
 * \brief Loads default values for Random Number Generator (RNG) peripheral
 *
 * \param rngConfig [out]: Pointer to configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_DefaultConfig( rng_ConfigStruct_t *rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_NULL_PTR != rngConfig )
    {
        nvic_IrqPrio_t      irqPrio   = 0u;
        nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

        nvicState = Nvic_Get_PeriphIrq_Prio( rng_PeriphConf[ RNG_PERIPH_1 ].RngNvic, &irqPrio );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState  = RNG_REQUEST_ERROR;
        }
        else
        {
            retState  = RNG_REQUEST_OK;
        }

        rngConfig->PeriphId        = RNG_PERIPH_1;
        rngConfig->TransferStyle   = RNG_TRANSFER_BLOCKING;
        rngConfig->IrqPrio         = irqPrio;
        rngConfig->DataRdyCallback = RNG_NULL_PTR;
        rngConfig->ErrorCallback   = RNG_NULL_PTR;
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Activates Random Number Generator (RNG) peripheral.
 *
 * After activation of RNG peripheral, this will start with generating data values.
 *
 * \param rngId [in]: Peripheral identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_Enable( rng_PeriphConf[ rngId ].RngReg );

        for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regValue  = LL_RNG_IsEnabled( rng_PeriphConf[ rngId ].RngReg );

            if( 0u != regValue )
            {
                rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_BUSY;

                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Random Number Generator (RNG) peripheral.
 *
 * \param rngId [in]: Peripheral identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( rng_PeriphConf[ rngId ].RngRcc );
        if( RCC_REQUEST_ERROR != rccState )
        {
            /* Clock was not successfully activated */
            retValue = RNG_REQUEST_ERROR;
        }
        else
        {
            LL_RNG_Disable( rng_PeriphConf[ rngId ].RngReg );

            for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                uint32_t regValue  = LL_RNG_IsEnabled( rng_PeriphConf[ rngId ].RngReg );

                if( 0u == regValue )
                {
                    rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_IDLE;

                    retValue = RNG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retValue = RNG_REQUEST_ERROR;
                }
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns current state of peripheral data processing.
 *
 * This function returns current state of peripheral data processing.
 *
 * \param rngId  [in]: Peripheral identification
 * \param state [out]: Pointer to variable where current state will be stored
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ExecState( rng_PeriphId_t rngId, rng_FunctionState_t * const state )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId ) &&
        ( RNG_NULL_PTR  != state )    )
    {
        if( RNG_EXEC_STATE_BUSY == rng_RuntimeData[ rngId ].ExecState )
        {
            *state = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *state = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Pauses Random Number Generator (RNG) peripheral execution.
 *
 * In blocking mode, the processing is done in task function.
 *
 * \param rngId [in]: Peripheral identification
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecPauseActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        if( RNG_EXEC_STATE_BUSY == rng_RuntimeData[rngId].ExecState )
        {
            /* In blocking mode, the processing is done in task function. */
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_PAUSED;

            retValue = RNG_REQUEST_OK;
        }
        else
        {
            retValue = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Resumes Random Number Generator (RNG) peripheral execution.
 *
 * In blocking mode, the processing is done in task function.
 *
 * \param rngId [in]: Peripheral identification
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecPauseInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        if( RNG_EXEC_STATE_PAUSED == rng_RuntimeData[ rngId ].ExecState )
        {
            /* In blocking mode, the processing is done in task function. */
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_BUSY;

            retValue = RNG_REQUEST_OK;
        }
        else
        {
            retValue = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns current pause state of peripheral data processing.
 *
 * This function returns current pause state of peripheral data processing.
 *
 * \param rngId  [in]: Peripheral identification
 * \param state [out]: Pointer to variable where current pause state will be stored
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ExecPauseState( rng_PeriphId_t rngId, rng_FunctionState_t * const state )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId ) &&
        ( RNG_NULL_PTR  != state )    )
    {
        if( RNG_EXEC_STATE_PAUSED == rng_RuntimeData[rngId].ExecState )
        {
            *state = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *state = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return (retValue);
}


/**
 * \brief Returns actual data from output register
 *
 * \return Value of generated data
 */
rng_RequestState_t Rng_Get_Data( rng_PeriphId_t rngId, rng_RandomData_t * const data )
{
    rng_FunctionState_t dataReadyState = RNG_FUNCTION_INACTIVE;
    rng_RequestState_t  retValue       = RNG_REQUEST_ERROR;

    retValue = Rng_Get_DataReadyState( rngId, &dataReadyState );

    if( ( RNG_FUNCTION_ACTIVE == dataReadyState ) &&
        ( RNG_REQUEST_OK      == retValue       ) &&
        ( RNG_NULL_PTR        != data           )    )
    {
        for( uint32_t index = 0u; RNG_FIFO_LEN > index; index++ )
        {
            data->value[ index ] = LL_RNG_ReadRandData32( rng_PeriphConf[ rngId ].RngReg );
        }
    }
    else
    {
        /* No data ready or incorrect peripheral ID requested. */
    }

    return ( retValue );
}


/*-------------------------- Primitive functionality -------------------------*/

/**
 * \brief Sets configuration of Random Number Generator (RNG) peripheral.
 *
 * The Random Number Generator (RNG) offers three configurations for the
 * conditioning component.
 * Each configuration is compliant with different standards and provides
 * different throughput and entropy rate on the output.
 * The configurations are:
 *
 * - NIST SP800-90B compliant configuration:
 *   The conditioning component is based on a cryptographic hash function
 *   (SHA-256) and provides full entropy on the output (128-bit).
 *
 * - AIS-31 T0 to T8 compliant configuration - high speed
 *   The conditioning component is based on a linear feedback shift register
 *   (LFSR) and provides lower entropy on the output (128-bit).
 *
 * - AIS-31 T0 to T8 compliant configuration - high entropy
 *   The conditioning component is based on a cryptographic hash function
 *   (SHA-256) and provides full entropy on the output (128-bit).
 *
 * \param rngId           [in]: RNG peripheral ID
 * \param configSelection [in]: Configuration selection
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_Configuration( rng_PeriphId_t rngId, rng_ConfigSelection_t configSelection )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT           > rngId           ) &&
        ( RNG_CONFIG_SELECTION_CNT > configSelection )    )
    {
        MODIFY_REG( rng_PeriphConf[ rngId ].RngReg->CR,
                    RNG_CR_CONDRST | RNG_CR_RNG_CONFIG1 | RNG_CR_CLKDIV | RNG_CR_RNG_CONFIG2 |
                    RNG_CR_NISTC   | RNG_CR_RNG_CONFIG3 | RNG_CR_CED ,
                    rng_ConfigList[ configSelection ].CrRegVal | RNG_CR_CONDRST);

        CLEAR_BIT ( rng_PeriphConf[ rngId ].RngReg->CR, RNG_CR_CONDRST );

        LL_RNG_SetNoiseConfig ( rng_PeriphConf[rngId].RngReg, rng_ConfigList[ configSelection ].NscrRegVal );
        LL_RNG_SetHealthConfig( rng_PeriphConf[rngId].RngReg, rng_ConfigList[ configSelection ].HtcrRegVal );
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns configuration of Random Number Generator (RNG) peripheral.
 *
 * The Random Number Generator (RNG) offers three configurations for the
 * conditioning component.
 * Each configuration is compliant with different standards and provides
 * different throughput and entropy rate on the output.
 * The configurations are:
 *
 * - NIST SP800-90B compliant configuration:
 *   The conditioning component is based on a cryptographic hash function
 *   (SHA-256) and provides full entropy on the output (128-bit).
 *
 * - AIS-31 T0 to T8 compliant configuration - high speed
 *   The conditioning component is based on a linear feedback shift register
 *   (LFSR) and provides lower entropy on the output (128-bit).
 *
 * - AIS-31 T0 to T8 compliant configuration - high entropy
 *   The conditioning component is based on a cryptographic hash function
 *   (SHA-256) and provides full entropy on the output (128-bit).
 *
 * \param rngId           [in] : RNG peripheral ID
 * \param configSelection [out]: Pointer where configuration selection will be stored
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_Configuration( rng_PeriphId_t rngId, rng_ConfigSelection_t * const configSelection )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId           ) &&
        ( RNG_NULL_PTR  != configSelection )    )
    {
        uint32_t crRegVal = READ_REG( rng_PeriphConf[ rngId ].RngReg->CR );
        crRegVal = crRegVal & ( RNG_CR_RNG_CONFIG1 | RNG_CR_CLKDIV      | RNG_CR_RNG_CONFIG2 |
                                RNG_CR_NISTC       | RNG_CR_RNG_CONFIG3 | RNG_CR_CED );

        uint32_t nscrRegVal = LL_RNG_GetNoiseConfig ( rng_PeriphConf[rngId].RngReg );
        uint32_t htcrRegVal = LL_RNG_GetHealthConfig( rng_PeriphConf[rngId].RngReg );

        for( uint32_t index = 0u; RNG_CONFIG_SELECTION_CNT > index; index++ )
        {
            if( ( crRegVal   == rng_ConfigList[ index ].CrRegVal   ) &&
                ( nscrRegVal == rng_ConfigList[ index ].NscrRegVal ) &&
                ( htcrRegVal == rng_ConfigList[ index ].HtcrRegVal )    )
            {
                *configSelection = rng_ConfigList[ index ].ConfigSelection;

                retValue = RNG_REQUEST_OK;

                break;
            }
            else
            {
                /* Keep return value as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns state of data ready flag.
 *
 * This flag is set by hardware when a new random number is ready to be read
 * from the output register. It is cleared by software reading the output
 * register.
 *
 * \param rngId [in] : RNG peripheral ID
 * \param state [out]: Pointer to state of data ready flag
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_DataReadyState( rng_PeriphId_t rngId, rng_FunctionState_t * const state )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId ) &&
        ( RNG_NULL_PTR  != state )    )
    {
        uint32_t regValue = LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ rngId ].RngReg );

        if( 0u != regValue )
        {
            *state = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *state = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns error mask of Random Number Generator (RNG) peripheral.
 *
 * \param rngId   [in] : RNG peripheral ID
 * \param errMask [out]: Pointer to error mask
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_Error( rng_PeriphId_t rngId, rng_Error_t * const errMask )
{
    rng_RequestState_t retValue     = RNG_REQUEST_ERROR;
    rng_Error_t        localErrMask = RNG_ERROR_NONE;

    if( ( RNG_PERIPH_CNT > rngId  ) &&
        ( RNG_NULL_PTR != errMask )    )
    {
        uint32_t errFlagCeis = LL_RNG_IsActiveFlag_CEIS( rng_PeriphConf[ rngId ].RngReg );
        uint32_t errFlagSeis = LL_RNG_IsActiveFlag_SEIS( rng_PeriphConf[ rngId ].RngReg );

        if( ( 0u != errFlagCeis ) &&
            ( 0u != errFlagSeis )    )
        {
            localErrMask = RNG_ERROR_ALL;
        }
        else if( 0u != errFlagCeis )
        {
            localErrMask = RNG_ERROR_CLOCK_ERR;
        }
        else if( 0u != errFlagSeis )
        {
            localErrMask = RNG_ERROR_SEED_ERR;
        }
        else
        {
            localErrMask = RNG_ERROR_NONE;
        }

        *errMask = localErrMask;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Clears error flags of Random Number Generator (RNG) peripheral.
 *
 * \param rngId   [in]: RNG peripheral ID
 * \param errMask [in]: Error mask to be cleared
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Clear_Error( rng_PeriphId_t rngId, rng_Error_t errMask )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        if( ( RNG_ERROR_CLOCK_ERR == errMask ) ||
            ( RNG_ERROR_ALL       == errMask )    )
        {
            LL_RNG_ClearFlag_CEIS( rng_PeriphConf[ rngId ].RngReg );
        }

        if( ( RNG_ERROR_SEED_ERR == errMask ) ||
            ( RNG_ERROR_ALL      == errMask )    )
        {
            LL_RNG_ClearFlag_SEIS( rng_PeriphConf[ rngId ].RngReg );
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates clock error detection functionality.
 *
 * When clock error detection is enabled, the RNG checks that the clock
 * frequency is within the allowed range. If a clock error is detected,
 * the CEIS bit in the SR register is set and an interrupt is generated if
 * the CEIE bit in the CR register is set.
 *
 * \note When the clock error detection is activated, the RNG clock frequency
 *       before the internal divider must be higher than the AHB clock frequency
 *       divided by 32, otherwise the clock checker always flags a clock error.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClockErrorDetectActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_EnableClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

        for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
        {
            uint32_t regValue = LL_RNG_IsEnabledClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

            if (0u != regValue)
            {
                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates clock error detection functionality.
 *
 * When clock error detection is disabled, the RNG does not check that the
 * clock frequency is within the allowed range.
 *
 * \param rngId [in]: RNG peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClockErrorDetectInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_DisableClkErrorDetect( rng_PeriphConf[rngId].RngReg );

        for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
        {
            uint32_t regValue = LL_RNG_IsEnabledClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

            if( 0u == regValue )
            {
                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns state of clock error detection functionality.
 *
 * \param rngId [in] : RNG peripheral ID
 * \param state [out]: Pointer to state of clock error detection functionality
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ClockErrorDetectState( rng_PeriphId_t rngId, rng_FunctionState_t * const state )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId ) &&
        ( RNG_NULL_PTR  != state )    )
    {
        uint32_t regValue = LL_RNG_IsEnabledClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

        if( 0u != regValue )
        {
            *state = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *state = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates auto reset functionality.
 *
 * \note Keeping the auto-reset enabled (automatic clearance of the SECS bit)
 *       simplifies the management of noise source errors.
 *
 * When a noise source (or seed) error occurs, the RNG stops generating random
 * numbers and sets to 1 both SEIS and SECS bits to indicate that a seed error
 * occurred. If a value is available in the RNG_DR register, it must not be used
 * as it may not have enough entropy.
 * When auto-reset is enabled, it clear the SEIS interrupt status bit in the
 * RNG_SR register before drawing random numbers again.
 * When ARDIS is set (auto-reset disabled) the following sequence must be used
 * to fully recover from a seed error:
 * 1. Write CONDRST at 1 and at 0 in the RNG_CR register to reset the
 *    conditioning logic.
 * 2. Wait for the CONDRST bit to be cleared by hardware. Then clear the SEIS
 *    interrupt status bit in the RNG_SR register.
 * 3. If SECS is still set in RNG_SR it means a new seed error occurred
 *    (unlikely). In this case go back to step 1.
 *
 * \note After a seed error RNG restarts generating random numbers when SECS is
 *       cleared.
 *
 * \note Activation this function is ignored if configuration lock is active.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_AutoResetActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_EnableArdis( rng_PeriphConf[ rngId ].RngReg );

        for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
        {
            uint32_t regValue = LL_RNG_IsEnabledArdis( rng_PeriphConf[ rngId ].RngReg );

            if( 0u != regValue )
            {
                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates seed error detection functionality.
 *
 * When seed error detection is disabled, the RNG does not check that
 * the seed value is not all zeros.
 *
 * \param rngId [in]: RNG peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_AutoResetInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_DisableArdis( rng_PeriphConf[ rngId ].RngReg );

        for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
        {
            uint32_t regValue = LL_RNG_IsEnabledArdis( rng_PeriphConf[ rngId ].RngReg );

            if( 0u == regValue )
            {
                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = RNG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/** * \brief Returns state of seed error detection functionality.
 *
 * \param rngId [in] : RNG peripheral ID
 * \param reqState [out]: Pointer to state of seed error detection functionality
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_AutoResetState( rng_PeriphId_t rngId, rng_FunctionState_t * const reqState )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId    ) &&
        ( RNG_NULL_PTR  != reqState )    )
    {
        uint32_t regValue = LL_RNG_IsEnabledArdis( rng_PeriphConf[rngId].RngReg );

        if( 0u != regValue )
        {
            *reqState = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *reqState = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets clock source for Random Number Generator (RNG) peripheral.
 *
 * \warning The clock source shall be 48MHz for NIST SP800-90B compliant.
 *
 * \param rngId     [in]: RNG peripheral ID
 * \param clkSource [in]: Clock source
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClkSource( rng_PeriphId_t rngId, rng_ClkSource_t clkSource )
{
    rng_RequestState_t retValue  = RNG_REQUEST_ERROR;
    rcc_RequestState_t rccState  = RCC_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT     > rngId     ) &&
        ( RNG_CLK_SOURCE_CNT > clkSource )    )
    {
        if( RNG_CLK_SOURCE_HSI48 == clkSource )
        {
            rccState = Rcc_Set_PeriphActive( RCC_PERIPH_RNG_HSI48 );
        }
        else if( RNG_CLK_SOURCE_HSI48_DIV2 == clkSource )
        {
            rccState = Rcc_Set_PeriphActive( RCC_PERIPH_RNG_HSI48_DIV2 );
        }
        else if( RNG_CLK_SOURCE_HSI16 == clkSource )
        {
            rccState = Rcc_Set_PeriphActive( RCC_PERIPH_RNG_HSI16 );
        }
        else
        {
            /* None of the supported clock sources was selected.
             * User will handle it by himself. */
            retValue = RNG_REQUEST_ERROR;
        }

        if( RCC_REQUEST_ERROR == rccState )
        {
            /* Clock was not successfully activated */
            retValue = RNG_REQUEST_ERROR;
        }
        else
        {
            retValue = RNG_REQUEST_OK;
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures clock divider for Random Number Generator (RNG) peripheral.
 *
 * \warning The clock divider shall not be changed for NIST SP800-90B compliant.
 *
 * \note The RNG clock frequency is divided by a programmable factor
 *      (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384 or
 *      32768) before being used by the RNG.
 *
 * \param rngId      [in]: RNG peripheral ID
 * \param clkDivider [in]: Clock divider value
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClkDivider( rng_PeriphId_t rngId, rng_ClkDivider_t clkDivider )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;
    uint32_t           regValue = 0u;

    if( ( RNG_PERIPH_CNT  > rngId      ) &&
        ( RNG_CLK_DIV_CNT > clkDivider )    )
    {
        regValue = ( ( clkDivider << RNG_CR_CLKDIV_Pos ) & RNG_CR_CLKDIV_Msk );

        LL_RNG_SetClockDivider( rng_PeriphConf[ rngId ].RngReg, regValue );

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns value of clock divider for Random Number Generator (RNG) peripheral.
 *
 * \param rngId      [in] : RNG peripheral ID
 * \param clkDivider [out]: Pointer to clock divider value
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ClkDivider( rng_PeriphId_t rngId, rng_ClkDivider_t * const clkDivider )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;
    uint32_t           regValue = 0u;

    if( ( RNG_PERIPH_CNT > rngId      ) &&
        ( RNG_NULL_PTR  != clkDivider )    )
    {
        regValue = LL_RNG_GetClockDivider( rng_PeriphConf[ rngId ].RngReg );

        *clkDivider = ( regValue >> RNG_CR_CLKDIV_Pos ) & RNG_CR_CLKDIV_Msk;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}

/*---------------------- Transfer handling configuration ---------------------*/

/**
 * \brief Configures data ready callback.
 *
 * This callback is called when new random data is ready to be read from
 * output register. The callback function is common for blocking and
 * interrupt transfer styles.
 * The generated random data is passed as parameter to the callback function.
 *
 * \param rngId    [in]: RNG peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_DataRdyCallback( rng_PeriphId_t rngId, rng_DataRdyCallback_t * const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        rng_RuntimeData[ rngId ].DataRdyCallback = callback;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns data ready callback.
 *
 * This callback is called when new random data is ready to be read from
 * output register. The callback function is common for blocking and
 * interrupt transfer styles.
 * The generated random data is passed as parameter to the callback function.
 *
 * \param rngId    [in] : RNG peripheral ID
 * \param callback [out]: Pointer to interrupt callback
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_DataRdyCallback( rng_PeriphId_t rngId, rng_DataRdyCallback_t ** const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        *callback = rng_RuntimeData[ rngId ].DataRdyCallback;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures error callback
 *
 * This callback is called when error is detected. The callback function
 * is common for blocking and interrupt transfer styles.
 * The error identification is passed as parameter to the callback function.
 *
 * \param rngId    [in]: RNG peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ErrorCallback( rng_PeriphId_t rngId, rng_ErrCallback_t * const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        rng_RuntimeData[ rngId ].ErrorCallback = callback;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns error callback
 *
 * This callback is called when error is detected. The callback function
 * is common for blocking and interrupt transfer styles.
 * The error identification is passed as parameter to the callback function.
 *
 * \param rngId    [in] : RNG peripheral ID
 * \param callback [out]: Pointer to interrupt callback
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ErrorCallback( rng_PeriphId_t rngId, rng_ErrCallback_t ** const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        *callback = rng_RuntimeData[ rngId ].ErrorCallback;

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/*-------------------- Interrupts handling functionality ---------------------*/

/**
 * \brief Activates global Interrupt Requests (IRQ) for RNG peripheral
 *        and configures Interrupt Service Routine (ISR).
 *
 * Activation of global interrupt request is necessary for active any of
 * peripheral interrupt triggers (like DRDY, SECS, CEIS ).
 *
 * \param rngId [in]: RNG peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t  retState               = RNG_REQUEST_ERROR;
    nvic_RequestState_t nvicActivationState    = NVIC_REQUEST_ERROR;
    nvic_RequestState_t nvicHandlerConfigState = NVIC_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        nvicHandlerConfigState = Nvic_Set_PeriphIrq_Handler( rng_PeriphConf[ rngId ].RngNvic,
                                                             rng_PeriphConf[ rngId ].RngIsr );

        nvicActivationState = Nvic_Set_PeriphIrq_Active( rng_PeriphConf[ rngId ].RngNvic );

        if( ( NVIC_REQUEST_OK != nvicActivationState    ) ||
            ( NVIC_REQUEST_OK != nvicHandlerConfigState )    )
        {
            retState  = RNG_REQUEST_ERROR;
        }
        else
        {
            LL_RNG_EnableIT( rng_PeriphConf[ rngId ].RngReg );

            for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                uint32_t regValue  = LL_RNG_IsEnabledIT( rng_PeriphConf[ rngId ].RngReg );

                if( 0u != regValue )
                {
                    retState = RNG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retState = RNG_REQUEST_ERROR;
                }
            }
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief De-activates global Interrupt Requests (IRQ) for RNG peripheral
 *        and configures Interrupt Service Routine (ISR).
 *
 * \param rngId [in]: RNG peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t  retState  = RNG_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        nvicState = Nvic_Set_PeriphIrq_Inactive( rng_PeriphConf[ rngId ].RngNvic );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState  = RNG_REQUEST_ERROR;
        }
        else
        {
            LL_RNG_DisableIT( rng_PeriphConf[ rngId ].RngReg );

            for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                uint32_t regValue  = LL_RNG_IsEnabledIT( rng_PeriphConf[ rngId ].RngReg );

                if( 0u == regValue )
                {
                    retState = RNG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retState = RNG_REQUEST_ERROR;
                }
            }
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns activation state of peripheral interrupt request.
 *
 * \note  When set, Interrupt Enable Bit is enabling interrupt generation
 *        in case of a error or data ready flag.
 *
 * \param rngId     [in]: RNG peripheral ID
 * \param reqState [out]: Activation state of peripheral interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_IrqState( rng_PeriphId_t rngId, rng_FunctionState_t * const reqState )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT       > rngId  ) &&
        ( RNG_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_RNG_IsEnabledIT( rng_PeriphConf[ rngId ].RngReg );

        if( 0u != regValue )
        {
            *reqState = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *reqState = RNG_FUNCTION_INACTIVE;
        }

        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Interrupt Requests (IRQ) configured priority.
 *
 * \param rngId   [in]: RNG peripheral ID
 * \param irqPrio [in]: Configured priority
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqPriority( rng_PeriphId_t rngId, rng_IrqPrio_t irqPrio )
{
    rng_RequestState_t retState  = RNG_REQUEST_ERROR;
    nvic_RequestState_t    nvicState = NVIC_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( rng_PeriphConf[ rngId ].RngNvic, irqPrio );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState  = RNG_REQUEST_ERROR;
        }
        else
        {
            retState  = RNG_REQUEST_OK;
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads Interrupt Requests (IRQ) configured priority.
 *
 * \param rngId    [in]: RNG peripheral ID
 * \param irqPrio [out]: Configured priority
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_IrqPriority( rng_PeriphId_t rngId, rng_IrqPrio_t * const irqPrio )
{
    rng_RequestState_t  retState  = RNG_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT  > rngId   ) &&
        ( RNG_NULL_PTR   != irqPrio )    )
    {
        nvicState = Nvic_Get_PeriphIrq_Prio( rng_PeriphConf[ rngId ].RngNvic, irqPrio );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState  = RNG_REQUEST_ERROR;
        }
        else
        {
            retState  = RNG_REQUEST_OK;
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */


/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief RNG Interrupt handler
 */
static void Rng_IsrHandler( void )
{
    /*----------------------------- Error interrupt --------------------------*/
    if( ( 0u != LL_RNG_IsActiveFlag_SEIS( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg )  ) ||
        ( 0u != LL_RNG_IsActiveFlag_CEIS( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg )  )    )
    {
        rng_Error_t errorBitMask = READ_BIT( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg->SR, RNG_ERROR_ALL );

        LL_RNG_ClearFlag_CEIS( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg );
        LL_RNG_ClearFlag_SEIS( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg );

        if( RNG_NULL_PTR != rng_RuntimeData[ RNG_PERIPH_1 ].ErrorCallback )
        {
            rng_RuntimeData[ RNG_PERIPH_1 ].ErrorCallback( errorBitMask );
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*-------------------- Data ready (DRDY) interrupt -----------------------*/
    if( 0u != LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ RNG_PERIPH_1 ].RngReg ) )
    {
        rng_RandomData_t randData;

        (void)Rng_Get_Data( RNG_PERIPH_1, &randData );

        if( RNG_NULL_PTR != rng_RuntimeData[ RNG_PERIPH_1 ].DataRdyCallback )
        {
            rng_RuntimeData[ RNG_PERIPH_1 ].DataRdyCallback( randData );
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }
}


/* ================================ TASKS =================================== */

