/**
 * \author Mr.Nobody
 * \file Rng.c
 * \ingroup Rng
 * \brief Random Number Generator (RNG) module common functionality
 *
 * # Introduction #
 * The RNG is a true random number generator that provides full entropy outputs
 * to the application as 32-bit samples. It is composed of a live entropy source
 * (analog) and an internal conditioning component.
 * The RNG is a NIST SP 800-90B compliant entropy source that can be used to
 * construct a nondeterministic random bit generator (NDRBG).
 *
 * # RNG main features #
 * - 32-bit true random numbers, delivered in 128-bit blocks (4 word FIFO).
 * - Startup and NIST SP800-90B approved continuous health tests (repetition
 *   count and adaptive proportion tests), associated error management.
 * - Clock error detection (RNG clock too low compared to AHB clock).
 * - Configuration lock (CONFIGLOCK) - once set, the configuration can be
 *   changed only after system reset.
 *
 * # Configuration changes #
 * Every change of the conditioning configuration (RNG_CONFIG1/2/3, NISTC, CLKDIV,
 * CED, ARDIS, NSCR, HTCR) is done with conditioning soft reset (CONDRST) set,
 * the module waits until the hardware finishes the reset (CONDRST reads 0) and
 * verifies the written value by read-back.
 *
 * \note  Interrupt style: a new block is ready shortly after the previous one
 *        is read, the data ready interrupt therefore follows immediately after
 *        every delivered block and the ISR occupies the CPU. The application
 *        shall pause the delivery (\ref Rng_Set_ExecPauseActive, allowed also
 *        from the data ready callback) when it has enough data.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Rng.h"                            /* Self include                   */
#include "Rng_Port.h"                       /* Own port file include          */
#include "Rng_Types.h"                      /* Module types definitions       */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Stm32_rng.h"                      /* RNG peripheral functionality   */
/* ============================== TYPEDEFS ================================== */

/** \brief Peripheral configuration item */
typedef struct
{
    RNG_TypeDef          *RngReg;  /**< RNG peripheral registers */
    nvic_PeriphIrqList_t  RngNvic; /**< RNG NVIC configuration ID */
    nvic_IsrCallback_t    RngIsr;  /**< RNG ISR routine          */
}   rng_PeriphConfig_t;


/** \brief Runtime data of the peripheral */
typedef struct
{
    rng_TransferStyle_t    TransferStyle;   /**< Data transfer style                     */
    rng_ExecState_t        ExecState;       /**< Execution state                         */
    rcc_PeriphId_t         RngRcc;          /**< RCC ID of the selected kernel clock     */
    rng_DataRdyCallback_t *DataRdyCallback; /**< Data ready callback                     */
    rng_ErrCallback_t     *ErrorCallback;   /**< Error callback                          */
}   rng_RuntimeData_t;


/** \brief LL function returning register (bit field) value of the peripheral */
typedef uint32_t ( *rng_LlGetter_t )( const RNG_TypeDef *rngReg );

/* ======================== FORWARD DECLARATIONS ============================ */

static rng_RequestState_t   Rng_Wait_RegState       ( rng_PeriphId_t rngId, rng_LlGetter_t llGetter, uint32_t reqValue );
static rng_RequestState_t   Rng_Set_CondReset       ( rng_PeriphId_t rngId );

static void                 Rng_IsrHandler          ( void );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define RNG_MAJOR_VERSION               ( 1u )

/** Value of minor version of SW module */
#define RNG_MINOR_VERSION               ( 0u )

/** Value of patch version of SW module */
#define RNG_PATCH_VERSION               ( 0u )

/** Maximal wait time for configuration request confirmation */
#define RNG_TIMEOUT_RAW                 ( 0x84FCB )

/** LL flag / state function return value signaling active state */
#define RNG_LL_STATE_ACTIVE             ( 1u )

/** LL flag / state function return value signaling inactive state */
#define RNG_LL_STATE_INACTIVE           ( 0u )

/** Default interrupt priority used by \ref Rng_Get_DefaultConfig */
#define RNG_DEFAULT_IRQ_PRIO            ( 0u )

/** CR register bits selecting the conditioning configuration */
#define RNG_CR_CONFIG_MASK              ( RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_NISTC )

#if defined(RNG_NSCR_EN_OSC6)
/** NSCR register reset value - all noise sources enabled (RNG with six oscillators) */
#define RNG_NSCR_RESET_VALUE            ( RNG_NSCR_EN_OSC1 | RNG_NSCR_EN_OSC2 | RNG_NSCR_EN_OSC3 | \
                                          RNG_NSCR_EN_OSC4 | RNG_NSCR_EN_OSC5 | RNG_NSCR_EN_OSC6 )
#else
/** NSCR register reset value - all noise sources enabled (RNG with three oscillators - STM32H5E4, STM32H5E5, STM32H5F4, STM32H5F5) */
#define RNG_NSCR_RESET_VALUE            ( RNG_NSCR_EN_OSC1 | RNG_NSCR_EN_OSC2 | RNG_NSCR_EN_OSC3 )
#endif /* RNG_NSCR_EN_OSC6 */

#if defined(RNG_NSCR_NIST_VALUE)
/** NSCR register value of NIST configuration (device specific, AN4230) */
#define RNG_NSCR_NIST_CONFIG            ( RNG_NSCR_NIST_VALUE )
#else
/** NSCR register value of NIST configuration - device has no specific value, reset value is used */
#define RNG_NSCR_NIST_CONFIG            ( RNG_NSCR_RESET_VALUE )
#endif /* RNG_NSCR_NIST_VALUE */

/** CR register value of AIS-31 high speed configuration (STM32U5 RNG version) */
#define RNG_CR_AIS31_SPEED_VALUE        ( 0x01801000u )

/** CR register value of AIS-31 high entropy configuration (STM32U5 RNG version) */
#define RNG_CR_AIS31_ENTROPY_VALUE      ( 0x00F00D00u )

/** HTCR register value of AIS-31 configurations (STM32U5 RNG version) */
#define RNG_HTCR_AIS31_VALUE            ( 0x0000AAC7u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Peripheral configuration, indexed by \ref rng_PeriphId_t */
static const rng_PeriphConfig_t rng_PeriphConf[ RNG_PERIPH_CNT ] =
{
    { .RngReg = RNG, .RngNvic = NVIC_PERIPH_IRQ_RNG, .RngIsr = Rng_IsrHandler },
};


/** \brief RCC IDs of kernel clock sources, indexed by \ref rng_ClkSource_t */
static const rcc_PeriphId_t rng_ClkSourceLut[ ] =
{
    RCC_PERIPH_RNG_HSI48, /**< \ref RNG_CLK_SOURCE_HSI48 */
    RCC_PERIPH_RNG_PLL1Q, /**< \ref RNG_CLK_SOURCE_PLL1Q */
    RCC_PERIPH_RNG_LSE,   /**< \ref RNG_CLK_SOURCE_LSE   */
    RCC_PERIPH_RNG_LSI,   /**< \ref RNG_CLK_SOURCE_LSI   */
};

_Static_assert( RNG_CLK_SOURCE_CNT == ( sizeof( rng_ClkSourceLut ) / sizeof( rcc_PeriphId_t ) ), "RNG clock source LUT size mismatch" );


/** \brief Conditioning configurations, indexed by \ref rng_ConfigSelection_t */
static const rng_ConfigurationList_t rng_ConfigList[ ] =
{
    { .Supported = RNG_FUNCTION_ACTIVE, .CrRegVal = RNG_CR_NIST_VALUE         , .HtcrRegVal = RNG_HTCR_NIST_VALUE , .NscrRegVal = RNG_NSCR_NIST_CONFIG }, /**< \ref RNG_CONFIG_SELECTION_NIST           */
#if ( RNG_CR_NIST_VALUE == 0x00F00D00u )
    { .Supported = RNG_FUNCTION_ACTIVE, .CrRegVal = RNG_CR_AIS31_SPEED_VALUE  , .HtcrRegVal = RNG_HTCR_AIS31_VALUE, .NscrRegVal = RNG_NSCR_RESET_VALUE }, /**< \ref RNG_CONFIG_SELECTION_AIS_31_SPEED   */
    { .Supported = RNG_FUNCTION_ACTIVE, .CrRegVal = RNG_CR_AIS31_ENTROPY_VALUE, .HtcrRegVal = RNG_HTCR_AIS31_VALUE, .NscrRegVal = RNG_NSCR_RESET_VALUE }, /**< \ref RNG_CONFIG_SELECTION_AIS_31_ENTROPY */
#endif /* RNG_CR_NIST_VALUE */
};

_Static_assert( RNG_CONFIG_SELECTION_CNT == ( sizeof( rng_ConfigList ) / sizeof( rng_ConfigurationList_t ) ), "RNG configuration LUT size mismatch" );


/** \brief Runtime data, indexed by \ref rng_PeriphId_t */
static volatile rng_RuntimeData_t rng_RuntimeData[ RNG_PERIPH_CNT ] =
{
    { .TransferStyle = RNG_TRANSFER_NONE, .ExecState = RNG_EXEC_STATE_IDLE, .RngRcc = RCC_PERIPH_RNG_HSI48, .DataRdyCallback = RNG_NULL_PTR, .ErrorCallback = RNG_NULL_PTR },
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
 * Sequence: generator stop, interrupt deactivation, kernel clock selection and
 * activation (clock of the previous source released), conditioning configuration,
 * callbacks and transfer style storage, interrupt configuration
 * (\ref RNG_TRANSFER_INTERRUPT only).
 *
 * \note  The generator is not started, call \ref Rng_Set_ExecActive afterwards.
 * \note  Internal oscillator of the selected clock source (HSI48, LSI) is started
 *        by RCC module, LSE and PLL1 Q output shall be running. The RNG kernel
 *        clock shall be 48 MHz for NIST compliance.
 *
 * \param rngConfig [in]: Peripheral configuration structure. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Init( rng_ConfigStruct_t * const rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( ( RNG_NULL_PTR       != rngConfig                ) &&
        ( RNG_PERIPH_CNT      > rngConfig->PeriphId      ) &&
        ( RNG_TRANSFER_CNT    > rngConfig->TransferStyle ) &&
        ( RNG_CLK_SOURCE_CNT  > rngConfig->ClkSource     )    )
    {
        const rng_PeriphId_t rngId = rngConfig->PeriphId;

        /*------------- Generator of previous initialization stopped -----------*/
        retState = Rng_Set_ExecInactive( rngId );

        if( RNG_REQUEST_OK == retState )
        {
            retState = Rng_Set_IrqInactive( rngId );
        }
        else
        {
            /* Generator was not stopped */
        }

        /*----------------------- Clock and peripheral -----------------------*/
        if( RNG_REQUEST_OK == retState )
        {
            retState = Rng_Set_ClkSource( rngId, rngConfig->ClkSource );
        }
        else
        {
            /* Previous step failed */
        }

        if( RNG_REQUEST_OK == retState )
        {
            retState = Rng_Set_Configuration( rngId, rngConfig->ConfigSelection );
        }
        else
        {
            /* Previous step failed */
        }

        /*------------------------- Transfer handling ------------------------*/
        if( RNG_REQUEST_OK == retState )
        {
            rng_RuntimeData[ rngId ].TransferStyle   = rngConfig->TransferStyle;
            rng_RuntimeData[ rngId ].DataRdyCallback = rngConfig->DataRdyCallback;
            rng_RuntimeData[ rngId ].ErrorCallback   = rngConfig->ErrorCallback;
        }
        else
        {
            /* Previous step failed */
        }

        if( ( RNG_REQUEST_OK         == retState                 ) &&
            ( RNG_TRANSFER_INTERRUPT == rngConfig->TransferStyle )    )
        {
            retState = Rng_Set_IrqPriority( rngId, rngConfig->IrqPrio );

            if( RNG_REQUEST_OK == retState )
            {
                retState = Rng_Set_IrqActive( rngId );
            }
            else
            {
                /* Priority configuration failed */
            }
        }
        else
        {
            /* Interrupts are not used or previous step failed */
        }
    }
    else
    {
        /* Null pointer or invalid configuration */
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes module Rng
 *
 * Disables interrupt (NVIC and peripheral), stops the generator, resets the
 * peripheral, disables its clock and removes user callbacks.
 *
 * \param rngConfig [in]: Peripheral configuration structure (only PeriphId is used).
 *                        Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Deinit( rng_ConfigStruct_t * const rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( ( RNG_NULL_PTR  != rngConfig           ) &&
        ( RNG_PERIPH_CNT > rngConfig->PeriphId )    )
    {
        const rng_PeriphId_t rngId = rngConfig->PeriphId;

        /* -1- Interrupt and generator */
        const rng_RequestState_t irqState  = Rng_Set_IrqInactive( rngId );
        const rng_RequestState_t execState = Rng_Set_ExecInactive( rngId );

        /* -2- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( rng_RuntimeData[ rngId ].RngRcc );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( rng_RuntimeData[ rngId ].RngRcc );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( rng_RuntimeData[ rngId ].RngRcc );

        /* -3- Runtime data */
        rng_RuntimeData[ rngId ].TransferStyle   = RNG_TRANSFER_NONE;
        rng_RuntimeData[ rngId ].ExecState       = RNG_EXEC_STATE_IDLE;
        rng_RuntimeData[ rngId ].DataRdyCallback = RNG_NULL_PTR;
        rng_RuntimeData[ rngId ].ErrorCallback   = RNG_NULL_PTR;

        if( ( RNG_REQUEST_OK == irqState      ) &&
            ( RNG_REQUEST_OK == execState     ) &&
            ( RCC_REQUEST_OK == rstActState   ) &&
            ( RCC_REQUEST_OK == rstInactState ) &&
            ( RCC_REQUEST_OK == clkState      )    )
        {
            retState = RNG_REQUEST_OK;
        }
        else
        {
            retState = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module Rng
 *
 * Processes peripherals initialized with \ref RNG_TRANSFER_BLOCKING while the
 * generation is running (not paused). Active errors are cleared and reported
 * by error callback, data generated without error are reported by data ready
 * callback. Shall be called periodically.
 */
void Rng_Task( void )
{
    for( rng_PeriphId_t rngId = RNG_PERIPH_1; RNG_PERIPH_CNT > rngId; rngId ++ )
    {
        if( ( RNG_TRANSFER_BLOCKING == rng_RuntimeData[ rngId ].TransferStyle ) &&
            ( RNG_EXEC_STATE_BUSY   == rng_RuntimeData[ rngId ].ExecState     )    )
        {
            rng_Error_t         errMask    = RNG_ERROR_NONE;
            rng_FunctionState_t readyState = RNG_FUNCTION_INACTIVE;

            (void)Rng_Get_Error( rngId, &errMask );

            if( RNG_ERROR_NONE != errMask )
            {
                (void)Rng_Clear_Error( rngId, errMask );

                if( RNG_NULL_PTR != rng_RuntimeData[ rngId ].ErrorCallback )
                {
                    rng_RuntimeData[ rngId ].ErrorCallback( errMask );
                }
                else
                {
                    /* Error callback is not configured */
                }
            }
            else
            {
                (void)Rng_Get_DataReadyState( rngId, &readyState );
            }

            if( RNG_FUNCTION_ACTIVE == readyState )
            {
                rng_RandomData_t         randData;
                const rng_RequestState_t dataState = Rng_Get_Data( rngId, &randData );

                if( ( RNG_REQUEST_OK == dataState                            ) &&
                    ( RNG_NULL_PTR   != rng_RuntimeData[ rngId ].DataRdyCallback )    )
                {
                    rng_RuntimeData[ rngId ].DataRdyCallback( randData );
                }
                else
                {
                    /* Data are not valid or callback is not configured */
                }
            }
            else
            {
                /* No data ready or error was processed */
            }
        }
        else
        {
            /* Peripheral is not polled by the task */
        }
    }
}


/**
 * \brief Loads default values for Random Number Generator (RNG) peripheral
 *
 * Default configuration: RNG_PERIPH_1, HSI48 kernel clock, NIST configuration,
 * blocking transfer style, interrupt priority 0, no callbacks.
 *
 * \param rngConfig [out]: Pointer to configuration structure. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_DefaultConfig( rng_ConfigStruct_t *rngConfig )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_NULL_PTR != rngConfig )
    {
        rngConfig->PeriphId        = RNG_PERIPH_1;
        rngConfig->ClkSource       = RNG_CLK_SOURCE_HSI48;
        rngConfig->ConfigSelection = RNG_CONFIG_SELECTION_NIST;
        rngConfig->TransferStyle   = RNG_TRANSFER_BLOCKING;
        rngConfig->IrqPrio         = RNG_DEFAULT_IRQ_PRIO;
        rngConfig->DataRdyCallback = RNG_NULL_PTR;
        rngConfig->ErrorCallback   = RNG_NULL_PTR;

        retState = RNG_REQUEST_OK;
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
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        /* State is set before the generator is enabled - in interrupt style the
         * data ready interrupt follows immediately and the ISR delivers data
         * only while the generation is running */
        rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_BUSY;

        LL_RNG_Enable( rng_PeriphConf[ rngId ].RngReg );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_ACTIVE );

        if( RNG_REQUEST_OK != retValue )
        {
            /* Generator has not been enabled */
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_IDLE;
        }
        else
        {
            /* Generation is running */
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
 * Generator is stopped, configuration and peripheral clock are kept.
 *
 * \param rngId [in]: Peripheral identification
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_Disable( rng_PeriphConf[ rngId ].RngReg );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_INACTIVE );

        if( RNG_REQUEST_OK == retValue )
        {
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_IDLE;
        }
        else
        {
            /* Generator has not been disabled */
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
 * \param rngId  [in]: Peripheral identification
 * \param state [out]: Pointer to store state, \ref RNG_FUNCTION_ACTIVE if the
 *                     generation is running (also when paused). Must not be NULL.
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
        if( RNG_EXEC_STATE_IDLE != rng_RuntimeData[ rngId ].ExecState )
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
 * \brief Pauses Random Number Generator (RNG) peripheral data delivery.
 *
 * Blocking style: \ref Rng_Task stops processing of the peripheral.
 * Interrupt style: peripheral interrupt is disabled (\ref Rng_Get_IrqState
 * reads inactive while paused).
 *
 * \pre   Generation shall be running (\ref Rng_Set_ExecActive), otherwise
 *        \ref RNG_REQUEST_ERROR.
 *
 * \param rngId [in]: Peripheral identification
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecPauseActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT      > rngId                                ) &&
        ( RNG_EXEC_STATE_BUSY == rng_RuntimeData[ rngId ].ExecState )    )
    {
        if( RNG_TRANSFER_INTERRUPT == rng_RuntimeData[ rngId ].TransferStyle )
        {
            LL_RNG_DisableIT( rng_PeriphConf[ rngId ].RngReg );

            retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledIT, RNG_LL_STATE_INACTIVE );
        }
        else
        {
            /* Blocking style is processed by the task */
            retValue = RNG_REQUEST_OK;
        }

        if( RNG_REQUEST_OK == retValue )
        {
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_PAUSED;
        }
        else
        {
            /* Interrupt has not been disabled */
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Resumes Random Number Generator (RNG) peripheral data delivery.
 *
 * \pre   Generation shall be paused (\ref Rng_Set_ExecPauseActive), otherwise
 *        \ref RNG_REQUEST_ERROR.
 *
 * \param rngId [in]: Peripheral identification
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ExecPauseInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT        > rngId                                ) &&
        ( RNG_EXEC_STATE_PAUSED == rng_RuntimeData[ rngId ].ExecState )    )
    {
        /* State is set before the interrupt is enabled - pending data ready
         * interrupt follows immediately (see \ref Rng_Set_ExecActive) */
        rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_BUSY;

        if( RNG_TRANSFER_INTERRUPT == rng_RuntimeData[ rngId ].TransferStyle )
        {
            LL_RNG_EnableIT( rng_PeriphConf[ rngId ].RngReg );

            retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledIT, RNG_LL_STATE_ACTIVE );

            if( ( RNG_REQUEST_OK      != retValue                            ) &&
                ( RNG_EXEC_STATE_BUSY != rng_RuntimeData[ rngId ].ExecState )    )
            {
                /* Delivery was resumed and paused again from the data ready callback */
                retValue = RNG_REQUEST_OK;
            }
            else
            {
                /* Interrupt enable confirmed or not applied */
            }
        }
        else
        {
            /* Blocking style is processed by the task */
            retValue = RNG_REQUEST_OK;
        }

        if( RNG_REQUEST_OK != retValue )
        {
            /* Interrupt has not been enabled */
            rng_RuntimeData[ rngId ].ExecState = RNG_EXEC_STATE_PAUSED;
        }
        else
        {
            /* Delivery resumed */
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
 * \param rngId  [in]: Peripheral identification
 * \param state [out]: Pointer to store pause state. Must not be NULL.
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
        if( RNG_EXEC_STATE_PAUSED == rng_RuntimeData[ rngId ].ExecState )
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
 * \brief Reads one 128-bit block of random data from the output FIFO.
 *
 * Every word is read after data ready flag (DRDY). Data are valid only if no
 * seed error occurred during the read (SEIS checked after the block, see
 * reference manual - data read with seed error must not be used).
 *
 * \param rngId [in] : Peripheral identification
 * \param data  [out]: Pointer to store random data. Must not be NULL.
 *
 * \return Returns \ref RNG_REQUEST_OK if the complete block was read without
 *         seed error. Returns \ref RNG_REQUEST_ERROR if no data are ready,
 *         seed error occurred or parameters are invalid.
 */
rng_RequestState_t Rng_Get_Data( rng_PeriphId_t rngId, rng_RandomData_t * const data )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId ) &&
        ( RNG_NULL_PTR  != data  )    )
    {
        const uint32_t readyState = LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == readyState )
        {
            for( uint32_t index = 0u; RNG_FIFO_LEN > index; index ++ )
            {
                retValue = Rng_Wait_RegState( rngId, LL_RNG_IsActiveFlag_DRDY, RNG_LL_STATE_ACTIVE );

                if( RNG_REQUEST_OK == retValue )
                {
                    data->value[ index ] = LL_RNG_ReadRandData32( rng_PeriphConf[ rngId ].RngReg );
                }
                else
                {
                    /* Next word of the block is not available */
                    break;
                }
            }

            if( ( RNG_REQUEST_OK      == retValue                                                 ) &&
                ( RNG_LL_STATE_ACTIVE == LL_RNG_IsActiveFlag_SEIS( rng_PeriphConf[ rngId ].RngReg ) )    )
            {
                /* Seed error - data must not be used */
                retValue = RNG_REQUEST_ERROR;
            }
            else
            {
                /* Block read without seed error or read failed */
            }
        }
        else
        {
            /* No data ready */
            retValue = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/*-------------------------- Primitive functionality -------------------------*/

/**
 * \brief Sets configuration of Random Number Generator (RNG) peripheral.
 *
 * The Random Number Generator (RNG) offers three configurations for the
 * conditioning component. Each configuration is compliant with different
 * standards and provides different throughput and entropy rate on the output:
 *
 * - NIST SP800-90B compliant configuration (device specific values from CMSIS
 *   device header, see AN4230).
 * - AIS-31 T0 to T8 compliant configuration - high speed.
 * - AIS-31 T0 to T8 compliant configuration - high entropy.
 *
 * Configuration is written with conditioning soft reset (CR config bits, NSCR,
 * HTCR) and verified by read-back. Clock divider, clock error detection and
 * auto reset are not changed.
 *
 * \pre   Configuration shall be supported by the device (AIS-31 only on RNG
 *        version shared with STM32U5) and configuration shall not be locked
 *        (CONFIGLOCK); otherwise \ref RNG_REQUEST_ERROR and no register is changed.
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

    if( ( RNG_PERIPH_CNT           > rngId                                      ) &&
        ( RNG_CONFIG_SELECTION_CNT > configSelection                            ) &&
        ( RNG_FUNCTION_ACTIVE     == rng_ConfigList[ configSelection ].Supported )    )
    {
        RNG_TypeDef * const            rngReg   = rng_PeriphConf[ rngId ].RngReg;
        const rng_ConfigurationList_t *config   = &rng_ConfigList[ configSelection ];
        const uint32_t                 lockState = LL_RNG_IsConfigLocked( rngReg );

        if( RNG_LL_STATE_INACTIVE == lockState )
        {
            /* Reference manual sequence: CONDRST with new CR values, NSCR, HTCR, CONDRST release */
            MODIFY_REG( rngReg->CR, RNG_CR_CONFIG_MASK | RNG_CR_CONDRST, ( config->CrRegVal & RNG_CR_CONFIG_MASK ) | RNG_CR_CONDRST );
            WRITE_REG ( rngReg->NSCR, config->NscrRegVal );
            LL_RNG_SetHealthConfig( rngReg, config->HtcrRegVal );
            LL_RNG_DisableCondReset( rngReg );

            retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE );

            if( ( RNG_REQUEST_OK                          == retValue                                ) &&
                ( ( config->CrRegVal & RNG_CR_CONFIG_MASK ) == READ_BIT( rngReg->CR, RNG_CR_CONFIG_MASK ) ) &&
                ( config->NscrRegVal                      == READ_REG( rngReg->NSCR )                ) &&
                ( config->HtcrRegVal                      == LL_RNG_GetHealthConfig( rngReg )        )    )
            {
                retValue = RNG_REQUEST_OK;
            }
            else
            {
                /* Configuration has not been applied */
                retValue = RNG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Configuration is locked until system reset */
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
 * \brief Returns configuration of Random Number Generator (RNG) peripheral.
 *
 * Configuration is identified by CR configuration bits, NSCR and HTCR values.
 *
 * \note  On RNG version shared with STM32U5 the NIST and AIS-31 high entropy
 *        configurations use the same register values, NIST is reported.
 *
 * \param rngId           [in] : RNG peripheral ID
 * \param configSelection [out]: Pointer where configuration selection will be stored. Must not be NULL.
 *
 * \return Returns \ref RNG_REQUEST_OK if the register values match one of the
 *         supported configurations. Otherwise returns \ref RNG_REQUEST_ERROR.
 */
rng_RequestState_t Rng_Get_Configuration( rng_PeriphId_t rngId, rng_ConfigSelection_t * const configSelection )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId           ) &&
        ( RNG_NULL_PTR  != configSelection )    )
    {
        const RNG_TypeDef * const rngReg  = rng_PeriphConf[ rngId ].RngReg;
        const uint32_t            crVal   = READ_BIT( rngReg->CR, RNG_CR_CONFIG_MASK );
        const uint32_t            nscrVal = READ_REG( rngReg->NSCR );
        const uint32_t            htcrVal = LL_RNG_GetHealthConfig( rngReg );

        for( rng_ConfigSelection_t searchId = RNG_CONFIG_SELECTION_NIST; RNG_CONFIG_SELECTION_CNT > searchId; searchId ++ )
        {
            if( ( RNG_FUNCTION_ACTIVE                              == rng_ConfigList[ searchId ].Supported  ) &&
                ( ( rng_ConfigList[ searchId ].CrRegVal & RNG_CR_CONFIG_MASK ) == crVal                     ) &&
                ( rng_ConfigList[ searchId ].NscrRegVal            == nscrVal                               ) &&
                ( rng_ConfigList[ searchId ].HtcrRegVal            == htcrVal                               )    )
            {
                *configSelection = searchId;

                retValue = RNG_REQUEST_OK;
                break;
            }
            else
            {
                /* Unknown configuration, keep return state as error */
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
 * \param state [out]: Pointer to state of data ready flag. Must not be NULL.
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
        const uint32_t regValue = LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == regValue )
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
 * Error mask is created from error interrupt status flags (SEIS, CEIS).
 *
 * \param rngId   [in] : RNG peripheral ID
 * \param errMask [out]: Pointer to error mask. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_Error( rng_PeriphId_t rngId, rng_Error_t * const errMask )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId   ) &&
        ( RNG_NULL_PTR  != errMask )    )
    {
        const uint32_t seedErr  = LL_RNG_IsActiveFlag_SEIS( rng_PeriphConf[ rngId ].RngReg );
        const uint32_t clockErr = LL_RNG_IsActiveFlag_CEIS( rng_PeriphConf[ rngId ].RngReg );

        if( ( RNG_LL_STATE_ACTIVE == seedErr  ) &&
            ( RNG_LL_STATE_ACTIVE == clockErr )    )
        {
            *errMask = RNG_ERROR_ALL;
        }
        else if( RNG_LL_STATE_ACTIVE == seedErr )
        {
            *errMask = RNG_ERROR_SEED_ERR;
        }
        else if( RNG_LL_STATE_ACTIVE == clockErr )
        {
            *errMask = RNG_ERROR_CLOCK_ERR;
        }
        else
        {
            *errMask = RNG_ERROR_NONE;
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
 * \brief Clears error flags of Random Number Generator (RNG) peripheral.
 *
 * Seed error: if the seed error is still active (SECS) and auto reset is
 * disabled, the conditioning logic is reset first (reference manual recovery
 * sequence), then SEIS is cleared. With auto reset enabled the hardware
 * recovers by itself.
 *
 * \param rngId   [in]: RNG peripheral ID
 * \param errMask [in]: Error mask to be cleared
 *
 * \return Returns \ref RNG_REQUEST_OK if requested flags read back cleared.
 *         Otherwise returns \ref RNG_REQUEST_ERROR (eg. the error is still present).
 */
rng_RequestState_t Rng_Clear_Error( rng_PeriphId_t rngId, rng_Error_t errMask )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId   ) &&
        ( RNG_ERROR_CNT  > errMask )    )
    {
        RNG_TypeDef * const rngReg = rng_PeriphConf[ rngId ].RngReg;

        retValue = RNG_REQUEST_OK;

        if( 0u != ( (uint32_t)errMask & (uint32_t)RNG_ERROR_CLOCK_ERR ) )
        {
            LL_RNG_ClearFlag_CEIS( rngReg );

            if( RNG_LL_STATE_ACTIVE == LL_RNG_IsActiveFlag_CEIS( rngReg ) )
            {
                retValue = RNG_REQUEST_ERROR;
            }
            else
            {
                /* Clock error flag cleared */
            }
        }
        else
        {
            /* Clock error is not requested */
        }

        if( 0u != ( (uint32_t)errMask & (uint32_t)RNG_ERROR_SEED_ERR ) )
        {
            if( ( RNG_LL_STATE_ACTIVE   == LL_RNG_IsActiveFlag_SECS( rngReg ) ) &&
                ( RNG_LL_STATE_INACTIVE == LL_RNG_IsEnabledArdis( rngReg )    )    )
            {
                if( RNG_REQUEST_OK != Rng_Set_CondReset( rngId ) )
                {
                    retValue = RNG_REQUEST_ERROR;
                }
                else
                {
                    /* Conditioning logic reset done */
                }
            }
            else
            {
                /* Seed error is not active or auto reset is enabled */
            }

            LL_RNG_ClearFlag_SEIS( rngReg );

            if( RNG_LL_STATE_ACTIVE == LL_RNG_IsActiveFlag_SEIS( rngReg ) )
            {
                retValue = RNG_REQUEST_ERROR;
            }
            else
            {
                /* Seed error flag cleared */
            }
        }
        else
        {
            /* Seed error is not requested */
        }
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
 * the IE bit in the CR register is set.
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

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledClkErrorDetect, RNG_LL_STATE_ACTIVE );
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
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClockErrorDetectInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_DisableClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledClkErrorDetect, RNG_LL_STATE_INACTIVE );
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
 * \param state [out]: Pointer to state of clock error detection functionality. Must not be NULL.
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
        const uint32_t regValue = LL_RNG_IsEnabledClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == regValue )
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
 * \brief Activates auto reset functionality (ARDIS = 0).
 *
 * When a noise source (or seed) error occurs, the RNG stops generating random
 * numbers and sets both SEIS and SECS bits. With auto reset enabled the
 * hardware clears SECS and restarts the generation automatically, application
 * only clears SEIS (\ref Rng_Clear_Error).
 *
 * \note  Keeping the auto reset enabled simplifies the management of noise
 *        source errors.
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

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledArdis, RNG_LL_STATE_ACTIVE );
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates auto reset functionality (ARDIS = 1).
 *
 * After seed error the conditioning logic has to be reset by software,
 * \ref Rng_Clear_Error performs the recovery sequence.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_AutoResetInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        LL_RNG_DisableArdis( rng_PeriphConf[ rngId ].RngReg );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledArdis, RNG_LL_STATE_INACTIVE );
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns state of auto reset functionality.
 *
 * \param rngId     [in] : RNG peripheral ID
 * \param reqState [out]: Pointer to state of auto reset functionality. Must not be NULL.
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
        const uint32_t regValue = LL_RNG_IsEnabledArdis( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == regValue )
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
 * \brief Selects kernel clock source and activates the peripheral clock.
 *
 * \warning The clock source shall be 48MHz for NIST SP800-90B compliance.
 *
 * \note  Internal oscillator (HSI48, LSI) is started by RCC module, LSE and
 *        PLL1 Q output shall be running.
 * \note  The kernel clock multiplexer completes a switch only with the clock of
 *        the previous source. After a system reset with LSI as kernel clock (LSI
 *        is stopped by the reset) the RNG would get no kernel clock - before a
 *        source other than LSI is selected, LSI is started and stopped again if
 *        it is not running.
 *
 * \param rngId     [in]: RNG peripheral ID
 * \param clkSource [in]: Clock source
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_ClkSource( rng_PeriphId_t rngId, rng_ClkSource_t clkSource )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT     > rngId     ) &&
        ( RNG_CLK_SOURCE_CNT > clkSource )    )
    {
        const rcc_PeriphId_t prevRcc  = rng_RuntimeData[ rngId ].RngRcc;
        const rcc_PeriphId_t newRcc   = rng_ClkSourceLut[ clkSource ];
        rcc_RequestState_t   rccState = RCC_REQUEST_OK;

        /* Peripheral clock of the previous source is released - RCC does not change
         * the kernel clock multiplexer of an active peripheral */
        if( prevRcc != newRcc )
        {
            rcc_FunctionState_t prevState = RCC_FUNCTION_INACTIVE;

            rccState = Rcc_Get_PeriphState( prevRcc, &prevState );

            if( ( RCC_REQUEST_OK      == rccState  ) &&
                ( RCC_FUNCTION_ACTIVE == prevState )    )
            {
                rccState = Rcc_Set_PeriphInactive( prevRcc );
            }
            else
            {
                /* Peripheral clock is not active or its state is not available */
            }
        }
        else
        {
            /* Same source - nothing to release */
        }

        /* Kernel clock switch pending from LSI (system reset while LSI was the kernel
         * clock, LSI stopped by the reset) is completed only by LSI clock - the RNG
         * would get no kernel clock. LSI is started and stopped again if it is not
         * running. */
        if( ( RCC_REQUEST_OK     == rccState  ) &&
            ( RNG_CLK_SOURCE_LSI != clkSource )    )
        {
            rcc_FunctionState_t lsiState = RCC_FUNCTION_ACTIVE;

            rccState = Rcc_Get_OscState( RCC_OSC_LSI, &lsiState );

            if( ( RCC_REQUEST_OK        == rccState ) &&
                ( RCC_FUNCTION_INACTIVE == lsiState )    )
            {
                rccState = Rcc_Set_OscActive( RCC_OSC_LSI );

                if( RCC_REQUEST_OK == rccState )
                {
                    rccState = Rcc_Set_OscInactive( RCC_OSC_LSI );
                }
                else
                {
                    /* LSI did not start */
                }
            }
            else
            {
                /* LSI is running (pending switch completes) or its state is not available */
            }
        }
        else
        {
            /* Previous source not released or LSI is the new source */
        }

        if( RCC_REQUEST_OK == rccState )
        {
            rccState = Rcc_Set_PeriphActive( newRcc );
        }
        else
        {
            /* Clock of the previous source not released or LSI did not start */
        }

        if( RCC_REQUEST_OK == rccState )
        {
            rng_RuntimeData[ rngId ].RngRcc = rng_ClkSourceLut[ clkSource ];

            retValue = RNG_REQUEST_OK;
        }
        else
        {
            /* Clock was not successfully activated */
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
 * \brief Configures clock divider for Random Number Generator (RNG) peripheral.
 *
 * \warning The clock divider shall not be changed for NIST SP800-90B compliance.
 *
 * \note The RNG clock frequency is divided by a programmable factor
 *      (1, 2, 4, 8, ... 32768) before being used by the RNG.
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

    if( ( RNG_PERIPH_CNT  > rngId      ) &&
        ( RNG_CLK_DIV_CNT > clkDivider )    )
    {
        const uint32_t regValue = ( (uint32_t)clkDivider << RNG_CR_CLKDIV_Pos ) & RNG_CR_CLKDIV_Msk;

        LL_RNG_SetClockDivider( rng_PeriphConf[ rngId ].RngReg, regValue );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_GetClockDivider, regValue );
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
 * \param clkDivider [out]: Pointer to clock divider value. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ClkDivider( rng_PeriphId_t rngId, rng_ClkDivider_t * const clkDivider )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId      ) &&
        ( RNG_NULL_PTR  != clkDivider )    )
    {
        const uint32_t regValue = LL_RNG_GetClockDivider( rng_PeriphConf[ rngId ].RngReg );

        *clkDivider = (rng_ClkDivider_t)( regValue >> RNG_CR_CLKDIV_Pos );

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
 * \param callback [in]: Pointer to callback, can be NULL
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
 * \param rngId    [in] : RNG peripheral ID
 * \param callback [out]: Pointer to store callback pointer. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_DataRdyCallback( rng_PeriphId_t rngId, rng_DataRdyCallback_t ** const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId    ) &&
        ( RNG_NULL_PTR  != callback )    )
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
 * \param callback [in]: Pointer to callback, can be NULL
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
 * \param rngId    [in] : RNG peripheral ID
 * \param callback [out]: Pointer to store callback pointer. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_ErrorCallback( rng_PeriphId_t rngId, rng_ErrCallback_t ** const callback )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId    ) &&
        ( RNG_NULL_PTR  != callback )    )
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
 * peripheral interrupt triggers (DRDY, SEIS, CEIS).
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Handler( rng_PeriphConf[ rngId ].RngNvic,
                                                                    rng_PeriphConf[ rngId ].RngIsr );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( rng_PeriphConf[ rngId ].RngNvic );
        }
        else
        {
            /* Handler configuration failed */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            LL_RNG_EnableIT( rng_PeriphConf[ rngId ].RngReg );

            retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledIT, RNG_LL_STATE_ACTIVE );
        }
        else
        {
            retState = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief De-activates global Interrupt Requests (IRQ) for RNG peripheral.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( rng_PeriphConf[ rngId ].RngNvic );

        LL_RNG_DisableIT( rng_PeriphConf[ rngId ].RngReg );

        retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledIT, RNG_LL_STATE_INACTIVE );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState = RNG_REQUEST_ERROR;
        }
        else
        {
            /* Keep peripheral interrupt deactivation result */
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
 * \param reqState [out]: Activation state of peripheral interrupt request. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_IrqState( rng_PeriphId_t rngId, rng_FunctionState_t * const reqState )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId    ) &&
        ( RNG_NULL_PTR  != reqState )    )
    {
        const uint32_t regValue = LL_RNG_IsEnabledIT( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == regValue )
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
 * \brief Configures Interrupt Requests (IRQ) priority.
 *
 * \param rngId   [in]: RNG peripheral ID
 * \param irqPrio [in]: Required priority (validated by NVIC module)
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_IrqPriority( rng_PeriphId_t rngId, rng_IrqPrio_t irqPrio )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( RNG_PERIPH_CNT > rngId )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( rng_PeriphConf[ rngId ].RngNvic, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = RNG_REQUEST_OK;
        }
        else
        {
            retState = RNG_REQUEST_ERROR;
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
 * \param irqPrio [out]: Configured priority. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Get_IrqPriority( rng_PeriphId_t rngId, rng_IrqPrio_t * const irqPrio )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId   ) &&
        ( RNG_NULL_PTR  != irqPrio )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( rng_PeriphConf[ rngId ].RngNvic, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = RNG_REQUEST_OK;
        }
        else
        {
            retState = RNG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RNG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Waits until register (bit field) reaches required value and no
 *        conditioning soft reset is in progress.
 *
 * \param rngId    [in]: RNG peripheral ID, must be valid
 * \param llGetter [in]: LL function returning checked register value
 * \param reqValue [in]: Required value
 *
 * \return Returns \ref RNG_REQUEST_OK if the value was reached within timeout.
 *         Otherwise returns \ref RNG_REQUEST_ERROR.
 */
static rng_RequestState_t Rng_Wait_RegState( rng_PeriphId_t rngId, rng_LlGetter_t llGetter, uint32_t reqValue )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; RNG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t condRstState = LL_RNG_IsEnabledCondReset( rng_PeriphConf[ rngId ].RngReg );
        const uint32_t regValue     = llGetter( rng_PeriphConf[ rngId ].RngReg );

        if( ( RNG_LL_STATE_INACTIVE == condRstState ) &&
            ( reqValue              == regValue     )    )
        {
            retState = RNG_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = RNG_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Resets conditioning logic (CONDRST 1 -> 0) and waits for completion.
 *
 * \param rngId [in]: RNG peripheral ID, must be valid
 *
 * \return Returns \ref RNG_REQUEST_OK if the reset finished within timeout.
 *         Otherwise returns \ref RNG_REQUEST_ERROR.
 */
static rng_RequestState_t Rng_Set_CondReset( rng_PeriphId_t rngId )
{
    LL_RNG_EnableCondReset( rng_PeriphConf[ rngId ].RngReg );
    LL_RNG_DisableCondReset( rng_PeriphConf[ rngId ].RngReg );

    return ( Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE ) );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief RNG interrupt handler.
 *
 * Errors are cleared and reported by error callback (data are not read). Data
 * ready: one block is read; it is reported by data ready callback only while
 * the generation is running, otherwise it is discarded (FIFO drain).
 */
static void Rng_IsrHandler( void )
{
    const rng_PeriphId_t rngId   = RNG_PERIPH_1;
    rng_Error_t          errMask = RNG_ERROR_NONE;

    (void)Rng_Get_Error( rngId, &errMask );

    if( RNG_ERROR_NONE != errMask )
    {
        /*----------------------------- Error interrupt ----------------------*/
        (void)Rng_Clear_Error( rngId, errMask );

        if( RNG_NULL_PTR != rng_RuntimeData[ rngId ].ErrorCallback )
        {
            rng_RuntimeData[ rngId ].ErrorCallback( errMask );
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }
    else if( RNG_LL_STATE_ACTIVE == LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ rngId ].RngReg ) )
    {
        /*-------------------- Data ready (DRDY) interrupt -------------------*/
        rng_RandomData_t         randData;
        const rng_RequestState_t dataState = Rng_Get_Data( rngId, &randData );

        if( ( RNG_REQUEST_OK      == dataState                                ) &&
            ( RNG_EXEC_STATE_BUSY == rng_RuntimeData[ rngId ].ExecState       ) &&
            ( RNG_NULL_PTR        != rng_RuntimeData[ rngId ].DataRdyCallback )    )
        {
            rng_RuntimeData[ rngId ].DataRdyCallback( randData );
        }
        else
        {
            /* Data are not valid, generation is not running or callback is not configured */
        }
    }
    else
    {
        /* No interrupt source is active */
    }
}

/* ================================ TASKS =================================== */

