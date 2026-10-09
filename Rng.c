/**
 * \author Mr.Nobody
 * \file Rng.c
 * \ingroup Rng
 * \brief Random Number Generator (RNG) module common functionality
 *
 * # Introduction #
 * The RNG is a true random number generator that provides 32-bit random numbers
 * generated from a continuous analog noise (ring oscillators) and post-processed
 * by a linear feedback shift register (LFSR).
 *
 * # RNG main features (STM32L4 / STM32L4+) #
 * - 32-bit random numbers from the 48 MHz clock multiplexer CLK48SEL (HSI48,
 *   PLLSAI1 output Q, main PLL output Q or MSI, shared with USB and SDMMC).
 *   Module delivers blocks of \ref RNG_FIFO_LEN words read one by one (DRDY).
 * - Seed error detection (abnormal sequence of the noise source), associated
 *   error management.
 * - Clock error detection (RNG clock too low compared to HCLK). Three RNG
 *   variants exist in the family (selected by CMSIS register definitions):
 *   - STM32L4 (RNG_CR without CED): clock error detection is always active,
 *     fixed conditioning, no clock divider, no auto reset.
 *   - STM32L4R / L4S (CED): clock error detection can be disabled, CED is
 *     changed only with stopped generator - the module stops and restarts a
 *     running generator for the change. Fixed conditioning, no clock divider,
 *     no auto reset.
 *   - STM32L4P5 / L4Q5 (CONDRST): conditioning configuration (NIST values of
 *     the CMSIS device header), clock divider CLKDIV, configuration lock,
 *     automatic reset after seed error (no ARDIS bit). Configuration changes
 *     are written with conditioning soft reset (CONDRST).
 * - Functions of the shared interface accept only the fixed hardware
 *   configuration of features not available on the MCU.
 *
 * # Seed error recovery #
 * Data read while seed error is active must not be used. STM32L4 / L4R / L4S
 * (RM0351, RM0394, RM0432): the application clears SEIS, then the generator
 * is restarted by clearing and setting RNGEN. STM32L4P5 / L4Q5: if the seed
 * error is still active (SECS), the conditioning logic is reset (CONDRST),
 * then SEIS is cleared (sequence of the STM32L4 HAL RNG_RecoverSeedError).
 * Both are done by \ref Rng_Clear_Error.
 *
 * \note  The health test configuration register (HTCR, STM32L4P5 / L4Q5) is
 *        written with every configuration, but it is not read back - read and
 *        write access requires the preceding magic key write.
 *
 * \note  STM32L4 lines with HASH (STM32L4A6): RNG shares the interrupt line
 *        with HASH (HASH_RNG), the module owns the interrupt handler.
 *
 * \note  Interrupt style: a new word is ready after a few tens of RNG clock
 *        periods (about 1 us at 48 MHz), the data ready interrupt therefore follows
 *        immediately after every delivered block and the ISR occupies the CPU.
 *        The application shall pause the delivery (\ref Rng_Set_ExecPauseActive,
 *        allowed also from the data ready callback) when it has enough data.
 *
 * \note  FIPS PUB 140-2 continuous random number test (comparison of
 *        consecutive numbers, first number after enable not used) is
 *        responsibility of the application.
 *
 * \note  The module is empty on MCUs without RNG peripheral.
 *
 * \note  STM32L4 errata sheets not reviewed yet.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Rng.h"                            /* Self include                   */
#include "Rng_Port.h"                       /* Own port file include          */
#include "Rng_Types.h"                      /* Module types definitions       */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
/* STM32L4P5 / L4Q5 LL_RNG_SetHealthConfig() ignores its parameter (always writes RNG_HTCFG) -
 * unused parameter warning of the LL header is suppressed, the function is not used */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Stm32_rng.h"                      /* RNG peripheral functionality   */
#pragma GCC diagnostic pop

#if defined(RNG)
/* ============================== TYPEDEFS ================================== */

/** \brief Peripheral configuration item */
typedef struct
{
    RNG_TypeDef          *RngReg;  /**< RNG peripheral registers  */
    nvic_PeriphIrqList_t  RngNvic; /**< RNG NVIC configuration ID */
    nvic_IsrCallback_t    RngIsr;  /**< RNG ISR routine           */
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
#if defined(RNG_CR_CONDRST)
static rng_RequestState_t   Rng_Set_CondReset       ( rng_PeriphId_t rngId );
#else
static rng_RequestState_t   Rng_Set_Restart         ( rng_PeriphId_t rngId );
#endif /* RNG_CR_CONDRST */
#if defined(RNG_CR_CED)
static rng_RequestState_t   Rng_Set_ClkErrorDetect  ( rng_PeriphId_t rngId, rng_FunctionState_t cedState );
#endif /* RNG_CR_CED */

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

#if defined(HASH) && \
    !defined(DMAMUX1)
/** RNG interrupt shared with HASH (STM32L4 lines with HASH) */
#define RNG_NVIC_IRQ                    ( NVIC_PERIPH_IRQ_HASH_RNG )
#else
/** RNG global interrupt */
#define RNG_NVIC_IRQ                    ( NVIC_PERIPH_IRQ_RNG )
#endif /* HASH */

#if defined(RCC_CRRCR_HSI48ON)
/** Default kernel clock source used by \ref Rng_Get_DefaultConfig */
#define RNG_DEFAULT_CLK_SOURCE          ( RNG_CLK_SOURCE_HSI48 )

/** RCC ID of the default kernel clock source */
#define RNG_DEFAULT_RCC                 ( RCC_PERIPH_RNG_HSI48 )
#else
/** Default kernel clock source used by \ref Rng_Get_DefaultConfig (no HSI48 on STM32L47x / L48x) */
#define RNG_DEFAULT_CLK_SOURCE          ( RNG_CLK_SOURCE_PLLQ )

/** RCC ID of the default kernel clock source */
#define RNG_DEFAULT_RCC                 ( RCC_PERIPH_RNG_PLLQ )
#endif /* RCC_CRRCR_HSI48ON */

#if defined(RNG_CR_CONDRST)
/** CR register bits selecting the conditioning configuration */
#define RNG_CR_CONFIG_MASK              ( RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_NISTC )

/** CR register value of NIST configuration (CMSIS device header, AN4230) */
#define RNG_CR_NIST_CONFIG              ( RNG_CR_CONFIG_VAL & RNG_CR_CONFIG_MASK )

/** HTCR register value of NIST configuration (CMSIS device header, AN4230) */
#define RNG_HTCR_NIST_CONFIG            ( RNG_HTCFG )
#endif /* RNG_CR_CONDRST */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Peripheral configuration, indexed by \ref rng_PeriphId_t */
static const rng_PeriphConfig_t rng_PeriphConf[ RNG_PERIPH_CNT ] =
{
    { .RngReg = RNG, .RngNvic = RNG_NVIC_IRQ, .RngIsr = Rng_IsrHandler },
};


/** \brief RCC IDs of kernel clock sources, indexed by \ref rng_ClkSource_t (the list of the sources contains
 *         the sources of the device line only) */
static const rcc_PeriphId_t rng_ClkSourceLut[ ] =
{
#if defined(RCC_CRRCR_HSI48ON)
    RCC_PERIPH_RNG_HSI48   , /**< \ref RNG_CLK_SOURCE_HSI48    */
#endif /* RCC_CRRCR_HSI48ON */
#if defined(RCC_CR_PLLSAI1ON)
    RCC_PERIPH_RNG_PLLSAI1Q, /**< \ref RNG_CLK_SOURCE_PLLSAI1Q */
#endif /* RCC_CR_PLLSAI1ON */
    RCC_PERIPH_RNG_PLLQ    , /**< \ref RNG_CLK_SOURCE_PLLQ     */
    RCC_PERIPH_RNG_MSI     , /**< \ref RNG_CLK_SOURCE_MSI      */
};

_Static_assert( RNG_CLK_SOURCE_CNT == ( sizeof( rng_ClkSourceLut ) / sizeof( rcc_PeriphId_t ) ), "RNG clock source LUT size mismatch" );


/** \brief Runtime data, indexed by \ref rng_PeriphId_t */
static volatile rng_RuntimeData_t rng_RuntimeData[ RNG_PERIPH_CNT ] =
{
    { .TransferStyle = RNG_TRANSFER_NONE, .ExecState = RNG_EXEC_STATE_IDLE, .RngRcc = RNG_DEFAULT_RCC, .DataRdyCallback = RNG_NULL_PTR, .ErrorCallback = RNG_NULL_PTR },
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
 * activation (clock of the previous source released), configuration (fixed
 * conditioning check, NIST configuration with conditioning reset on STM32L4P5 /
 * L4Q5), callbacks and transfer style storage, interrupt configuration
 * (\ref RNG_TRANSFER_INTERRUPT only).
 *
 * \note  The generator is not started, call \ref Rng_Set_ExecActive afterwards.
 * \note  HSI48 and MSI are started by RCC module, PLL outputs Q shall be
 *        configured and running when selected (they are not started by RCC module).
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
 * generation is running (not paused). Active errors are cleared (seed error
 * recovery included) and reported by error callback, data generated without
 * error are reported by data ready callback. Shall be called periodically.
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

                if( ( RNG_REQUEST_OK == dataState                                ) &&
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
 * Default configuration: RNG_PERIPH_1, HSI48 kernel clock (main PLL output Q on
 * STM32L47x / L48x without HSI48 - 40 MHz in RCC default configuration), NIST
 * configuration, blocking transfer style, interrupt priority 0, no callbacks.
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
        rngConfig->ClkSource       = RNG_DEFAULT_CLK_SOURCE;
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
 * \brief Reads one block of random data (\ref RNG_FIFO_LEN words).
 *
 * STM32L4 RNG delivers one 32-bit word at a time, every word is read after
 * data ready flag (DRDY). Data are valid only if no seed error occurred during
 * the read (SEIS checked after the block, see reference manual - data read
 * with seed error must not be used).
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

            const uint32_t seedErrState = LL_RNG_IsActiveFlag_SEIS( rng_PeriphConf[ rngId ].RngReg );

            if( ( RNG_REQUEST_OK      == retValue     ) &&
                ( RNG_LL_STATE_ACTIVE == seedErrState )    )
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
 * Only \ref RNG_CONFIG_SELECTION_NIST is available, AIS-31 configurations are
 * not defined for STM32L4 / STM32L4+:
 * - STM32L4, STM32L4R / L4S: fixed (not configurable) post-processing, no
 *   register is changed.
 * - STM32L4P5 / L4Q5: CR configuration bits (RNG_CONFIG1 / 2 / 3, NISTC) and
 *   HTCR are written with the NIST values of the CMSIS device header with
 *   conditioning soft reset (CONDRST), the CR configuration bits and the
 *   CONDRST release are verified by read-back (HTCR is write-only for the
 *   module). Clock divider and clock error detection are not changed.
 *
 * \pre   STM32L4P5 / L4Q5: configuration shall not be locked (CONFIGLOCK);
 *        otherwise \ref RNG_REQUEST_ERROR and no register is changed.
 *
 * \param rngId           [in]: RNG peripheral ID
 * \param configSelection [in]: Configuration selection
 *
 * \return Returns \ref RNG_REQUEST_OK for \ref RNG_CONFIG_SELECTION_NIST
 *         applied without error. Otherwise returns \ref RNG_REQUEST_ERROR.
 */
rng_RequestState_t Rng_Set_Configuration( rng_PeriphId_t rngId, rng_ConfigSelection_t configSelection )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT            > rngId           ) &&
        ( RNG_CONFIG_SELECTION_NIST == configSelection )    )
    {
#if defined(RNG_CR_CONDRST)
        RNG_TypeDef * const rngReg    = rng_PeriphConf[ rngId ].RngReg;
        const uint32_t      lockState = LL_RNG_IsConfigLocked( rngReg );

        if( RNG_LL_STATE_INACTIVE == lockState )
        {
            /* Reference manual sequence: CONDRST with new CR values, HTCR (magic key and value), CONDRST release */
            MODIFY_REG( rngReg->CR, RNG_CR_CONFIG_MASK | RNG_CR_CONDRST, RNG_CR_NIST_CONFIG | RNG_CR_CONDRST );
            WRITE_REG( rngReg->HTCR, RNG_HTCFG_1 );
            WRITE_REG( rngReg->HTCR, RNG_HTCR_NIST_CONFIG );
            LL_RNG_DisableCondReset( rngReg );

            retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE );

            const uint32_t crConfig = READ_BIT( rngReg->CR, RNG_CR_CONFIG_MASK );

            if( ( RNG_REQUEST_OK     == retValue ) &&
                ( RNG_CR_NIST_CONFIG == crConfig )    )
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
#else
        /* Fixed hardware configuration */
        retValue = RNG_REQUEST_OK;
#endif /* RNG_CR_CONDRST */
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
 * STM32L4, STM32L4R / L4S: fixed configuration, \ref RNG_CONFIG_SELECTION_NIST
 * is always reported. STM32L4P5 / L4Q5: \ref RNG_CONFIG_SELECTION_NIST is
 * reported if the CR configuration bits hold the NIST values.
 *
 * \param rngId           [in] : RNG peripheral ID
 * \param configSelection [out]: Pointer where configuration selection will be stored. Must not be NULL.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error (STM32L4P5 / L4Q5: unknown configuration).
 */
rng_RequestState_t Rng_Get_Configuration( rng_PeriphId_t rngId, rng_ConfigSelection_t * const configSelection )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

    if( ( RNG_PERIPH_CNT > rngId           ) &&
        ( RNG_NULL_PTR  != configSelection )    )
    {
#if defined(RNG_CR_CONDRST)
        const uint32_t crConfig = READ_BIT( rng_PeriphConf[ rngId ].RngReg->CR, RNG_CR_CONFIG_MASK );

        if( RNG_CR_NIST_CONFIG == crConfig )
        {
            *configSelection = RNG_CONFIG_SELECTION_NIST;

            retValue = RNG_REQUEST_OK;
        }
        else
        {
            /* Unknown configuration */
            retValue = RNG_REQUEST_ERROR;
        }
#else
        *configSelection = RNG_CONFIG_SELECTION_NIST;

        retValue = RNG_REQUEST_OK;
#endif /* RNG_CR_CONDRST */
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
 * Clock error: CEIS is cleared, the generation continues automatically when
 * the clock is correct again (CECS cleared by hardware).
 *
 * Seed error (STM32L4, STM32L4R / L4S): SEIS is cleared and, if the seed error
 * is still active (SECS) and the generator is enabled, the generator is
 * restarted by clearing and setting RNGEN (reference manual recovery sequence).
 *
 * Seed error (STM32L4P5 / L4Q5): if the seed error is still active (SECS), the
 * conditioning logic is reset first (CONDRST), then SEIS is cleared. Without
 * active SECS the hardware has already recovered (automatic reset).
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

            const uint32_t clkErrState = LL_RNG_IsActiveFlag_CEIS( rngReg );

            if( RNG_LL_STATE_ACTIVE == clkErrState )
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
#if defined(RNG_CR_CONDRST)
            const uint32_t seedErrCurrent = LL_RNG_IsActiveFlag_SECS( rngReg );

            if( RNG_LL_STATE_ACTIVE == seedErrCurrent )
            {
                const rng_RequestState_t condRstState = Rng_Set_CondReset( rngId );

                if( RNG_REQUEST_OK != condRstState )
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
                /* Seed error is not active - recovered by automatic reset */
            }

            LL_RNG_ClearFlag_SEIS( rngReg );
#else
            LL_RNG_ClearFlag_SEIS( rngReg );

            const uint32_t seedErrCurrent = LL_RNG_IsActiveFlag_SECS( rngReg );
            const uint32_t genEnabled     = LL_RNG_IsEnabled( rngReg );

            if( ( RNG_LL_STATE_ACTIVE == seedErrCurrent ) &&
                ( RNG_LL_STATE_ACTIVE == genEnabled     )    )
            {
                const rng_RequestState_t restartState = Rng_Set_Restart( rngId );

                if( RNG_REQUEST_OK != restartState )
                {
                    retValue = RNG_REQUEST_ERROR;
                }
                else
                {
                    /* Generator restarted */
                }
            }
            else
            {
                /* Seed error is not active or generator is stopped */
            }
#endif /* RNG_CR_CONDRST */

            const uint32_t seedErrState = LL_RNG_IsActiveFlag_SEIS( rngReg );

            if( RNG_LL_STATE_ACTIVE == seedErrState )
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
 * \brief Activates clock error detection functionality (CED = 0).
 *
 * When clock error detection is enabled, the RNG checks that the clock
 * frequency is within the allowed range. If a clock error is detected,
 * the CEIS bit in the SR register is set and an interrupt is generated if
 * the IE bit in the CR register is set.
 *
 * \note  CED can be changed only with stopped generator - a running generator
 *        is stopped and restarted (STM32L4P5 / L4Q5: written with conditioning
 *        soft reset in addition).
 * \note  STM32L4 RNG (no CED bit): clock error detection is always active, the
 *        request is confirmed without register access.
 * \note  The RNG clock frequency must be high enough compared to HCLK,
 *        otherwise the clock checker flags a clock error.
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
#if defined(RNG_CR_CED)
        retValue = Rng_Set_ClkErrorDetect( rngId, RNG_FUNCTION_ACTIVE );
#else
        /* Clock error detection is always active */
        retValue = RNG_REQUEST_OK;
#endif /* RNG_CR_CED */
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates clock error detection functionality (CED = 1).
 *
 * When clock error detection is disabled, the RNG does not check that the
 * clock frequency is within the allowed range.
 *
 * \note  CED can be changed only with stopped generator - a running generator
 *        is stopped and restarted (STM32L4P5 / L4Q5: written with conditioning
 *        soft reset in addition).
 * \note  STM32L4 RNG (no CED bit): clock error detection can not be disabled,
 *        the request is rejected.
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
#if defined(RNG_CR_CED)
        retValue = Rng_Set_ClkErrorDetect( rngId, RNG_FUNCTION_INACTIVE );
#else
        /* Clock error detection can not be disabled */
        retValue = RNG_REQUEST_ERROR;
#endif /* RNG_CR_CED */
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
 * STM32L4 RNG (no CED bit): the state is always active.
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
#if defined(RNG_CR_CED)
        const uint32_t regValue = LL_RNG_IsEnabledClkErrorDetect( rng_PeriphConf[ rngId ].RngReg );

        if( RNG_LL_STATE_ACTIVE == regValue )
        {
            *state = RNG_FUNCTION_ACTIVE;
        }
        else
        {
            *state = RNG_FUNCTION_INACTIVE;
        }
#else
        /* Clock error detection is always active */
        *state = RNG_FUNCTION_ACTIVE;
#endif /* RNG_CR_CED */

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
 * STM32L4P5 / L4Q5 RNG resets itself after a seed error (no ARDIS bit - fixed
 * active state), the request is confirmed without register access.
 * STM32L4, STM32L4R / L4S RNG has no auto reset, the generator is restarted
 * by software (\ref Rng_Clear_Error) - the request is rejected.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return STM32L4P5 / L4Q5: \ref RNG_REQUEST_OK for a valid peripheral ID.
 *         Otherwise returns \ref RNG_REQUEST_ERROR (function is not supported).
 */
rng_RequestState_t Rng_Set_AutoResetActive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

#if defined(RNG_CR_CONDRST)
    if( RNG_PERIPH_CNT > rngId )
    {
        /* Auto reset is always active */
        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }
#else
    (void)rngId;

    /* Auto reset is not available */
    retValue = RNG_REQUEST_ERROR;
#endif /* RNG_CR_CONDRST */

    return ( retValue );
}


/**
 * \brief Deactivates auto reset functionality.
 *
 * STM32L4, STM32L4R / L4S RNG has no auto reset (fixed inactive state), the
 * request is confirmed without register access. Seed error recovery is done by
 * \ref Rng_Clear_Error. STM32L4P5 / L4Q5 RNG auto reset can not be disabled
 * (no ARDIS bit) - the request is rejected.
 *
 * \param rngId [in]: RNG peripheral ID
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
rng_RequestState_t Rng_Set_AutoResetInactive( rng_PeriphId_t rngId )
{
    rng_RequestState_t retValue = RNG_REQUEST_ERROR;

#if defined(RNG_CR_CONDRST)
    (void)rngId;

    /* Auto reset can not be disabled */
    retValue = RNG_REQUEST_ERROR;
#else
    if( RNG_PERIPH_CNT > rngId )
    {
        /* Auto reset is always inactive */
        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }
#endif /* RNG_CR_CONDRST */

    return ( retValue );
}


/**
 * \brief Returns state of auto reset functionality.
 *
 * STM32L4P5 / L4Q5: always active. STM32L4, STM32L4R / L4S: always inactive.
 *
 * \param rngId     [in]: RNG peripheral ID
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
#if defined(RNG_CR_CONDRST)
        *reqState = RNG_FUNCTION_ACTIVE;
#else
        *reqState = RNG_FUNCTION_INACTIVE;
#endif /* RNG_CR_CONDRST */

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
 * The peripheral clock of the previous source is released first - RCC module
 * does not change the kernel clock multiplexer of an active peripheral.
 *
 * \note  HSI48 and MSI are started by RCC module, PLL outputs Q shall be running.
 * \note  The 48 MHz clock multiplexer is shared with USB and SDMMC - RCC module
 *        refuses a different source while their clock is active.
 * \note  \ref rng_ClkSource_t contains the sources of the device line only (no
 *        HSI48 on STM32L47x / L48x, no PLLSAI1 output Q on STM32L41x / L42x).
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

    if( ( RNG_PERIPH_CNT      > rngId     ) &&
        ( RNG_CLK_SOURCE_CNT  > clkSource )    )
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

        if( RCC_REQUEST_OK == rccState )
        {
            rccState = Rcc_Set_PeriphActive( newRcc );
        }
        else
        {
            /* Clock of the previous source not released */
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
 * STM32L4P5 / L4Q5: CLKDIV is written with conditioning soft reset (CONDRST),
 * the CONDRST release and the divider are verified by read-back. Configuration
 * lock (CONFIGLOCK) blocks the change (read-back fails).
 *
 * STM32L4, STM32L4R / L4S: RNG has no clock divider, only \ref RNG_CLK_DIV_1
 * is accepted (no register access).
 *
 * \note  With clock error detection the RNG clock frequency before the
 *        divider is checked against HCLK.
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

#if defined(RNG_CR_CONDRST)
    if( ( RNG_PERIPH_CNT  > rngId      ) &&
        ( RNG_CLK_DIV_CNT > clkDivider )    )
    {
        RNG_TypeDef * const rngReg   = rng_PeriphConf[ rngId ].RngReg;
        const uint32_t      regValue = (uint32_t)clkDivider << RNG_CR_CLKDIV_Pos;

        /* LL function writes CLKDIV with CONDRST set and releases CONDRST */
        LL_RNG_SetClockDivider( rngReg, regValue );

        retValue = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE );

        if( RNG_REQUEST_OK == retValue )
        {
            retValue = Rng_Wait_RegState( rngId, LL_RNG_GetClockDivider, regValue );
        }
        else
        {
            /* Conditioning soft reset has not been released */
        }
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }
#else
    if( ( RNG_PERIPH_CNT > rngId      ) &&
        ( RNG_CLK_DIV_1 == clkDivider )    )
    {
        /* Clock is not divided */
        retValue = RNG_REQUEST_OK;
    }
    else
    {
        retValue = RNG_REQUEST_ERROR;
    }
#endif /* RNG_CR_CONDRST */

    return ( retValue );
}


/**
 * \brief Returns value of clock divider for Random Number Generator (RNG) peripheral.
 *
 * STM32L4, STM32L4R / L4S: RNG has no clock divider, \ref RNG_CLK_DIV_1 is
 * always reported.
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
#if defined(RNG_CR_CONDRST)
        const uint32_t regValue = LL_RNG_GetClockDivider( rng_PeriphConf[ rngId ].RngReg );

        *clkDivider = (rng_ClkDivider_t)( regValue >> RNG_CR_CLKDIV_Pos );
#else
        *clkDivider = RNG_CLK_DIV_1;
#endif /* RNG_CR_CONDRST */

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
 * \brief Waits until register (bit field) reaches required value.
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
        const uint32_t regValue = llGetter( rng_PeriphConf[ rngId ].RngReg );

        if( reqValue == regValue )
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


#if defined(RNG_CR_CONDRST)
/**
 * \brief Resets the conditioning logic after seed error (CONDRST 1 -> 0).
 *
 * \param rngId [in]: RNG peripheral ID, must be valid
 *
 * \return Returns \ref RNG_REQUEST_OK if CONDRST reads back released within
 *         timeout. Otherwise returns \ref RNG_REQUEST_ERROR.
 */
static rng_RequestState_t Rng_Set_CondReset( rng_PeriphId_t rngId )
{
    LL_RNG_EnableCondReset( rng_PeriphConf[ rngId ].RngReg );
    LL_RNG_DisableCondReset( rng_PeriphConf[ rngId ].RngReg );

    return ( Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE ) );
}
#else
/**
 * \brief Restarts the generator after seed error (RNGEN 1 -> 0 -> 1).
 *
 * \param rngId [in]: RNG peripheral ID, must be valid
 *
 * \return Returns \ref RNG_REQUEST_OK if the generator was restarted within
 *         timeout. Otherwise returns \ref RNG_REQUEST_ERROR.
 */
static rng_RequestState_t Rng_Set_Restart( rng_PeriphId_t rngId )
{
    LL_RNG_Disable( rng_PeriphConf[ rngId ].RngReg );

    rng_RequestState_t retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_INACTIVE );

    LL_RNG_Enable( rng_PeriphConf[ rngId ].RngReg );

    if( RNG_REQUEST_OK == retState )
    {
        retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_ACTIVE );
    }
    else
    {
        /* Generator has not been stopped */
    }

    return ( retState );
}
#endif /* RNG_CR_CONDRST */


#if defined(RNG_CR_CED)
/**
 * \brief Configures clock error detection (CED) with stopped generator.
 *
 * CED is taken into account only while RNGEN = 0 - a running generator is
 * stopped for the change and started again afterwards. STM32L4P5 / L4Q5: the
 * LL function writes CED with conditioning soft reset (CONDRST), the CONDRST
 * release is verified.
 *
 * \param rngId    [in]: RNG peripheral ID, must be valid
 * \param cedState [in]: \ref RNG_FUNCTION_ACTIVE - clock error detection enabled
 *
 * \return Returns \ref RNG_REQUEST_OK if CED reads back the requested state and
 *         a running generator was restarted within timeout. Otherwise returns
 *         \ref RNG_REQUEST_ERROR.
 */
static rng_RequestState_t Rng_Set_ClkErrorDetect( rng_PeriphId_t rngId, rng_FunctionState_t cedState )
{
    RNG_TypeDef * const rngReg     = rng_PeriphConf[ rngId ].RngReg;
    const uint32_t      genEnabled = LL_RNG_IsEnabled( rngReg );
    rng_RequestState_t  retState   = RNG_REQUEST_OK;

    if( RNG_LL_STATE_ACTIVE == genEnabled )
    {
        LL_RNG_Disable( rngReg );

        retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_INACTIVE );
    }
    else
    {
        /* Generator is stopped */
    }

    if( RNG_REQUEST_OK == retState )
    {
        if( RNG_FUNCTION_ACTIVE == cedState )
        {
            LL_RNG_EnableClkErrorDetect( rngReg );

            retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledClkErrorDetect, RNG_LL_STATE_ACTIVE );
        }
        else
        {
            LL_RNG_DisableClkErrorDetect( rngReg );

            retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledClkErrorDetect, RNG_LL_STATE_INACTIVE );
        }
    }
    else
    {
        /* Generator has not been stopped, CED is not changed */
    }

#if defined(RNG_CR_CONDRST)
    if( RNG_REQUEST_OK == retState )
    {
        retState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabledCondReset, RNG_LL_STATE_INACTIVE );
    }
    else
    {
        /* CED change failed */
    }
#endif /* RNG_CR_CONDRST */

    if( RNG_LL_STATE_ACTIVE == genEnabled )
    {
        /* Running generator is started again also after a failed change */
        LL_RNG_Enable( rngReg );

        const rng_RequestState_t restartState = Rng_Wait_RegState( rngId, LL_RNG_IsEnabled, RNG_LL_STATE_ACTIVE );

        if( RNG_REQUEST_OK != restartState )
        {
            retState = RNG_REQUEST_ERROR;
        }
        else
        {
            /* Generator is running again */
        }
    }
    else
    {
        /* Generator was stopped before the change */
    }

    return ( retState );
}
#endif /* RNG_CR_CED */

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief RNG interrupt handler.
 *
 * Errors are cleared and reported by error callback (data are not read). Data
 * ready: one block is read; it is reported by data ready callback only while
 * the generation is running, otherwise it is discarded.
 */
static void Rng_IsrHandler( void )
{
    const rng_PeriphId_t rngId      = RNG_PERIPH_1;
    const uint32_t       readyState = LL_RNG_IsActiveFlag_DRDY( rng_PeriphConf[ rngId ].RngReg );
    rng_Error_t          errMask    = RNG_ERROR_NONE;

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
    else if( RNG_LL_STATE_ACTIVE == readyState )
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
    __DSB();    /* Cortex-M4 r0p1 erratum 838869: stores completed before the exception return */
}

/* ================================ TASKS =================================== */

#endif /* RNG */
