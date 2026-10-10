/**
 * \author Mr.Nobody
 * \file Test_Rng.c
 * \ingroup Rng
 * \brief Unit tests of Random Number Generator (RNG) module.
 *
 * Rng.c is compiled unchanged with real LL drivers. RNG registers are emulated
 * by RegMem, RCC and NVIC modules are mocked by CMock.
 *
 * \note Emulated registers are plain RAM: CONDRST is cleared by the module write
 *       (HW clears it after the conditioning reset), status flags (DRDY, SEIS,
 *       CEIS, SECS) are preset by tests. Flags are cleared by writing inverted
 *       mask (rc_w0), emulated SR therefore contains the remaining bits set.
 *
 * \note Tests depend on RNG version of the device: RNG with conditioning soft
 *       reset (RNG_CR_CONDRST - STM32H72x / STM32H73x / STM32H7A3 / STM32H7B0 /
 *       STM32H7B3) or RNG without configuration (STM32H74x / STM32H75x).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Rng_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "Stm32_rng.h"                      /* RNG registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_Rng_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static void                 Ut_Rng_DataRdyCallback      ( rng_RandomData_t value );
static void                 Ut_Rng_ErrorCallback        ( rng_Error_t errMask );
static rng_ConfigStruct_t   Ut_Rng_Get_Config           ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Init                 ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Init_Running         ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Expect_PrevClock     ( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState );
static void                 Ut_Rng_Expect_LsiState      ( rcc_FunctionState_t lsiState );
static void                 Ut_Rng_Release              ( void );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by tests */
#define UT_RNG_IRQ_PRIO                     ( 3u )

/** Random value preset in data register */
#define UT_RNG_DATA                         ( 0xA5C3E1F7u )

#if defined(STM32H7) && \
    defined(HASH)
/** RNG interrupt shared with HASH peripheral */
#define UT_RNG_NVIC_IRQ                     ( NVIC_PERIPH_IRQ_HASH_RNG )
#else
/** RNG interrupt */
#define UT_RNG_NVIC_IRQ                     ( NVIC_PERIPH_IRQ_RNG )
#endif /* STM32H7 && HASH */

#if defined(RNG_CR_CONDRST)
/** CR register bits selecting the conditioning configuration */
#define UT_RNG_CR_CONFIG_MASK               ( RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_NISTC )

#if defined(STM32H7RS)
/** Expected HTCR value of NIST configuration - STM32H7R / STM32H7S (Zephyr stm32h7rs.dtsi) */
#define UT_RNG_HTCR_NIST                    ( 0x0000AAC7u )
#elif defined(RNG_VER_3_2)
/** Expected HTCR value of NIST configuration - RNG version 3.2 (AN4230, STM32H7 HAL RNG_HTCFG) */
#define UT_RNG_HTCR_NIST                    ( 0x00007274u )
#else
/** Expected HTCR value of NIST configuration - RNG version 3.1 (AN4230, STM32H7 HAL RNG_HTCFG) */
#define UT_RNG_HTCR_NIST                    ( 0x000CAA74u )
#endif /* STM32H7RS */
#endif /* RNG_CR_CONDRST */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC */
static nvic_IsrCallback_t   utRng_Isr;

/** Count of data ready callback calls */
static uint32_t             utRng_DataCnt;

/** Data passed to the last data ready callback */
static rng_RandomData_t     utRng_LastData;

/** Count of error callback calls */
static uint32_t             utRng_ErrorCnt;

/** Error mask passed to the last error callback */
static rng_Error_t          utRng_LastError;

/** Data ready callback pauses the delivery (non-zero) */
static uint8_t              utRng_PauseInCallback;

/** Result of the pause requested by the data ready callback */
static rng_RequestState_t   utRng_PauseResult;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Rng_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utRng_Isr       = NULL;
    utRng_DataCnt   = 0u;
    utRng_ErrorCnt  = 0u;
    utRng_LastError = RNG_ERROR_NONE;

    utRng_PauseInCallback = 0u;
    utRng_PauseResult     = RNG_REQUEST_ERROR;

    for( uint32_t index = 0u; RNG_FIFO_LEN > index; index ++ )
    {
        utRng_LastData.value[ index ] = 0u;
    }

    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Rng_NvicSetHandlerStub );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Rng_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Rng_Get_ModuleVersion_ReturnsVersion( void )
{
    rng_ModuleVersion_t version = Rng_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Rng_Get_DefaultConfig() fills default configuration.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, peripheral 1, HSI48, NIST, blocking, priority 0, no callbacks.
 * - NULL pointer: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_DefaultConfig_ReturnsDefaults( void )
{
    rng_ConfigStruct_t config;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( RNG_PERIPH_1,              config.PeriphId );
    TEST_ASSERT_EQUAL( RNG_CLK_SOURCE_HSI48,      config.ClkSource );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, config.ConfigSelection );
    TEST_ASSERT_EQUAL( RNG_TRANSFER_BLOCKING,     config.TransferStyle );
    TEST_ASSERT_EQUAL_UINT32( 0u,                 config.IrqPrio );
    TEST_ASSERT_NULL( config.DataRdyCallback );
    TEST_ASSERT_NULL( config.ErrorCallback );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_DefaultConfig( NULL ) );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Rng_Init() with blocking style stops the generator and configures NIST conditioning.
 *
 * \details Initializes with default configuration (HSI48, NIST, blocking), state
 *          of previous initialization (RNGEN, IE) is preset.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, RCC clock HSI48 activated, NVIC IRQ deactivated.
 * - RNG with CONDRST: CR configuration bits hold hardware default NIST values (0),
 *   HTCR holds NIST value (AN4230), CONDRST released.
 * - Generator and peripheral interrupt disabled, no ISR registered.
 */
void Ut_Rng_Init_Blocking_ConfiguresNist( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

#if defined(RNG_CR_CONDRST)
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE | RNG_CR_NISTC;    /* State of previous initialization */
#else
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;                   /* State of previous initialization */
#endif /* RNG_CR_CONDRST */

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR & UT_RNG_CR_CONFIG_MASK );
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_HTCR_NIST, RNG->HTCR );
    TEST_ASSERT_BITS_LOW( RNG_CR_CONDRST, RNG->CR );
#endif /* RNG_CR_CONDRST */
    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
    TEST_ASSERT_NULL( utRng_Isr );
}


/**
 * \brief   Rng_Init() with interrupt style configures NVIC and peripheral interrupt.
 *
 * \par Expected results
 * - NVIC priority, handler and activation requested, peripheral IE set.
 */
void Ut_Rng_Init_Interrupt_ActivatesIrq( void )
{
    Ut_Rng_Init( RNG_TRANSFER_INTERRUPT );

    TEST_ASSERT_NOT_NULL( utRng_Isr );
    TEST_ASSERT_BITS_HIGH( RNG_CR_IE, RNG->CR );
    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN, RNG->CR );
}


/**
 * \brief   Rng_Init() selects required kernel clock source.
 *
 * \par Expected results
 * - RCC activation of RNG with LSE clock source.
 * - STM32H7R / STM32H7S (HSI48 only, LSE is not in the list): source out of the list, RNG_REQUEST_ERROR, no RCC call.
 */
void Ut_Rng_Init_LseClkSource_ActivatesLseClock( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

#if !defined(STM32H7RS)
    config.ClkSource = RNG_CLK_SOURCE_LSE;

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_HSI48, RCC_FUNCTION_INACTIVE );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_LSE, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );
#else
    TEST_ASSERT_EQUAL_UINT32( 1u, (uint32_t)RNG_CLK_SOURCE_CNT );
    config.ClkSource = RNG_CLK_SOURCE_CNT;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );
#endif /* STM32H7RS */
}


/**
 * \brief   Rng_Init() rejects invalid configuration without HW access.
 *
 * \details NULL pointer, invalid peripheral, transfer style and clock source.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, no RCC / NVIC call, registers untouched.
 */
void Ut_Rng_Init_InvalidConfig_ReturnsError( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( NULL ) );

    config.PeriphId = RNG_PERIPH_CNT;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    config = Ut_Rng_Get_Config( RNG_TRANSFER_CNT );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );
    config.ClkSource = RNG_CLK_SOURCE_CNT;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->HTCR );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Init() fails when the clock is not activated.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, registers untouched (interrupt deactivated before the clock).
 */
void Ut_Rng_Init_RccFailure_ReturnsError( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
}


/**
 * \brief   Rng_Init() rejects AIS-31 configuration (not available on STM32H7).
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, callbacks are not stored, interrupt is not configured.
 * - RNG with CONDRST: configuration registers untouched.
 */
void Ut_Rng_Init_Ais31Config_ReturnsError( void )
{
    rng_ConfigStruct_t     config    = Ut_Rng_Get_Config( RNG_TRANSFER_INTERRUPT );
    rng_DataRdyCallback_t *dataRdyCb = Ut_Rng_DataRdyCallback;

    config.ConfigSelection = RNG_CONFIG_SELECTION_AIS_31_SPEED;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_DataRdyCallback( RNG_PERIPH_1, NULL ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataRdyCallback( RNG_PERIPH_1, &dataRdyCb ) );
    TEST_ASSERT_NULL( dataRdyCb );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->HTCR );
#endif /* RNG_CR_CONDRST */
    TEST_ASSERT_NULL( utRng_Isr );
}


/**
 * \brief   Rng_Init() fails when the configuration is locked.
 *
 * \details CONFIGLOCK is preset (configuration locked until system reset).
 *          Ignored on RNG without configuration lock (STM32H74x / STM32H75x).
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, CR configuration bits and HTCR untouched.
 */
void Ut_Rng_Init_ConfigLocked_ReturnsError( void )
{
#if defined(RNG_CR_CONDRST)
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    RNG->CR = RNG_CR_CONFIGLOCK | RNG_CR_NISTC;

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( RNG_CR_NISTC, RNG->CR & UT_RNG_CR_CONFIG_MASK );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->HTCR );
#else
    TEST_IGNORE_MESSAGE( "RNG without configuration lock" );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Deinit() releases interrupt, generator, reset and clock.
 *
 * \details Initializes with LSE clock (STM32H7R / STM32H7S: HSI48) and interrupt
 *          style, starts the generator and deinitializes the module.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, NVIC deactivated, RCC reset pulse and clock disable of the
 *   used clock source (LSE / HSI48), RNGEN and IE cleared, callbacks removed.
 * - NULL pointer / invalid peripheral: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Deinit_ReleasesResources( void )
{
    rng_ConfigStruct_t     config      = Ut_Rng_Get_Config( RNG_TRANSFER_INTERRUPT );
    rng_DataRdyCallback_t *dataRdyCb   = Ut_Rng_DataRdyCallback;
    rng_ErrCallback_t     *errorCb     = Ut_Rng_ErrorCallback;
#if defined(LL_RCC_RNG_CLKSOURCE_LSE)
    const rcc_PeriphId_t   clkRcc      = RCC_PERIPH_RNG_LSE;

    config.ClkSource = RNG_CLK_SOURCE_LSE;
#else
    /* STM32H7R / STM32H7S: HSI48 is the only kernel clock */
    const rcc_PeriphId_t   clkRcc      = RCC_PERIPH_RNG_HSI48;
#endif /* LL_RCC_RNG_CLKSOURCE_LSE */

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
#if defined(LL_RCC_RNG_CLKSOURCE_LSE)
    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_HSI48, RCC_FUNCTION_INACTIVE );
#endif /* LL_RCC_RNG_CLKSOURCE_LSE */
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( clkRcc, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_RNG_NVIC_IRQ, UT_RNG_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( clkRcc, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( clkRcc, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( clkRcc, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Deinit( &config ) );

    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataRdyCallback( RNG_PERIPH_1, &dataRdyCb ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ErrorCallback( RNG_PERIPH_1, &errorCb ) );
    TEST_ASSERT_NULL( dataRdyCb );
    TEST_ASSERT_NULL( errorCb );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Deinit( NULL ) );
    config.PeriphId = RNG_PERIPH_CNT;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Deinit( &config ) );
}


/**
 * \brief   Rng_Deinit() reports RCC failure.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, all release steps are still executed.
 */
void Ut_Rng_Deinit_RccFailure_ReturnsError( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    Ut_Rng_Init( RNG_TRANSFER_BLOCKING );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Deinit( &config ) );
}

/* ============================ CONFIGURATION =============================== */

/**
 * \brief   Rng_Get_Configuration() identifies NIST configuration.
 *
 * \details RNG with CONDRST: initializes the module (NIST), then sets NISTC (custom
 *          configuration). STM32H74x / STM32H75x: fixed configuration is set and read.
 *
 * \par Expected results
 * - NIST configuration read back.
 * - RNG with CONDRST: unknown CR configuration bits: RNG_REQUEST_ERROR.
 * - STM32H74x / STM32H75x: registers untouched.
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_Configuration_IdentifiesNist( void )
{
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;

#if defined(RNG_CR_CONDRST)
    Ut_Rng_Init( RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );

    RNG->CR |= RNG_CR_NISTC;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
#else
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_NIST ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_CNT, &selection ) );
}


/**
 * \brief   Rng_Set_Configuration() keeps divider and control bits.
 *
 * \details Presets clock divider, CED, IE and NISTC, then sets NIST configuration.
 *          Ignored on RNG without configuration (STM32H74x / STM32H75x).
 *
 * \par Expected results
 * - RNG_REQUEST_OK, NISTC cleared, preset control bits unchanged, CONDRST released.
 */
void Ut_Rng_Set_Configuration_KeepsControlBits( void )
{
#if defined(RNG_CR_CONDRST)
    const uint32_t controlBits = LL_RNG_CLKDIV_BY_4 | RNG_CR_CED | RNG_CR_IE;

    RNG->CR = controlBits | RNG_CR_NISTC;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_NIST ) );

    TEST_ASSERT_EQUAL_HEX32( controlBits, RNG->CR );
#else
    TEST_IGNORE_MESSAGE( "RNG without configuration" );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Set_Configuration() rejects AIS-31 configurations.
 *
 * \details AIS-31 configuration values are not defined for STM32H7 RNG versions.
 *
 * \par Expected results
 * - AIS-31: RNG_REQUEST_ERROR, registers untouched.
 * - Invalid selection / ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_Configuration_Ais31_NotSupported( void )
{
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_SPEED ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_ENTROPY ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->HTCR );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_CNT, RNG_CONFIG_SELECTION_NIST ) );
}


/**
 * \brief   Rng_Set_ClkSource() activates RCC clock of required source.
 *
 * \details Sources are changed one after another, the clock of the previous source is
 *          reported active (except the last change).
 * \note    Bug AB#955: the clock of the previous source was not released - RCC refused
 *          the change of the kernel clock multiplexer of the active peripheral.
 *
 * \par Expected results
 * - Every source maps to its RCC peripheral ID, RNG_REQUEST_OK.
 * - Active clock of the previous source released before the new one is activated,
 *   same source or inactive previous clock: no release.
 * - Release failure / RCC failure: RNG_REQUEST_ERROR. Invalid source / ID:
 *   RNG_REQUEST_ERROR without RCC call.
 * - STM32H7R / STM32H7S: HSI48 accepted, it is the only item of the list.
 */
void Ut_Rng_Set_ClkSource_ActivatesRccClock( void )
{
#if defined(LL_RCC_RNG_CLKSOURCE_LSI)
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_HSI48, RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_PLL1Q, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_PLL1Q ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_PLL1Q, RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_RNG_PLL1Q, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_LSE ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_PLL1Q, RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_RNG_PLL1Q, RCC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_LSE, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_LSE ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_LSE, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_LSI, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_LSI ) );
#else
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 ) );

    /* HSI48 is the only item of the list */
    TEST_ASSERT_EQUAL_UINT32( 1u, (uint32_t)RNG_CLK_SOURCE_CNT );
#endif /* LL_RCC_RNG_CLKSOURCE_LSI */

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_CNT, RNG_CLK_SOURCE_HSI48 ) );
}


/**
 * \brief   Rng_Set_ClkSource() completes a kernel clock switch pending from LSI.
 *
 * \details LSI is reported inactive when HSI48 is selected, then LSI start fails.
 * \note    Bug AB#954: after a system reset with LSI as kernel clock (LSI stopped by
 *          the reset) the multiplexer waits for LSI clock - CONDRST was never released
 *          with HSI48 (NUCLEO-H503RB) until LSI ran again.
 *
 * \par Expected results
 * - LSI inactive: LSI started and stopped before the HSI48 clock is activated, RNG_REQUEST_OK.
 * - LSI start fails: RNG_REQUEST_ERROR, HSI48 clock not activated.
 * - LSI selected: LSI state not read.
 * - STM32H7R / STM32H7S: ignored (no RNG kernel clock multiplexer).
 */
void Ut_Rng_Set_ClkSource_LsiStopped_LsiStartedAndStopped( void )
{
#if defined(LL_RCC_RNG_CLKSOURCE_LSI)
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_INACTIVE );
    Rcc_Set_OscActive_ExpectAndReturn( RCC_OSC_LSI, RCC_REQUEST_OK );
    Rcc_Set_OscInactive_ExpectAndReturn( RCC_OSC_LSI, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 ) );

    Ut_Rng_Expect_LsiState( RCC_FUNCTION_INACTIVE );
    Rcc_Set_OscActive_ExpectAndReturn( RCC_OSC_LSI, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_HSI48, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_LSI, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_LSI ) );
#else
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: no RNG kernel clock multiplexer (HSI48 only)" );
#endif /* LL_RCC_RNG_CLKSOURCE_LSI */
}


/**
 * \brief   Rng_Set_ClkDivider() / Rng_Get_ClkDivider() round trip.
 *
 * \par Expected results
 * - RNG with CONDRST: divider 1, 4 and 32768 written to CR.CLKDIV and read back,
 *   CONDRST released.
 * - STM32H74x / STM32H75x (no divider): divider 1 accepted and read back, other
 *   divider rejected, registers untouched.
 * - Invalid divider / ID / NULL: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_ClkDivider_RoundTrip( void )
{
    rng_ClkDivider_t divider = RNG_CLK_DIV_CNT;

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_4 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_RNG_CLKDIV_BY_4, RNG->CR & RNG_CR_CLKDIV );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_4, divider );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_32768 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_RNG_CLKDIV_BY_32768, RNG->CR & RNG_CR_CLKDIV );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_32768, divider );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_RNG_CLKDIV_BY_1, RNG->CR & RNG_CR_CLKDIV );
    TEST_ASSERT_BITS_LOW( RNG_CR_CONDRST, RNG->CR );
#else
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );

    /* RNG_CLK_DIV_1 is the only item of the list */
    TEST_ASSERT_EQUAL_UINT32( 1u, (uint32_t)RNG_CLK_DIV_CNT );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkDivider( RNG_PERIPH_CNT, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClkDivider( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClkDivider( RNG_PERIPH_CNT, &divider ) );
}


/**
 * \brief   Clock error detection activation and state with stopped generator.
 *
 * \par Expected results
 * - Inactive: CED set, state inactive, generator stays stopped.
 * - Active: CED cleared (detection enabled), state active, generator stays stopped.
 * - CONDRST released (RNG with CONDRST).
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_ClockErrorDetect_ControlsCedBit( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CED, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClockErrorDetectState( RNG_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_CNT ) );
}


/**
 * \brief   Clock error detection change keeps a running generator running.
 *
 * \details Presets RNGEN | IE (running generator with interrupt), changes CED.
 *
 * \note    STM32H74x / STM32H75x: CED is taken into account only with RNGEN = 0 -
 *          the module stops the generator for the change and restarts it. RNG with
 *          CONDRST: CED is written with conditioning soft reset. Emulated register
 *          keeps the final state only.
 *
 * \par Expected results
 * - Inactive: RNG_REQUEST_OK, CED set, RNGEN and IE set.
 * - Active: RNG_REQUEST_OK, CED cleared, RNGEN and IE set.
 */
void Ut_Rng_Set_ClockErrorDetect_Running_GeneratorRunsAfterChange( void )
{
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE | RNG_CR_CED, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
}


/**
 * \brief   Auto reset is not available (always inactive, no ARDIS bit on STM32H7).
 *
 * \par Expected results
 * - Active: RNG_REQUEST_ERROR, Inactive: RNG_REQUEST_OK, state always inactive.
 * - Registers untouched. NULL / invalid ID: RNG_REQUEST_ERROR.
 * - STM32H7R / STM32H7S: ignored (ARDIS available, see
 *   Ut_Rng_Set_AutoReset_ControlsArdis).
 */
void Ut_Rng_Set_AutoReset_AlwaysInactive( void )
{
#if defined(RNG_CR_ARDIS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: auto reset (ARDIS) available" );
#else
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_AutoResetInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetInactive( RNG_PERIPH_CNT ) );
#endif /* RNG_CR_ARDIS */
}


/**
 * \brief   Auto reset functions control ARDIS (STM32H7R / STM32H7S).
 *
 * \par Expected results
 * - Inactive: ARDIS set, state inactive. Active: ARDIS cleared, state active.
 * - CONDRST released, other CR bits kept. NULL / invalid ID: RNG_REQUEST_ERROR.
 * - Classic STM32H7 lines: ignored (no ARDIS bit).
 */
void Ut_Rng_Set_AutoReset_ControlsArdis( void )
{
#if defined(RNG_CR_ARDIS)
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    RNG->CR = RNG_CR_CED;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_AutoResetInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CED | RNG_CR_ARDIS, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CED, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetInactive( RNG_PERIPH_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "Classic STM32H7: no auto reset (ARDIS) bit" );
#endif /* RNG_CR_ARDIS */
}

/* ============================== EXECUTION ================================= */

/**
 * \brief   Rng_Set_ExecActive() / Rng_Set_ExecInactive() control the generator.
 *
 * \par Expected results
 * - Active: RNGEN set, execution state active.
 * - Inactive: RNGEN cleared, execution state inactive. Invalid ID / NULL: error.
 */
void Ut_Rng_Set_Exec_ControlsGenerator( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    Ut_Rng_Init( RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_HIGH( RNG_CR_RNGEN, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecInactive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ExecState( RNG_PERIPH_1, NULL ) );
}


/**
 * \brief   Pause in blocking style changes only the execution state.
 *
 * \par Expected results
 * - Pause without running generation: RNG_REQUEST_ERROR.
 * - Pause: pause state active, execution state still active, RNGEN kept.
 * - Second pause: RNG_REQUEST_ERROR. Resume: pause state inactive, second resume error.
 */
void Ut_Rng_Set_ExecPause_Blocking_ChangesState( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_INACTIVE;

    Ut_Rng_Init( RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecPauseState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_BITS_HIGH( RNG_CR_RNGEN, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecPauseState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ExecPauseState( RNG_PERIPH_1, NULL ) );
}


/**
 * \brief   Pause in interrupt style masks the peripheral interrupt.
 *
 * \par Expected results
 * - Pause: IE cleared. Resume: IE set again.
 */
void Ut_Rng_Set_ExecPause_Interrupt_MasksIrq( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_LOW( RNG_CR_IE, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_HIGH( RNG_CR_IE, RNG->CR );
}


/**
 * \brief   Rng_Set_ExecActive() keeps the generation inactive when the enable is not confirmed.
 *
 * \details CONDRST preset (conditioning reset in progress) - RNGEN is never confirmed.
 * \note    Bug AB#587: the execution state is set before the generator is enabled
 *          (data ready interrupt follows immediately), it is restored when the
 *          enable fails.
 *
 * \note    Ignored on RNG without CONDRST (STM32H74x / STM32H75x) - the emulated
 *          register confirms every write.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, execution state inactive.
 */
void Ut_Rng_Set_ExecActive_EnableNotConfirmed_StaysInactive( void )
{
#if defined(RNG_CR_CONDRST)
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    Ut_Rng_Init( RNG_TRANSFER_INTERRUPT );

    RNG->CR |= RNG_CR_CONDRST;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
#else
    TEST_IGNORE_MESSAGE( "RNG without conditioning soft reset" );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Set_ExecPauseInactive() keeps the pause when the interrupt enable is not confirmed.
 *
 * \details Interrupt style running and paused, CONDRST preset - IE is never confirmed.
 * \note    Bug AB#587: the execution state is set before the interrupt is enabled,
 *          it is restored to paused when the enable fails.
 *
 * \note    Ignored on RNG without CONDRST (STM32H74x / STM32H75x) - the emulated
 *          register confirms every write.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, pause state still active.
 */
void Ut_Rng_Set_ExecPauseInactive_IrqEnableNotConfirmed_StaysPaused( void )
{
#if defined(RNG_CR_CONDRST)
    rng_FunctionState_t state = RNG_FUNCTION_INACTIVE;

    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );

    RNG->CR |= RNG_CR_CONDRST;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecPauseState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
#else
    TEST_IGNORE_MESSAGE( "RNG without conditioning soft reset" );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Data ready callback pauses the delivery in interrupt style.
 *
 * \details Callback calls Rng_Set_ExecPauseActive() from the ISR.
 * \note    Bug AB#587: the application stops the continuous data ready interrupt
 *          from the callback, the delivery is resumed by Rng_Set_ExecPauseInactive().
 *
 * \par Expected results
 * - One block delivered, pause from the callback OK, IE cleared, pause state active.
 * - Second ISR call delivers nothing. Resume OK, IE set, next ISR delivers data.
 */
void Ut_Rng_Isr_PauseFromCallback_StopsDelivery( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_INACTIVE;

    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    utRng_PauseInCallback = 1u;

    RNG->SR = RNG_SR_DRDY;
    RNG->DR = UT_RNG_DATA;

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_DataCnt );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, utRng_PauseResult );
    TEST_ASSERT_BITS_LOW( RNG_CR_IE, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecPauseState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_DataCnt );

    utRng_PauseInCallback = 0u;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_HIGH( RNG_CR_IE, RNG->CR );

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 2u, utRng_DataCnt );
}

/* ================================= DATA =================================== */

/**
 * \brief   Rng_Get_Data() reads one block of 4 words.
 *
 * \details Presets DRDY and data register value.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, all 4 words hold the data register value.
 * - Data ready state active.
 */
void Ut_Rng_Get_Data_DataReady_ReadsBlock( void )
{
    rng_RandomData_t    data  = { { 0u } };
    rng_FunctionState_t state = RNG_FUNCTION_INACTIVE;

    RNG->SR = RNG_SR_DRDY;
    RNG->DR = UT_RNG_DATA;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataReadyState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Data( RNG_PERIPH_1, &data ) );

    for( uint32_t index = 0u; RNG_FIFO_LEN > index; index ++ )
    {
        TEST_ASSERT_EQUAL_HEX32( UT_RNG_DATA, data.value[ index ] );
    }
}


/**
 * \brief   Rng_Get_Data() rejects missing or invalid data.
 *
 * \par Expected results
 * - No DRDY: RNG_REQUEST_ERROR, data ready state inactive.
 * - DRDY with seed error (SEIS): RNG_REQUEST_ERROR.
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_Data_NotReadyOrSeedError_ReturnsError( void )
{
    rng_RandomData_t    data  = { { 0u } };
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Data( RNG_PERIPH_1, &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataReadyState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    RNG->SR = RNG_SR_DRDY | RNG_SR_SEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Data( RNG_PERIPH_1, &data ) );

    RNG->SR = RNG_SR_DRDY;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Data( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Data( RNG_PERIPH_CNT, &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_DataReadyState( RNG_PERIPH_1, NULL ) );
}

/* ================================ ERRORS ================================== */

/**
 * \brief   Rng_Get_Error() maps status flags to error mask.
 *
 * \par Expected results
 * - No flag: NONE, SEIS: SEED_ERR, CEIS: CLOCK_ERR, both: ALL.
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_Error_MapsFlags( void )
{
    rng_Error_t errMask = RNG_ERROR_CNT;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );

    RNG->SR = RNG_SR_SEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_SEED_ERR, errMask );

    RNG->SR = RNG_SR_CEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_CLOCK_ERR, errMask );

    RNG->SR = RNG_SR_CEIS | RNG_SR_SEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_ALL, errMask );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Error( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Error( RNG_PERIPH_CNT, &errMask ) );
}


/**
 * \brief   Rng_Clear_Error() clears requested flags.
 *
 * \par Expected results
 * - CLOCK_ERR: CEIS cleared. SEED_ERR: SEIS cleared.
 * - Invalid mask / ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Clear_Error_ClearsFlags( void )
{
    RNG->SR = RNG_SR_CEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_CLOCK_ERR ) );
    TEST_ASSERT_BITS_LOW( RNG_SR_CEIS, RNG->SR );

    RNG->SR = RNG_SR_SEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_SEED_ERR ) );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Clear_Error( RNG_PERIPH_CNT, RNG_ERROR_ALL ) );
}


/**
 * \brief   Rng_Clear_Error() recovers seed error of a running generator.
 *
 * \details Generator running, SECS and SEIS preset (active seed error).
 *
 * \note    RNG with CONDRST: conditioning soft reset (CONDRST 1 -> 0) before SEIS
 *          is cleared. STM32H74x / STM32H75x: SEIS cleared, generator restarted
 *          (RNGEN 1 -> 0 -> 1). Emulated register keeps the final state only.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, SEIS cleared, generator and interrupt enabled, CONDRST released.
 */
void Ut_Rng_Clear_Error_SeedRunning_RecoversGenerator( void )
{
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;
    RNG->SR = RNG_SR_SECS | RNG_SR_SEIS;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_SEED_ERR ) );

    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
}


/**
 * \brief   Rng_Clear_Error() keeps stopped generator stopped after seed error.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, SEIS cleared, RNGEN stays low, CONDRST released.
 */
void Ut_Rng_Clear_Error_SeedStopped_KeepsGeneratorStopped( void )
{
    RNG->SR = RNG_SR_SECS | RNG_SR_SEIS;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_SEED_ERR ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
}

/* =============================== CALLBACKS ================================ */

/**
 * \brief   Callback setters and getters.
 *
 * \par Expected results
 * - Set callbacks read back, NULL is accepted as callback.
 * - NULL output pointer / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_Callbacks_RoundTrip( void )
{
    rng_DataRdyCallback_t *dataRdyCb = NULL;
    rng_ErrCallback_t     *errorCb   = NULL;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_DataRdyCallback( RNG_PERIPH_1, Ut_Rng_DataRdyCallback ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ErrorCallback( RNG_PERIPH_1, Ut_Rng_ErrorCallback ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataRdyCallback( RNG_PERIPH_1, &dataRdyCb ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ErrorCallback( RNG_PERIPH_1, &errorCb ) );
    TEST_ASSERT_EQUAL_PTR( Ut_Rng_DataRdyCallback, dataRdyCb );
    TEST_ASSERT_EQUAL_PTR( Ut_Rng_ErrorCallback, errorCb );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_DataRdyCallback( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataRdyCallback( RNG_PERIPH_1, &dataRdyCb ) );
    TEST_ASSERT_NULL( dataRdyCb );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_DataRdyCallback( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ErrorCallback( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_DataRdyCallback( RNG_PERIPH_CNT, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ErrorCallback( RNG_PERIPH_CNT, NULL ) );
}

/* ============================== INTERRUPTS ================================ */

/**
 * \brief   Rng_Set_IrqActive() / Rng_Set_IrqInactive() control NVIC and IE.
 *
 * \par Expected results
 * - Active: handler registered, NVIC activated, IE set, state active.
 * - Inactive: NVIC deactivated, IE cleared, state inactive.
 * - NVIC activation failure: RNG_REQUEST_ERROR, IE not set.
 */
void Ut_Rng_Set_Irq_ControlsNvicAndIe( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_INACTIVE;

    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_IrqActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_NOT_NULL( utRng_Isr );
    TEST_ASSERT_BITS_HIGH( RNG_CR_IE, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_IrqInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_LOW( RNG_CR_IE, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_IrqActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_BITS_LOW( RNG_CR_IE, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_IrqActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_IrqInactive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_IrqState( RNG_PERIPH_1, NULL ) );
}


/**
 * \brief   Interrupt priority is forwarded to NVIC module.
 *
 * \par Expected results
 * - Set / get call NVIC with RNG IRQ, NVIC result is returned.
 * - NULL / invalid ID: RNG_REQUEST_ERROR without NVIC call.
 */
void Ut_Rng_Set_IrqPriority_ForwardsToNvic( void )
{
    static nvic_IrqPrio_t nvicPrio = UT_RNG_IRQ_PRIO;
    rng_IrqPrio_t         irqPrio  = 0u;

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_RNG_NVIC_IRQ, UT_RNG_IRQ_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_IrqPriority( RNG_PERIPH_1, UT_RNG_IRQ_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_RNG_NVIC_IRQ, 99u, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_IrqPriority( RNG_PERIPH_1, 99u ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_RNG_NVIC_IRQ, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &nvicPrio );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqPriority( RNG_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RNG_IRQ_PRIO, irqPrio );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_IrqPriority( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_IrqPriority( RNG_PERIPH_CNT, 0u ) );
}


/**
 * \brief   ISR delivers data to data ready callback.
 *
 * \par Expected results
 * - Data ready callback called once with data register value, no error callback.
 */
void Ut_Rng_Isr_DataReady_CallsDataCallback( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    RNG->SR = RNG_SR_DRDY;
    RNG->DR = UT_RNG_DATA;

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_DataCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_DATA, utRng_LastData.value[ 0u ] );
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_DATA, utRng_LastData.value[ RNG_FIFO_LEN - 1u ] );
}


/**
 * \brief   ISR reports and clears errors without data delivery.
 *
 * \par Expected results
 * - Error callback called once with SEED_ERR, SEIS cleared, no data callback.
 */
void Ut_Rng_Isr_SeedError_CallsErrorCallback( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    RNG->SR = RNG_SR_DRDY | RNG_SR_SEIS;

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_ErrorCnt );
    TEST_ASSERT_EQUAL( RNG_ERROR_SEED_ERR, utRng_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
}


/**
 * \brief   ISR discards data when the generation is not running.
 *
 * \details Interrupt style initialized, generator not started, DRDY preset.
 *
 * \par Expected results
 * - No callback called.
 */
void Ut_Rng_Isr_NotRunning_DiscardsData( void )
{
    Ut_Rng_Init( RNG_TRANSFER_INTERRUPT );

    RNG->SR = RNG_SR_DRDY;

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_ErrorCnt );
}

/* ================================= TASK =================================== */

/**
 * \brief   Rng_Task() delivers data in blocking style.
 *
 * \par Expected results
 * - Data ready callback called once with data register value.
 */
void Ut_Rng_Task_Blocking_CallsDataCallback( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_BLOCKING );

    RNG->SR = RNG_SR_DRDY;
    RNG->DR = UT_RNG_DATA;

    Rng_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_DataCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_DATA, utRng_LastData.value[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_ErrorCnt );
}


/**
 * \brief   Rng_Task() reports and clears errors in blocking style.
 *
 * \par Expected results
 * - Error callback called once with CLOCK_ERR, CEIS cleared, no data callback.
 */
void Ut_Rng_Task_Blocking_ClockError_CallsErrorCallback( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_BLOCKING );

    RNG->SR = RNG_SR_DRDY | RNG_SR_CEIS;

    Rng_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_ErrorCnt );
    TEST_ASSERT_EQUAL( RNG_ERROR_CLOCK_ERR, utRng_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );
    TEST_ASSERT_BITS_LOW( RNG_SR_CEIS, RNG->SR );
}


/**
 * \brief   Rng_Task() does not process paused, stopped or interrupt driven peripheral.
 *
 * \par Expected results
 * - No callback called while paused, after stop and in interrupt style.
 */
void Ut_Rng_Task_PausedOrInterrupt_DoesNothing( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );
    RNG->SR = RNG_SR_DRDY;
    Rng_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecInactive( RNG_PERIPH_1 ) );
    RNG->SR = RNG_SR_DRDY;
    Rng_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );

    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );
    RNG->SR = RNG_SR_DRDY;
    Rng_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Stub of Nvic_Set_PeriphIrq_Handler - stores registered RNG ISR */
static nvic_RequestState_t Ut_Rng_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_RNG_NVIC_IRQ, irqId );

    utRng_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** Data ready user callback - counts calls and stores data */
static void Ut_Rng_DataRdyCallback( rng_RandomData_t value )
{
    utRng_DataCnt++;
    utRng_LastData = value;

    if( 0u != utRng_PauseInCallback )
    {
        utRng_PauseResult = Rng_Set_ExecPauseActive( RNG_PERIPH_1 );
    }
    else
    {
        /* Delivery continues */
    }
}


/** Error user callback - counts calls and stores error mask */
static void Ut_Rng_ErrorCallback( rng_Error_t errMask )
{
    utRng_ErrorCnt++;
    utRng_LastError = errMask;
}


/** Returns default configuration with test callbacks, priority and transfer style */
static rng_ConfigStruct_t Ut_Rng_Get_Config( rng_TransferStyle_t transferStyle )
{
    rng_ConfigStruct_t config;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );

    config.TransferStyle   = transferStyle;
    config.IrqPrio         = UT_RNG_IRQ_PRIO;
    config.DataRdyCallback = Ut_Rng_DataRdyCallback;
    config.ErrorCallback   = Ut_Rng_ErrorCallback;

    return ( config );
}


/** Initializes the module with default configuration and required transfer style */
static void Ut_Rng_Init( rng_TransferStyle_t transferStyle )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( transferStyle );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Ut_Rng_Expect_LsiState( RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );

    if( RNG_TRANSFER_INTERRUPT == transferStyle )
    {
        Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_RNG_NVIC_IRQ, UT_RNG_IRQ_PRIO, NVIC_REQUEST_OK );
        Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    }
    else
    {
        /* Interrupt is not configured */
    }

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );
}


/** Initializes the module and starts the generator */
static void Ut_Rng_Init_Running( rng_TransferStyle_t transferStyle )
{
    Ut_Rng_Init( transferStyle );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
}


/** Expects the state read of LSI before a kernel clock source other than LSI is selected */
static void Ut_Rng_Expect_LsiState( rcc_FunctionState_t lsiState )
{
#if defined(LL_RCC_RNG_CLKSOURCE_LSI)
    static rcc_FunctionState_t state;

    state = lsiState;

    Rcc_Get_OscState_ExpectAndReturn( RCC_OSC_LSI, NULL, RCC_REQUEST_OK );
    Rcc_Get_OscState_IgnoreArg_retState();
    Rcc_Get_OscState_ReturnThruPtr_retState( &state );
#else
    /* STM32H7R / STM32H7S: no kernel clock multiplexer - LSI state is not read */
    (void)lsiState;
#endif /* LL_RCC_RNG_CLKSOURCE_LSI */
}


/**
 * \brief Brings the module to its reset state before every test (AB#1256).
 *
 * The module keeps its runtime data in static variables shared by all tests of the test binary
 * (transfer style, execution state, callbacks and the RCC identification of the kernel clock
 * source). Rng_Deinit() resets the runtime data, Rng_Set_ClkSource() restores the default kernel
 * clock source HSI48 - RCC / NVIC calls are ignored. The RCC and NVIC mocks are re-initialized
 * together afterwards (CMock mocks share one memory).
 */
static void Ut_Rng_Release( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_OscState_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_OscActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_OscInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );

    (void)Rng_Deinit( &config );
    (void)Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 );

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
}


/** Expects the state read of the peripheral clock of the previous kernel clock source */
static void Ut_Rng_Expect_PrevClock( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState )
{
    static rcc_FunctionState_t state;

    state = clockState;

    Rcc_Get_PeriphState_ExpectAndReturn( rccId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &state );
}
