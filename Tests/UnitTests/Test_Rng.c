/**
 * \author Mr.Nobody
 * \file Test_Rng.c
 * \ingroup Rng
 * \brief Unit tests of Random Number Generator (RNG) module.
 *
 * Rng.c is compiled unchanged with real LL drivers. RNG registers are emulated
 * by RegMem, RCC and NVIC modules are mocked by CMock.
 *
 * \note Emulated registers are plain RAM: status flags (DRDY, SEIS, CEIS, SECS)
 *       are preset by tests. Flags are cleared by writing inverted mask (rc_w0),
 *       emulated SR therefore contains the remaining bits set.
 *
 * \note Three RNG variants of the family are covered by the presets: STM32L4
 *       (no CED - STM32L432xC, L476xG, L496xG, L412xB, L452xE), STM32L4R / L4S
 *       (CED - STM32L4R5xI) and STM32L4P5 / L4Q5 (CONDRST, CLKDIV - STM32L4P5xG).
 *       Variant specific expectations are selected by CMSIS definitions.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "Rng_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
/* STM32L4P5 / L4Q5 LL_RNG_SetHealthConfig() ignores its parameter - unused parameter warning suppressed */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Stm32_rng.h"                      /* RNG registers definition       */
#pragma GCC diagnostic pop
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_Rng_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static void                 Ut_Rng_DataRdyCallback      ( rng_RandomData_t value );
static void                 Ut_Rng_ErrorCallback        ( rng_Error_t errMask );
static rng_ConfigStruct_t   Ut_Rng_Get_Config           ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Init                 ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Init_Running         ( rng_TransferStyle_t transferStyle );
static void                 Ut_Rng_Expect_PrevClock     ( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by tests */
#define UT_RNG_IRQ_PRIO                     ( 3u )

/** Random value preset in data register */
#define UT_RNG_DATA                         ( 0xA5C3E1F7u )

#if defined(HASH) && \
    !defined(DMAMUX1)
/** RNG interrupt shared with HASH */
#define UT_RNG_NVIC_IRQ                     ( NVIC_PERIPH_IRQ_HASH_RNG )
#else
/** RNG interrupt */
#define UT_RNG_NVIC_IRQ                     ( NVIC_PERIPH_IRQ_RNG )
#endif /* HASH */

#if defined(RCC_CRRCR_HSI48ON)
/** Default kernel clock source */
#define UT_RNG_DEFAULT_CLK_SOURCE           ( RNG_CLK_SOURCE_HSI48 )

/** RCC ID of the default kernel clock source */
#define UT_RNG_DEFAULT_RCC                  ( RCC_PERIPH_RNG_HSI48 )
#else
/** Default kernel clock source (no HSI48 on STM32L47x / L48x) */
#define UT_RNG_DEFAULT_CLK_SOURCE           ( RNG_CLK_SOURCE_PLLQ )

/** RCC ID of the default kernel clock source */
#define UT_RNG_DEFAULT_RCC                  ( RCC_PERIPH_RNG_PLLQ )
#endif /* RCC_CRRCR_HSI48ON */

#if defined(RNG_CR_CONDRST)
/** CR configuration bits of NIST configuration (STM32L4P5 / L4Q5) */
#define UT_RNG_CR_NIST_CONFIG               ( RNG_CR_CONFIG_VAL & ( RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_NISTC ) )
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

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utRng_Isr       = NULL;
    utRng_DataCnt   = 0u;
    utRng_ErrorCnt  = 0u;
    utRng_LastError = RNG_ERROR_NONE;

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
 * - RNG_REQUEST_OK, peripheral 1, HSI48 (PLL Q on STM32L47x / L48x), NIST,
 *   blocking, priority 0, no callbacks.
 * - NULL pointer: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_DefaultConfig_ReturnsDefaults( void )
{
    rng_ConfigStruct_t config;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( RNG_PERIPH_1,              config.PeriphId );
    TEST_ASSERT_EQUAL( UT_RNG_DEFAULT_CLK_SOURCE, config.ClkSource );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, config.ConfigSelection );
    TEST_ASSERT_EQUAL( RNG_TRANSFER_BLOCKING,     config.TransferStyle );
    TEST_ASSERT_EQUAL_UINT32( 0u,                 config.IrqPrio );
    TEST_ASSERT_NULL( config.DataRdyCallback );
    TEST_ASSERT_NULL( config.ErrorCallback );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_DefaultConfig( NULL ) );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Rng_Init() with blocking style stops generator and interrupt.
 *
 * \details State of previous initialization (RNGEN, IE) is preset.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, NVIC IRQ deactivated, RCC clock of RNG (default source) activated.
 * - Generator and peripheral interrupt disabled, no ISR registered.
 * - STM32L4P5 / L4Q5: CR holds the NIST configuration bits (CONDRST released),
 *   HTCR holds the NIST value.
 */
void Ut_Rng_Init_Blocking_StopsGenerator( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;    /* State of previous initialization */

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );

    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_CR_NIST_CONFIG, RNG->CR );
    TEST_ASSERT_EQUAL_HEX32( RNG_HTCFG, RNG->HTCR );
#endif /* RNG_CR_CONDRST */
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
}


/**
 * \brief   Rng_Init() fails when the clock is not activated.
 *
 * \details Generator and interrupt are stopped before the clock selection.
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, NVIC IRQ deactivated, registers untouched.
 */
void Ut_Rng_Init_RccFailure_ReturnsError( void )
{
    rng_ConfigStruct_t config = Ut_Rng_Get_Config( RNG_TRANSFER_BLOCKING );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
}


/**
 * \brief   Rng_Init() rejects AIS-31 configuration (not defined for STM32L4 / STM32L4+).
 *
 * \par Expected results
 * - RNG_REQUEST_ERROR, callbacks are not stored, interrupt is not configured.
 */
void Ut_Rng_Init_Ais31Config_ReturnsError( void )
{
    rng_ConfigStruct_t     config    = Ut_Rng_Get_Config( RNG_TRANSFER_INTERRUPT );
    rng_DataRdyCallback_t *dataRdyCb = Ut_Rng_DataRdyCallback;

    config.ConfigSelection = RNG_CONFIG_SELECTION_AIS_31_SPEED;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_DataRdyCallback( RNG_PERIPH_1, NULL ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Init( &config ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DataRdyCallback( RNG_PERIPH_1, &dataRdyCb ) );
    TEST_ASSERT_NULL( dataRdyCb );
    TEST_ASSERT_BITS_LOW( RNG_CR_IE, RNG->CR );
    TEST_ASSERT_NULL( utRng_Isr );
}


/**
 * \brief   Rng_Deinit() releases interrupt, generator, reset and clock.
 *
 * \details Initializes with interrupt style, starts the generator and
 *          deinitializes the module.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, NVIC deactivated, RCC reset pulse and clock disable,
 *   RNGEN and IE cleared, callbacks removed.
 * - NULL pointer / invalid peripheral: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Deinit_ReleasesResources( void )
{
    rng_ConfigStruct_t     config      = Ut_Rng_Get_Config( RNG_TRANSFER_INTERRUPT );
    rng_DataRdyCallback_t *dataRdyCb   = Ut_Rng_DataRdyCallback;
    rng_ErrCallback_t     *errorCb     = Ut_Rng_ErrorCallback;

    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_RNG_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );

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
    Rcc_Set_ResetActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Deinit( &config ) );
}

/* ============================ CONFIGURATION =============================== */

/**
 * \brief   Only NIST configuration is accepted.
 *
 * \par Expected results
 * - Set NIST: RNG_REQUEST_OK, Get: NIST.
 * - STM32L4, STM32L4R / L4S: fixed configuration, registers untouched.
 * - STM32L4P5 / L4Q5: CR holds the NIST configuration bits (CONDRST released),
 *   HTCR holds the NIST value.
 * - AIS-31 / invalid selection / invalid ID / NULL: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_Configuration_OnlyNist( void )
{
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_NIST ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_SPEED ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_ENTROPY ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_CNT, RNG_CONFIG_SELECTION_NIST ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_CNT, &selection ) );

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL_HEX32( UT_RNG_CR_NIST_CONFIG, RNG->CR );
    TEST_ASSERT_EQUAL_HEX32( RNG_HTCFG, RNG->HTCR );
#else
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Locked or unknown configuration (STM32L4P5 / L4Q5).
 *
 * \details CONFIGLOCK preset, NIST requested. Then CR configuration bits
 *          differing from NIST are preset and read.
 *
 * \par Expected results
 * - Locked: RNG_REQUEST_ERROR, CR and HTCR untouched.
 * - Unknown configuration bits: Rng_Get_Configuration() returns RNG_REQUEST_ERROR.
 * - Other lines (fixed configuration): test ignored.
 */
void Ut_Rng_Set_Configuration_LockedOrUnknown_ReturnsError( void )
{
#if defined(RNG_CR_CONDRST)
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;

    RNG->CR = RNG_CR_CONFIGLOCK;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_NIST ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CONFIGLOCK, RNG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->HTCR );

    RNG->CR = UT_RNG_CR_NIST_CONFIG ^ RNG_CR_NISTC;

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
#else
    TEST_IGNORE_MESSAGE( "RNG without configuration lock (fixed configuration)" );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Set_ClkSource() activates RCC clock of required source.
 *
 * \details Sources are changed one after another (default -> MSI -> default), the
 *          clock of the previous source is reported active or inactive.
 *
 * \par Expected results
 * - Every source maps to its RCC peripheral ID, RNG_REQUEST_OK.
 * - Active clock of the previous source released before the new one is activated,
 *   same source or inactive previous clock: no release.
 * - Release failure, state read failure, RCC failure: RNG_REQUEST_ERROR.
 * - Invalid source / ID: RNG_REQUEST_ERROR without RCC call.
 * - Default source selected at the end (state of the following tests in one process).
 */
void Ut_Rng_Set_ClkSource_ActivatesRccClock( void )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, UT_RNG_DEFAULT_CLK_SOURCE ) );

    Ut_Rng_Expect_PrevClock( UT_RNG_DEFAULT_RCC, RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_MSI, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_MSI ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_MSI, RCC_FUNCTION_ACTIVE );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_RNG_MSI, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, UT_RNG_DEFAULT_CLK_SOURCE ) );

    Rcc_Get_PeriphState_ExpectAndReturn( RCC_PERIPH_RNG_MSI, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, UT_RNG_DEFAULT_CLK_SOURCE ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_MSI, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, UT_RNG_DEFAULT_CLK_SOURCE ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_MSI, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, UT_RNG_DEFAULT_CLK_SOURCE ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkSource( RNG_PERIPH_CNT, UT_RNG_DEFAULT_CLK_SOURCE ) );
}


/**
 * \brief   Every kernel clock source maps to its RCC peripheral ID.
 *
 * \details Sources PLLSAI1 Q, MSI, PLL Q and HSI48 of the list of the device line
 *          are selected one after another, the clock of the previous source is
 *          reported inactive (HSI48 is not in the list of STM32L47x / L48x,
 *          PLLSAI1 Q is not in the list of STM32L41x / L42x).
 *
 * \par Expected results
 * - RCC clock of the source activated, RNG_REQUEST_OK.
 * - Default source selected at the end.
 */
void Ut_Rng_Set_ClkSource_MapsSourcesToRcc( void )
{
    rcc_PeriphId_t currentRcc = UT_RNG_DEFAULT_RCC;

#if defined(RCC_CR_PLLSAI1ON)
    Ut_Rng_Expect_PrevClock( currentRcc, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_PLLSAI1Q, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_PLLSAI1Q ) );
    currentRcc = RCC_PERIPH_RNG_PLLSAI1Q;
#endif /* RCC_CR_PLLSAI1ON */

    Ut_Rng_Expect_PrevClock( currentRcc, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_MSI, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_MSI ) );

    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_MSI, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_PLLQ, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_PLLQ ) );

#if defined(RCC_CRRCR_HSI48ON)
    Ut_Rng_Expect_PrevClock( RCC_PERIPH_RNG_PLLQ, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_RNG_HSI48, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, RNG_CLK_SOURCE_HSI48 ) );
#endif /* RCC_CRRCR_HSI48ON */
}


/**
 * \brief   The lists of kernel clock sources and clock dividers contain the values of the
 *          device line only.
 *
 * \details Counts the values of \ref rng_ClkSource_t and \ref rng_ClkDivider_t of the build:
 *          PLL Q and MSI on every line, HSI48 (not on STM32L47x / L48x) and PLLSAI1 Q (not on
 *          STM32L41x / L42x) by the device features; divider 1 on every line, dividers 2 -
 *          32768 only on STM32L4P5 / L4Q5 (CLKDIV).
 *
 * \par Expected results
 * - RNG_CLK_SOURCE_CNT is 2 to 4 according to the device line.
 * - RNG_CLK_DIV_CNT is 16 with CLKDIV, 1 without.
 */
void Ut_Rng_ClkSourceAndDividerLists_MatchDeviceLine( void )
{
    uint32_t expectedSources  = 2u;
    uint32_t expectedDividers = 1u;

#if defined(RCC_CRRCR_HSI48ON)
    expectedSources++;
#endif /* RCC_CRRCR_HSI48ON */
#if defined(RCC_CR_PLLSAI1ON)
    expectedSources++;
#endif /* RCC_CR_PLLSAI1ON */
#if defined(RNG_CR_CONDRST)
    expectedDividers = 16u;
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL_UINT32( expectedSources, (uint32_t)RNG_CLK_SOURCE_CNT );
    TEST_ASSERT_EQUAL_UINT32( expectedDividers, (uint32_t)RNG_CLK_DIV_CNT );
}


/**
 * \brief   Clock divider.
 *
 * \par Expected results
 * - STM32L4P5 / L4Q5: dividers 2 and 32768 written to CLKDIV (CONDRST released)
 *   and read back, divider 1 restores CLKDIV 0.
 * - Other lines (no divider, divider 1 is the only item of the list): divider 1
 *   accepted and read back, registers untouched.
 * - Invalid divider / ID / NULL: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_ClkDivider_SupportedDividers( void )
{
    rng_ClkDivider_t divider = RNG_CLK_DIV_CNT;

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CLKDIV_0, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_2, divider );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_32768 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CLKDIV, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_32768, divider );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkDivider( RNG_PERIPH_CNT, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClkDivider( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_ClkDivider( RNG_PERIPH_CNT, &divider ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
}


/**
 * \brief   Clock error detection activation and state with stopped generator.
 *
 * \par Expected results
 * - STM32L4R / L4S, STM32L4P5 / L4Q5: Inactive: CED set, state inactive.
 *   Active: CED cleared (detection enabled), state active. Generator stays
 *   stopped, CONDRST released.
 * - STM32L4 (no CED): Active: RNG_REQUEST_OK, Inactive: RNG_REQUEST_ERROR,
 *   state always active, registers untouched.
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_ClockErrorDetect_ControlsCedBit( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

#if defined(RNG_CR_CED)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_CED, RNG->CR );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
#else
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );
#endif /* RNG_CR_CED */

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
 * \brief   Clock error detection change of a running generator restarts it.
 *
 * \details Presets RNGEN | IE (running generator with interrupt), changes CED.
 *
 * \note    CED is taken into account only with RNGEN = 0 - the module stops
 *          the generator for the change. Emulated register keeps the final
 *          state only.
 *
 * \par Expected results
 * - STM32L4R / L4S, STM32L4P5 / L4Q5: Inactive: RNG_REQUEST_OK, CED set, RNGEN
 *   and IE set again. Active: RNG_REQUEST_OK, CED cleared, RNGEN and IE set again.
 * - STM32L4 (no CED): Inactive rejected, Active accepted, CR untouched.
 */
void Ut_Rng_Set_ClockErrorDetect_Running_RestartsGenerator( void )
{
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;

#if defined(RNG_CR_CED)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE | RNG_CR_CED, RNG->CR );
#else
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
#endif /* RNG_CR_CED */

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
}


/**
 * \brief   Auto reset has a fixed state.
 *
 * \par Expected results
 * - STM32L4P5 / L4Q5 (automatic reset, no ARDIS): Active: RNG_REQUEST_OK,
 *   Inactive: RNG_REQUEST_ERROR, state always active.
 * - Other lines (no auto reset): Active: RNG_REQUEST_ERROR, Inactive:
 *   RNG_REQUEST_OK, state always inactive.
 * - Registers untouched. NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Set_AutoReset_FixedState( void )
{
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetInactive( RNG_PERIPH_1 ) );
    state = RNG_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
#else
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_AutoResetInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL_HEX32( 0u, RNG->CR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Get_AutoResetState( RNG_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetInactive( RNG_PERIPH_CNT ) );
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
 * - Current state flags (SECS, CECS) are not reported.
 * - NULL / invalid ID: RNG_REQUEST_ERROR.
 */
void Ut_Rng_Get_Error_MapsFlags( void )
{
    rng_Error_t errMask = RNG_ERROR_CNT;

    RNG->SR = RNG_SR_SECS | RNG_SR_CECS;
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
 * - CLOCK_ERR: CEIS cleared. SEED_ERR: SEIS cleared. ALL: both cleared.
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

    RNG->SR = RNG_SR_CEIS | RNG_SR_SEIS;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_ALL ) );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_CNT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Clear_Error( RNG_PERIPH_CNT, RNG_ERROR_ALL ) );
}


/**
 * \brief   Rng_Clear_Error() recovers running generator after seed error.
 *
 * \details Generator running, SECS and SEIS preset (active seed error).
 *
 * \par Expected results
 * - RNG_REQUEST_OK, SEIS cleared, generator enabled again (RNGEN high),
 *   IE kept (STM32L4, STM32L4R / L4S: RNGEN restart; STM32L4P5 / L4Q5:
 *   conditioning reset, CONDRST released).
 */
void Ut_Rng_Clear_Error_SeedRunning_RestartsGenerator( void )
{
    RNG->CR = RNG_CR_RNGEN | RNG_CR_IE;
    RNG->SR = RNG_SR_SECS | RNG_SR_SEIS;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_SEED_ERR ) );

    TEST_ASSERT_BITS_HIGH( RNG_CR_RNGEN | RNG_CR_IE, RNG->CR );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_BITS_LOW( RNG_CR_CONDRST, RNG->CR );
#endif /* RNG_CR_CONDRST */
}


/**
 * \brief   Rng_Clear_Error() keeps stopped generator stopped after seed error.
 *
 * \par Expected results
 * - RNG_REQUEST_OK, SEIS cleared, RNGEN stays low (STM32L4P5 / L4Q5:
 *   conditioning reset, CONDRST released).
 */
void Ut_Rng_Clear_Error_SeedStopped_KeepsGeneratorStopped( void )
{
    RNG->SR = RNG_SR_SECS | RNG_SR_SEIS;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Clear_Error( RNG_PERIPH_1, RNG_ERROR_SEED_ERR ) );

    TEST_ASSERT_BITS_LOW( RNG_CR_RNGEN, RNG->CR );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_BITS_LOW( RNG_CR_CONDRST, RNG->CR );
#endif /* RNG_CR_CONDRST */
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
 * - Error callback called once with SEED_ERR, SEIS cleared, generator still
 *   enabled (restarted), no data callback.
 */
void Ut_Rng_Isr_SeedError_CallsErrorCallback( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    RNG->SR = RNG_SR_DRDY | RNG_SR_SECS | RNG_SR_SEIS;

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_ErrorCnt );
    TEST_ASSERT_EQUAL( RNG_ERROR_SEED_ERR, utRng_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utRng_DataCnt );
    TEST_ASSERT_BITS_LOW( RNG_SR_SEIS, RNG->SR );
    TEST_ASSERT_BITS_HIGH( RNG_CR_RNGEN, RNG->CR );
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


/**
 * \brief   RNG interrupt service routine ends with DSB.
 *
 * \details Interrupt style initialized and started, data ready preset, ISR called once.
 *
 * \note    Cortex-M4 r0p1 erratum 838869 "Store immediate overlapping exception return
 *          operation might vector to incorrect interrupt" (STM32G4 Bug AB#1104, STM32L4 errata
 *          sheets not reviewed yet): a buffered store with immediate offset still pending at the
 *          exception return may vector to an incorrect interrupt. Workaround - DSB before the
 *          exception return of every handler.
 *
 * \par Expected results
 * - ISR executes exactly one DSB, data delivered.
 */
void Ut_Rng_Isr_EndsWithDsb( void )
{
    Ut_Rng_Init_Running( RNG_TRANSFER_INTERRUPT );

    RNG->SR = RNG_SR_DRDY;
    RNG->DR = UT_RNG_DATA;

    const uint32_t dsbCnt = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

    utRng_Isr();

    TEST_ASSERT_EQUAL_UINT32( dsbCnt + 1u, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utRng_DataCnt );
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
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_RNG_DEFAULT_RCC, RCC_REQUEST_OK );

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


/** Expects the state read of the peripheral clock of the previous kernel clock source */
static void Ut_Rng_Expect_PrevClock( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState )
{
    static rcc_FunctionState_t state;

    state = clockState;

    Rcc_Get_PeriphState_ExpectAndReturn( rccId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &state );
}
