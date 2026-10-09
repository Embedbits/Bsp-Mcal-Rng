/**
 * \author Mr.Nobody
 * \file ItTest_Rng.c
 * \ingroup Rng
 * \brief Integration tests of Random Number Generator (RNG) module on target.
 *
 * Rng module runs on the MCU together with real RCC and NVIC modules and
 * hardware. Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): generation of random data (one word per DRDY), data
 * ready interrupt, pause of the interrupt delivery, kernel clock sources of
 * the 48 MHz clock multiplexer (HSI48, main PLL output Q, PLLSAI1 output Q),
 * clock error detection change of a running generator, variant specific
 * features (clock divider, auto reset) and reinitialization.
 *
 * The test set is board independent (no external wiring, no board pins), it
 * runs on every STM32L4 / STM32L4+ board. RNG variants:
 * - STM32L4 (no CED): clock error detection always active.
 * - STM32L4R / L4S (CED): clock error detection can be disabled.
 * - STM32L4P5 / L4Q5 (CONDRST): clock divider, automatic reset after seed error.
 *
 * Used resources:
 * - RNG, kernel clock HSI48 (started by RCC module), main PLL output Q (40 MHz
 *   in RCC default configuration - default source of STM32L47x / L48x without
 *   HSI48) or PLLSAI1 output Q (configured by the test - HSI16, M 1, N 12,
 *   Q 4 = 48 MHz)
 * - no external wiring
 *
 * \note Clock error can not be provoked safely - the threshold of the clock
 *       checker depends on the RNG variant, all used RNG clocks (40 - 48 MHz)
 *       are high enough compared to HCLK (80 / 120 MHz). MSI kernel clock is
 *       not tested (its frequency depends on the MSI range of the application).
 *
 * \note Waiting is bounded by number of loop iterations (no reference timer),
 *       limits are set with large margin for the default system clock.
 *
 * \note STM32L4 errata sheets not reviewed yet.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Rng_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* PLLSAI1 output Q configuration */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 It_Rng_Init_Module          ( rng_ClkSource_t clkSource, rng_TransferStyle_t transferStyle );
static rng_RequestState_t   It_Rng_Wait_Data            ( rng_RandomData_t * const data );
static void                 It_Rng_Wait_DataCnt         ( uint32_t dataCnt, rng_FunctionState_t taskState );
#if defined(RCC_CR_PLLSAI1ON)
static void                 It_Rng_Set_PllSai1QActive   ( void );
#endif /* RCC_CR_PLLSAI1ON */

static void                 It_Rng_DataRdyCallback      ( rng_RandomData_t value );
static void                 It_Rng_ErrorCallback        ( rng_Error_t errMask );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Maximal number of wait loop iterations (data, callbacks, errors) */
#define IT_RNG_WAIT_LOOPS                   ( 2000000u )

/** Number of loop iterations used to check that no callback is called */
#define IT_RNG_SILENCE_LOOPS                ( 200000u )

/** Number of data blocks required by callback tests */
#define IT_RNG_BLOCK_CNT                    ( 3u )

/** Interrupt priority used by tests */
#define IT_RNG_IRQ_PRIO                     ( 5u )

#if defined(RCC_CRRCR_HSI48ON)
/** Default kernel clock source (HSI48) */
#define IT_RNG_CLK_DEFAULT                  ( RNG_CLK_SOURCE_HSI48 )

/** Second kernel clock source of the switching test (main PLL output Q, 40 MHz) */
#define IT_RNG_CLK_SECOND                   ( RNG_CLK_SOURCE_PLLQ )
#else
/** Default kernel clock source (main PLL output Q, STM32L47x / L48x without HSI48) */
#define IT_RNG_CLK_DEFAULT                  ( RNG_CLK_SOURCE_PLLQ )

/** Second kernel clock source of the switching test (PLLSAI1 output Q, 48 MHz) */
#define IT_RNG_CLK_SECOND                   ( RNG_CLK_SOURCE_PLLSAI1Q )
#endif /* RCC_CRRCR_HSI48ON */

/** PLLSAI1 multiplier N of the kernel clock tests (HSI16 / 1 * 12 = VCO 192 MHz) */
#define IT_RNG_PLLSAI1_N_MULT               ( 12u )

/** PLLSAI1 output Q divider of the kernel clock tests (VCO 192 MHz / 4 = 48 MHz) */
#define IT_RNG_PLLSAI1_Q_DIV                ( 4u )

/* ========================== LOCAL VARIABLES =============================== */

/** Count of data ready callback calls */
static volatile uint32_t            itRng_DataCnt   = 0u;

/** Count of error callback calls */
static volatile uint32_t            itRng_ErrorCnt  = 0u;

/** Error mask passed to the last error callback */
static volatile rng_Error_t         itRng_LastError = RNG_ERROR_NONE;

/** Count of data callbacks after which the callback pauses delivery (0 - never) */
static volatile uint32_t            itRng_PauseCnt  = 0u;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itRng_DataCnt   = 0u;
    itRng_ErrorCnt  = 0u;
    itRng_LastError = RNG_ERROR_NONE;
    itRng_PauseCnt  = 0u;
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief   Rng_Init() with default configuration reports NIST configuration.
 *
 * \details Initializes the module with default configuration (HSI48 or main
 *          PLL output Q, NIST, blocking).
 *
 * \par Expected results
 * - Rng_Init() returns RNG_REQUEST_OK.
 * - Configuration reads back NIST, divider 1, interrupt and generation inactive.
 */
void It_Rng_Init_DefaultConfig_NistReadBack( void )
{
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;
    rng_ClkDivider_t      divider   = RNG_CLK_DIV_CNT;
    rng_FunctionState_t   state     = RNG_FUNCTION_ACTIVE;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_BLOCKING );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Running generator delivers different random blocks.
 *
 * \details Starts the generator and reads two blocks by Rng_Get_Data().
 *
 * \par Expected results
 * - Both blocks read within timeout, no error flag active.
 * - Blocks are not zero and differ from each other.
 */
void It_Rng_Get_Data_Running_DeliversRandomBlocks( void )
{
    rng_RandomData_t firstBlock;
    rng_RandomData_t secondBlock;
    rng_Error_t      errMask = RNG_ERROR_ALL;
    uint32_t         diffCnt = 0u;
    uint32_t         orValue = 0u;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &firstBlock ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &secondBlock ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );

    for( uint32_t index = 0u; RNG_FIFO_LEN > index; index ++ )
    {
        orValue |= firstBlock.value[ index ] | secondBlock.value[ index ];

        if( firstBlock.value[ index ] != secondBlock.value[ index ] )
        {
            diffCnt++;
        }
        else
        {
            /* Same word in both blocks */
        }
    }

    TEST_ASSERT_NOT_EQUAL_UINT32( 0u, orValue );
    TEST_ASSERT_EQUAL_UINT32( RNG_FIFO_LEN, diffCnt );
}


/**
 * \brief   Rng_Task() delivers data in blocking style.
 *
 * \par Expected results
 * - Data ready callback called at least 3 times by the task, no error callback.
 */
void It_Rng_Task_Blocking_CallsDataCallback( void )
{
    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_BLOCKING );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    It_Rng_Wait_DataCnt( IT_RNG_BLOCK_CNT, RNG_FUNCTION_ACTIVE );

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( IT_RNG_BLOCK_CNT, itRng_DataCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itRng_ErrorCnt );
}


/**
 * \brief   Data ready interrupt delivers data in interrupt style.
 *
 * \details STM32L4 RNG raises data ready interrupt for every block (new word
 *          after a few tens of RNG clock periods) - the ISR occupies the CPU completely.
 *          The callback pauses the delivery after 3 blocks (intended usage).
 *
 * \par Expected results
 * - Data ready callback called exactly 3 times from ISR (task not called),
 *   no error callback, interrupt priority reads back.
 * - Delivery paused from the callback (pause state active, IE cleared).
 */
void It_Rng_Isr_Interrupt_CallsDataCallback( void )
{
    rng_IrqPrio_t       irqPrio = 0u;
    rng_FunctionState_t state   = RNG_FUNCTION_INACTIVE;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_INTERRUPT );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqPriority( RNG_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( IT_RNG_IRQ_PRIO, irqPrio );

    itRng_PauseCnt = IT_RNG_BLOCK_CNT;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    It_Rng_Wait_DataCnt( IT_RNG_BLOCK_CNT, RNG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( IT_RNG_BLOCK_CNT, itRng_DataCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itRng_ErrorCnt );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecPauseState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_IrqState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Pause stops interrupt data delivery, resume restarts it.
 *
 * \details The callback pauses the delivery after the first block (see
 *          It_Rng_Isr_Interrupt_CallsDataCallback), then after next 3 blocks.
 *
 * \par Expected results
 * - No data callback while paused.
 * - Data callbacks continue after resume.
 */
void It_Rng_Set_ExecPause_Interrupt_StopsDelivery( void )
{
    uint32_t pausedCnt = 0u;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_INTERRUPT );

    itRng_PauseCnt = 1u;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    It_Rng_Wait_DataCnt( 1u, RNG_FUNCTION_INACTIVE );

    /* Delivery paused by the callback, second pause request is rejected */
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ExecPauseActive( RNG_PERIPH_1 ) );
    pausedCnt = itRng_DataCnt;
    TEST_ASSERT_EQUAL_UINT32( 1u, pausedCnt );

    for( volatile uint32_t loopCnt = 0u; IT_RNG_SILENCE_LOOPS > loopCnt; loopCnt ++ )
    {
        /* Generation keeps running, data must not be delivered */
    }

    TEST_ASSERT_EQUAL_UINT32( pausedCnt, itRng_DataCnt );

    itRng_PauseCnt = pausedCnt + IT_RNG_BLOCK_CNT;
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecPauseInactive( RNG_PERIPH_1 ) );

    It_Rng_Wait_DataCnt( pausedCnt + IT_RNG_BLOCK_CNT, RNG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( pausedCnt + IT_RNG_BLOCK_CNT, itRng_DataCnt );
}


/**
 * \brief   Fixed features keep their configuration while the generator runs.
 *
 * \details Requests of features without hardware support while the generator
 *          is running: AIS-31 configuration (all lines), auto reset change
 *          (fixed state of the variant). Lines without CLKDIV have divider 1
 *          only in the list of dividers.
 *
 * \par Expected results
 * - Unsupported requests return RNG_REQUEST_ERROR, fixed states read back:
 *   NIST, divider 1 (lines without CLKDIV), auto reset active on STM32L4P5 /
 *   L4Q5 (automatic reset), inactive on the other lines.
 * - Generator keeps delivering data without error.
 */
void It_Rng_FixedFeatures_Running_KeepConfiguration( void )
{
    rng_FunctionState_t   state     = RNG_FUNCTION_INACTIVE;
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;
    rng_Error_t           errMask   = RNG_ERROR_ALL;
    rng_RandomData_t      data;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_SPEED ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
#else
    rng_ClkDivider_t divider = RNG_CLK_DIV_CNT;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );

    state = RNG_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
}


/**
 * \brief   Clock error detection is changed while the generator is running.
 *
 * \details STM32L4R / L4S, STM32L4P5 / L4Q5: deactivates and activates clock
 *          error detection of the running generator (CED is taken into account
 *          only with RNGEN = 0, the module restarts the generator), reads data
 *          after every change. STM32L4 (no CED): deactivation is rejected.
 *
 * \par Expected results
 * - CED lines: both requests return RNG_REQUEST_OK, state reads back
 *   inactive / active.
 * - STM32L4: deactivation RNG_REQUEST_ERROR, activation RNG_REQUEST_OK, state
 *   always active.
 * - Execution state stays active, data are delivered without error after
 *   every change.
 */
void It_Rng_ClockErrorDetect_Running_GeneratorRestarted( void )
{
    rng_FunctionState_t state   = RNG_FUNCTION_ACTIVE;
    rng_Error_t         errMask = RNG_ERROR_ALL;
    rng_RandomData_t    data;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

#if defined(RNG_CR_CED)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );
#else
    state = RNG_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
#endif /* RNG_CR_CED */
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClockErrorDetectActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
}


/**
 * \brief   Clock divider of a running generator.
 *
 * \details STM32L4P5 / L4Q5: sets divider 2 (RNG clock 24 MHz with HSI48),
 *          reads data, restores divider 1. Lines without CLKDIV: divider 1 is
 *          the only item of the list of dividers.
 *
 * \par Expected results
 * - STM32L4P5 / L4Q5: RNG_REQUEST_OK, divider reads back 2 / 1, data are
 *   delivered without error with both dividers.
 * - Other lines: divider 1 RNG_REQUEST_OK, data are delivered without error.
 */
void It_Rng_Set_ClkDivider_Running_DeliversData( void )
{
    rng_ClkDivider_t divider = RNG_CLK_DIV_CNT;
    rng_Error_t      errMask = RNG_ERROR_ALL;
    rng_RandomData_t data;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

#if defined(RNG_CR_CONDRST)
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_2, divider );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
#endif /* RNG_CR_CONDRST */

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
}


/**
 * \brief   Main PLL output Q as kernel clock delivers random data.
 *
 * \details RCC default configuration enables main PLL output Q (40 MHz), the
 *          module is initialized with main PLL output Q kernel clock and started.
 *
 * \par Expected results
 * - Rng_Init() returns RNG_REQUEST_OK, data are delivered without error.
 */
void It_Rng_Init_PllQClock_DeliversData( void )
{
    rng_Error_t      errMask = RNG_ERROR_ALL;
    rng_RandomData_t data;

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLLQ, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
}


/**
 * \brief   PLLSAI1 output Q as kernel clock delivers random data.
 *
 * \details PLLSAI1 is configured (HSI16, M 1, N 12, Q 4 = 48 MHz), the module
 *          is initialized with PLLSAI1 output Q kernel clock and started.
 *          STM32L41x / L42x (no PLLSAI1): the source is not in the list of
 *          kernel clock sources, the test is ignored.
 *
 * \par Expected results
 * - PLLSAI1 lines: Rng_Init() returns RNG_REQUEST_OK, data are delivered
 *   without error.
 */
void It_Rng_Init_PllSai1QClock_DeliversData( void )
{
#if defined(RCC_CR_PLLSAI1ON)
    rng_Error_t      errMask = RNG_ERROR_ALL;
    rng_RandomData_t data;

    It_Rng_Set_PllSai1QActive();

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLLSAI1Q, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
#else
    TEST_IGNORE_MESSAGE( "No PLLSAI1 on STM32L41x / L42x - PLLSAI1 output Q is not a kernel clock source of the device line" );
#endif /* RCC_CR_PLLSAI1ON */
}


/**
 * \brief   Kernel clock is switched between two sources and back while the
 *          generator is running.
 *
 * \details Initializes with the default source (HSI48, main PLL output Q on
 *          STM32L47x / L48x) and reads data, switches to the second source
 *          (main PLL output Q, PLLSAI1 output Q on STM32L47x / L48x - the
 *          peripheral clock of the previous source is released first, RCC does
 *          not change the multiplexer of an active peripheral), reads data,
 *          switches back.
 *
 * \par Expected results
 * - Every Rng_Set_ClkSource() returns RNG_REQUEST_OK.
 * - Data are delivered without error with every source.
 */
void It_Rng_Set_ClkSource_Running_SwitchesKernelClock( void )
{
    rng_Error_t      errMask = RNG_ERROR_ALL;
    rng_RandomData_t data;

#if !defined(RCC_CRRCR_HSI48ON)
    It_Rng_Set_PllSai1QActive();
#endif /* RCC_CRRCR_HSI48ON */

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, IT_RNG_CLK_SECOND ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ClkSource( RNG_PERIPH_1, IT_RNG_CLK_DEFAULT ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Error( RNG_PERIPH_1, &errMask ) );
    TEST_ASSERT_EQUAL( RNG_ERROR_NONE, errMask );
}


/**
 * \brief   Rng_Deinit() stops the peripheral.
 *
 * \par Expected results
 * - Rng_Deinit() returns RNG_REQUEST_OK, execution state inactive.
 * - Module can be initialized again and delivers data.
 */
void It_Rng_Deinit_StopsAndAllowsReinit( void )
{
    rng_ConfigStruct_t  config;
    rng_FunctionState_t state = RNG_FUNCTION_ACTIVE;
    rng_RandomData_t    data;

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Deinit( &config ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    It_Rng_Init_Module( IT_RNG_CLK_DEFAULT, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, It_Rng_Wait_Data( &data ) );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Initializes the module with default configuration, required clock source and transfer style */
static void It_Rng_Init_Module( rng_ClkSource_t clkSource, rng_TransferStyle_t transferStyle )
{
    rng_ConfigStruct_t config;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );

    config.ClkSource       = clkSource;
    config.TransferStyle   = transferStyle;
    config.IrqPrio         = IT_RNG_IRQ_PRIO;
    config.DataRdyCallback = It_Rng_DataRdyCallback;
    config.ErrorCallback   = It_Rng_ErrorCallback;

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Init( &config ) );
}


/** Waits for one data block read by Rng_Get_Data() */
static rng_RequestState_t It_Rng_Wait_Data( rng_RandomData_t * const data )
{
    rng_RequestState_t retState = RNG_REQUEST_ERROR;

    for( uint32_t loopCnt = 0u; IT_RNG_WAIT_LOOPS > loopCnt; loopCnt ++ )
    {
        retState = Rng_Get_Data( RNG_PERIPH_1, data );

        if( RNG_REQUEST_OK == retState )
        {
            break;
        }
        else
        {
            /* Data are not ready yet */
        }
    }

    return ( retState );
}


/** Waits until data ready callback is called required number of times (task optionally called) */
static void It_Rng_Wait_DataCnt( uint32_t dataCnt, rng_FunctionState_t taskState )
{
    for( uint32_t loopCnt = 0u; IT_RNG_WAIT_LOOPS > loopCnt; loopCnt ++ )
    {
        if( dataCnt <= itRng_DataCnt )
        {
            break;
        }
        else if( RNG_FUNCTION_ACTIVE == taskState )
        {
            Rng_Task();
        }
        else
        {
            /* Data are delivered by interrupt */
        }
    }
}


#if defined(RCC_CR_PLLSAI1ON)
/** Configures PLLSAI1 output Q 48 MHz (source of the running main PLL) - kernel clock tests */
static void It_Rng_Set_PllSai1QActive( void )
{
    rcc_PllConfigStruct_t pllConfig =
    {
        .Pll_Source = RCC_PLL_SRC_HSI, .M_Divider = 1u,                   .N_Multiplier = IT_RNG_PLLSAI1_N_MULT,
        .P_Divider  = 0u,              .Q_Divider = IT_RNG_PLLSAI1_Q_DIV, .R_Divider    = 0u
    };

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_PllConfig( RCC_PLL_2, &pllConfig ) );
}
#endif /* RCC_CR_PLLSAI1ON */


/** Data ready user callback - counts calls, pauses delivery after required count */
static void It_Rng_DataRdyCallback( rng_RandomData_t value )
{
    (void)value;

    itRng_DataCnt++;

    if( ( 0u             != itRng_PauseCnt ) &&
        ( itRng_PauseCnt <= itRng_DataCnt  )    )
    {
        (void)Rng_Set_ExecPauseActive( RNG_PERIPH_1 );
    }
    else
    {
        /* Delivery continues */
    }
}


/** Error user callback - counts calls and stores error mask */
static void It_Rng_ErrorCallback( rng_Error_t errMask )
{
    itRng_ErrorCnt++;
    itRng_LastError = errMask;
}
