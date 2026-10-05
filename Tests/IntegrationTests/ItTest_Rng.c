/**
 * \author Mr.Nobody
 * \file ItTest_Rng.c
 * \ingroup Rng
 * \brief Integration tests of Random Number Generator (RNG) module on target.
 *
 * Rng module runs on the MCU together with real RCC and NVIC modules and
 * hardware. Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): generation of random data (one word per DRDY), data
 * ready interrupt, pause of the interrupt delivery, fixed features of STM32F4
 * RNG and reinitialization.
 *
 * Used resources:
 * - RNG, kernel clock PLL48CLK (48 MHz main PLL Q output of RCC default
 *   configuration, PLL is started by StartUp)
 * - no external wiring
 *
 * \note Clock error can not be provoked - RNG clock (48 MHz) is always higher
 *       than HCLK / 16 on STM32F4 and the kernel clock source is fixed.
 *
 * \note Waiting is bounded by number of loop iterations (no reference timer),
 *       limits are set with large margin for the default system clock.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Rng_Port.h"                       /* Module under test              */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 It_Rng_Init_Module          ( rng_ClkSource_t clkSource, rng_TransferStyle_t transferStyle );
static rng_RequestState_t   It_Rng_Wait_Data            ( rng_RandomData_t * const data );
static void                 It_Rng_Wait_DataCnt         ( uint32_t dataCnt, rng_FunctionState_t taskState );

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
 * \brief   Rng_Init() with default configuration reports fixed configuration.
 *
 * \details Initializes the module with default configuration (PLL48CLK, NIST,
 *          blocking).
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

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_BLOCKING );

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

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_NONE );
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
    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_BLOCKING );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    It_Rng_Wait_DataCnt( IT_RNG_BLOCK_CNT, RNG_FUNCTION_ACTIVE );

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( IT_RNG_BLOCK_CNT, itRng_DataCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itRng_ErrorCnt );
}


/**
 * \brief   Data ready interrupt delivers data in interrupt style.
 *
 * \details STM32F4 RNG raises data ready interrupt for every block (new word
 *          every 40 RNG clock periods) - the ISR occupies the CPU completely.
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

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_INTERRUPT );

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

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_INTERRUPT );

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
 * \brief   Features not available on STM32F4 keep the fixed configuration.
 *
 * \details Requests of features without hardware support (AIS-31, clock
 *          divider, clock error detection deactivation, auto reset) while the
 *          generator is running.
 *
 * \par Expected results
 * - Unsupported requests return RNG_REQUEST_ERROR, fixed states read back
 *   (clock error detection active, auto reset inactive, divider 1, NIST).
 * - Generator keeps delivering data without error.
 */
void It_Rng_FixedFeatures_Running_KeepConfiguration( void )
{
    rng_FunctionState_t   state     = RNG_FUNCTION_INACTIVE;
    rng_ConfigSelection_t selection = RNG_CONFIG_SELECTION_CNT;
    rng_ClkDivider_t      divider   = RNG_CLK_DIV_CNT;
    rng_Error_t           errMask   = RNG_ERROR_ALL;
    rng_RandomData_t      data;

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_Configuration( RNG_PERIPH_1, RNG_CONFIG_SELECTION_AIS_31_SPEED ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClkDivider( RNG_PERIPH_1, RNG_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_ClockErrorDetectInactive( RNG_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_ERROR, Rng_Set_AutoResetActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_Configuration( RNG_PERIPH_1, &selection ) );
    TEST_ASSERT_EQUAL( RNG_CONFIG_SELECTION_NIST, selection );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClkDivider( RNG_PERIPH_1, &divider ) );
    TEST_ASSERT_EQUAL( RNG_CLK_DIV_1, divider );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ClockErrorDetectState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_ACTIVE, state );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_AutoResetState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

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

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_NONE );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Set_ExecActive( RNG_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Deinit( &config ) );

    TEST_ASSERT_EQUAL( RNG_REQUEST_OK, Rng_Get_ExecState( RNG_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( RNG_FUNCTION_INACTIVE, state );

    It_Rng_Init_Module( RNG_CLK_SOURCE_PLL48CLK, RNG_TRANSFER_NONE );
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

    for( uint32_t loopCnt = 0u; ( IT_RNG_WAIT_LOOPS > loopCnt ) && ( RNG_REQUEST_OK != retState ); loopCnt ++ )
    {
        retState = Rng_Get_Data( RNG_PERIPH_1, data );
    }

    return ( retState );
}


/** Waits until data ready callback is called required number of times (task optionally called) */
static void It_Rng_Wait_DataCnt( uint32_t dataCnt, rng_FunctionState_t taskState )
{
    for( uint32_t loopCnt = 0u; ( IT_RNG_WAIT_LOOPS > loopCnt ) && ( dataCnt > itRng_DataCnt ); loopCnt ++ )
    {
        if( RNG_FUNCTION_ACTIVE == taskState )
        {
            Rng_Task();
        }
        else
        {
            /* Data are delivered by interrupt */
        }
    }
}


/** Data ready user callback - counts calls, pauses delivery after required count */
static void It_Rng_DataRdyCallback( rng_RandomData_t value )
{
    (void)value;

    itRng_DataCnt++;

    if( ( 0u != itRng_PauseCnt ) && ( itRng_PauseCnt <= itRng_DataCnt ) )
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
