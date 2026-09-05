/**
 * \author Mr.Nobody
 * \file Rng_Port.h
 * \ingroup Rng
 * \brief Random Number Generator (RNG) module public functionality
 *
 * This file contains all available public functionality, any other files shall 
 * be used outside of the module.
 *
 */

#ifndef RNG_RNG_PORT_H
#define RNG_RNG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Rng_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

rng_ModuleVersion_t     Rng_Get_ModuleVersion           ( void );

rng_RequestState_t      Rng_Init                        ( rng_ConfigStruct_t * const rngConfig );
rng_RequestState_t      Rng_Deinit                      ( rng_ConfigStruct_t * const rngConfig );
void                    Rng_Task                        ( void );

rng_RequestState_t      Rng_Get_DefaultConfig           ( rng_ConfigStruct_t *rngConfig );

rng_RequestState_t      Rng_Set_ExecActive              ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Set_ExecInactive            ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Get_ExecState               ( rng_PeriphId_t rngId, rng_FunctionState_t * const state );

rng_RequestState_t      Rng_Set_ExecPauseActive         ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Set_ExecPauseInactive       ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Get_ExecPauseState          ( rng_PeriphId_t rngId, rng_FunctionState_t * const state );

rng_RequestState_t      Rng_Get_Data                    ( rng_PeriphId_t rngId, rng_RandomData_t * const data );

/*-------------------------- Primitive functionality -------------------------*/

rng_RequestState_t      Rng_Set_Configuration           ( rng_PeriphId_t rngId, rng_ConfigSelection_t configSelection );
rng_RequestState_t      Rng_Get_Configuration           ( rng_PeriphId_t rngId, rng_ConfigSelection_t * const configSelection );

rng_RequestState_t      Rng_Get_DataReadyState          ( rng_PeriphId_t rngId, rng_FunctionState_t * const state );

rng_RequestState_t      Rng_Get_Error                   ( rng_PeriphId_t rngId, rng_Error_t * const errMask );
rng_RequestState_t      Rng_Clear_Error                 ( rng_PeriphId_t rngId, rng_Error_t errMask );

rng_RequestState_t      Rng_Set_ClockErrorDetectActive  ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Set_ClockErrorDetectInactive( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Get_ClockErrorDetectState   ( rng_PeriphId_t rngId, rng_FunctionState_t * const state );

rng_RequestState_t      Rng_Set_AutoResetActive         ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Set_AutoResetInactive       ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Get_AutoResetState          ( rng_PeriphId_t rngId, rng_FunctionState_t * const reqState );

rng_RequestState_t      Rng_Set_ClkSource               ( rng_PeriphId_t rngId, rng_ClkSource_t clkSource );

rng_RequestState_t      Rng_Set_ClkDivider              ( rng_PeriphId_t rngId, rng_ClkDivider_t clkDivider );
rng_RequestState_t      Rng_Get_ClkDivider              ( rng_PeriphId_t rngId, rng_ClkDivider_t * const clkDivider );

/*---------------------- Transfer handling configuration ---------------------*/

rng_RequestState_t      Rng_Set_DataRdyCallback         ( rng_PeriphId_t rngId, rng_DataRdyCallback_t * const callback );
rng_RequestState_t      Rng_Get_DataRdyCallback         ( rng_PeriphId_t rngId, rng_DataRdyCallback_t ** const callback );

rng_RequestState_t      Rng_Set_ErrorCallback           ( rng_PeriphId_t rngId, rng_ErrCallback_t * const callback );
rng_RequestState_t      Rng_Get_ErrorCallback           ( rng_PeriphId_t rngId, rng_ErrCallback_t ** const callback );

/*-------------------- Interrupts handling functionality ---------------------*/

rng_RequestState_t      Rng_Set_IrqActive               ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Set_IrqInactive             ( rng_PeriphId_t rngId );
rng_RequestState_t      Rng_Get_IrqState                ( rng_PeriphId_t rngId, rng_FunctionState_t * const reqState );

rng_RequestState_t      Rng_Set_IrqPriority             ( rng_PeriphId_t rngId, rng_IrqPrio_t irqPrio );
rng_RequestState_t      Rng_Get_IrqPriority             ( rng_PeriphId_t rngId, rng_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* RNG_RNG_PORT_H */

