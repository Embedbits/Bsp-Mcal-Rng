# RNG Module

This module provides an abstraction for controlling and retrieving data from the hardware Random Number Generator (RNG).  
It offers initialization, configuration, execution control, error handling, and data retrieval functionality.

Each STM32 family is supported in a **dedicated branch** of this repository (`STM32U5`, `STM32H5`, ...)
with a common public interface.

---

## 📘 STM32H7 specifics

- **Kernel clock** (`rng_ClkSource_t`, RCC_D2CCIP2R.RNGSEL, STM32H7A3 / H7B0 / H7B3: RCC_CDCCIP2R.RNGSEL):
  `HSI48`, `PLL1Q`, `LSE`, `LSI`. Internal oscillators (HSI48, LSI) are started automatically by the RCC
  module when the peripheral clock is activated; LSE and PLL1 Q output shall be configured by the
  application. 48 MHz is required for NIST compliance. STM32H7R3 / H7R7 / H7S3 / H7S7 (Ral family STM32H7RS)
  have no RNG kernel clock multiplexer - the list contains `HSI48` only (`PLL1Q`, `LSE` and `LSI` are not defined).
- **RNG versions** - the module selects the behavior by the RNG features of the device:

  | Devices | RNG | Configuration | Clock divider | CED change | Seed error recovery |
  |---|---|---|---|---|---|
  | STM32H74x / H75x | without configuration | fixed, reported as `NIST` | list with `RNG_CLK_DIV_1` only | generator stopped and restarted | RNGEN 1 -> 0 -> 1 |
  | STM32H72x / H73x | version 3.2 (`RNG_CR_CONDRST`) | `NIST`: CR config bits 0 (NISTC = 0), HTCR `0x00007274` | `CLKDIV` | with CONDRST | CONDRST 1 -> 0 |
  | STM32H7A3 / H7B0 / H7B3 | version 3.1 (`RNG_CR_CONDRST`) | `NIST`: CR config bits 0 (NISTC = 0), HTCR `0x000CAA74` | `CLKDIV` | with CONDRST | CONDRST 1 -> 0 |
  | STM32H7R3 / H7R7 / H7S3 / H7S7 | `RNG_CR_CONDRST` + `RNG_CR_ARDIS` | `NIST`: CR config bits 0 (NISTC = 0), HTCR `0x0000AAC7` | `CLKDIV` | with CONDRST | auto reset (ARDIS = 0) or CONDRST 1 -> 0 |

- HTCR values are the AN4230 NIST values used by the STM32H7 HAL (`RNG_HTCFG`; STM32H7R / H7S: value of
  Zephyr `stm32h7rs.dtsi`). HTCR is written with the magic number but it is not read back (reading requires the
  magic number write); `Rng_Get_Configuration()` identifies the configuration by the CR configuration bits.
- **AIS-31 configurations** are not defined for STM32H7 RNG versions, `Rng_Set_Configuration()` rejects them.
- **Auto reset** (ARDIS) does not exist on the classic STM32H7 lines - `Rng_Set_AutoResetActive()` returns
  error, `Rng_Set_AutoResetInactive()` is accepted, the state is always inactive. STM32H7R / H7S: the auto reset
  functions control ARDIS (hardware default: auto reset active).
- On RNG version 3.x every configuration change (CR config bits, CLKDIV, CED, HTCR) is written with
  conditioning soft reset (CONDRST), the module waits for its completion and verifies the value by read-back.
  Locked configuration (CONFIGLOCK) is reported as error.
- **Interrupt** - on devices with HASH peripheral (STM32H730 / H733 / H735 / H750 / H753 / H755 / H757 /
  H7B0 / H7B3) the RNG interrupt is shared with HASH (`NVIC_PERIPH_IRQ_HASH_RNG`). STM32H7S has own RNG and HASH
  interrupts.
- Before a kernel clock source other than LSI is selected, LSI is started and stopped again if it is not
  running - a kernel clock switch pending from LSI after a system reset is completed (observed on STM32H5,
  AB#954; applied to STM32H7 as a precaution, not needed on STM32H7R / H7S without multiplexer).

### Behavior notes

- `Rng_Init()` stops the generator, disables the interrupt, selects the kernel clock (the clock of the
  previous source is released) and applies the configuration; the generation is started by
  `Rng_Set_ExecActive()`.
- `Rng_Get_Data()` returns `RNG_REQUEST_OK` only for a complete 128-bit block read without seed error.
- Blocking style - `Rng_Task()` clears and reports errors (error callback), otherwise delivers data
  (data ready callback). Interrupt style - the same is done by the ISR.
- `Rng_Set_ExecPauseActive()` stops the delivery; in interrupt style the peripheral interrupt is masked
  while paused. `Rng_Get_ExecState()` reads active also while paused.
- `Rng_Clear_Error( SEED_ERR )` performs the recovery sequence of the RNG version (see the table).

### Typical usage

```c
#include "Rng_Port.h"

static void App_RandomReady( rng_RandomData_t value )
{
    // 4 x 32-bit random words
}

void App_RngInit( void )
{
    rng_ConfigStruct_t rngConfig;

    (void)Rng_Get_DefaultConfig( &rngConfig );  // HSI48 (started by RCC), NIST, blocking
    rngConfig.TransferStyle   = RNG_TRANSFER_INTERRUPT;
    rngConfig.IrqPrio         = 5u;
    rngConfig.DataRdyCallback = App_RandomReady;

    if( RNG_REQUEST_OK == Rng_Init( &rngConfig ) )
    {
        (void)Rng_Set_ExecActive( RNG_PERIPH_1 );
    }
}
```

---

## Public Types

### Version Types
- `rng_MajorVersion_t` – Major version number (uint8_t).  
- `rng_MinorVersion_t` – Minor version number (uint8_t).  
- `rng_PatchVersion_t` – Patch version number (uint8_t).  
- `rng_ModuleVersion_t` – Struct holding `{ Major, Minor, Patch }`.

### State & Status Types
- `rng_FunctionState_t` – Function active/inactive state.  
- `rng_FlagState_t` – Generic flag state.  
- `rng_RequestState_t` – Request processing result (`RNG_REQUEST_OK` / `RNG_REQUEST_ERROR`).  

### Identification & Configuration Types
- `rng_PeriphId_t` – Peripheral identifier (e.g. `RNG_PERIPH_1`).  
- `rng_ClkSource_t` – RNG kernel clock source selection (H7: `HSI48`, `PLL1Q`, `LSE`, `LSI`).  
- `rng_ClkDivider_t` – RNG clock divider selection.  
- `rng_ConfigSelection_t` – RNG mode configuration (e.g. `NIST`, `AIS_31_SPEED`, `AIS_31_ENTROPY`).  
- `rng_Error_t` – Error mask (e.g. `SEED_ERR`, `CLOCK_ERR`).  
- `rng_TransferStyle_t` – Data transfer style (none, blocking, interrupt).  

### Data Types
- `rng_RandomData_t` – Random data output container (`uint32_t value[RNG_FIFO_LEN]`).  
- `rng_IrqPrio_t` – IRQ priority type.  
- `rng_DataRdyCallback_t` – Data ready callback type.  
- `rng_ErrCallback_t` – Error callback type.  

### Configuration Structure
- `rng_ConfigStruct_t` – Aggregates all peripheral configuration fields:  
  - `PeriphId`  
  - `ClkSource`  
  - `ConfigSelection`  
  - `TransferStyle`  
  - `IrqPrio`  
  - `DataRdyCallback`  
  - `ErrorCallback`  

---

## Public API

### Module Information
```c
rng_ModuleVersion_t Rng_Get_ModuleVersion(void);
```

### Initialization
```c
rng_RequestState_t Rng_Init(rng_ConfigStruct_t * const rngConfig);
rng_RequestState_t Rng_Deinit(rng_ConfigStruct_t * const rngConfig);
void               Rng_Task(void);
rng_RequestState_t Rng_Get_DefaultConfig(rng_ConfigStruct_t *rngConfig);
```

### Execution Control
```c
rng_RequestState_t Rng_Set_ExecActive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Set_ExecInactive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Get_ExecState(rng_PeriphId_t rngId, rng_FunctionState_t * const state);

rng_RequestState_t Rng_Set_ExecPauseActive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Set_ExecPauseInactive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Get_ExecPauseState(rng_PeriphId_t rngId, rng_FunctionState_t * const state);
```

### Data Access
```c
rng_RequestState_t Rng_Get_Data(rng_PeriphId_t rngId, rng_RandomData_t * const data);
rng_RequestState_t Rng_Get_DataReadyState(rng_PeriphId_t rngId, rng_FunctionState_t * const state);
```

### Configuration
```c
rng_RequestState_t Rng_Set_Configuration(rng_PeriphId_t rngId, rng_ConfigSelection_t configSelection);
rng_RequestState_t Rng_Get_Configuration(rng_PeriphId_t rngId, rng_ConfigSelection_t * const configSelection);

rng_RequestState_t Rng_Set_ClkSource(rng_PeriphId_t rngId, rng_ClkSource_t clkSource);

rng_RequestState_t Rng_Set_ClkDivider(rng_PeriphId_t rngId, rng_ClkDivider_t clkDivider);
rng_RequestState_t Rng_Get_ClkDivider(rng_PeriphId_t rngId, rng_ClkDivider_t * const clkDivider);
```

### Error Handling
```c
rng_RequestState_t Rng_Get_Error(rng_PeriphId_t rngId, rng_Error_t * const errMask);
rng_RequestState_t Rng_Clear_Error(rng_PeriphId_t rngId, rng_Error_t errMask);

rng_RequestState_t Rng_Set_ClockErrorDetectActive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Set_ClockErrorDetectInactive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Get_ClockErrorDetectState(rng_PeriphId_t rngId, rng_FunctionState_t * const state);

rng_RequestState_t Rng_Set_AutoResetActive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Set_AutoResetInactive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Get_AutoResetState(rng_PeriphId_t rngId, rng_FunctionState_t * const reqState);
```

### Callback Handling
```c
rng_RequestState_t Rng_Set_DataRdyCallback(rng_PeriphId_t rngId, rng_DataRdyCallback_t * const callback);
rng_RequestState_t Rng_Get_DataRdyCallback(rng_PeriphId_t rngId, rng_DataRdyCallback_t ** const callback);

rng_RequestState_t Rng_Set_ErrorCallback(rng_PeriphId_t rngId, rng_ErrCallback_t * const callback);
rng_RequestState_t Rng_Get_ErrorCallback(rng_PeriphId_t rngId, rng_ErrCallback_t ** const callback);
```

### Interrupt Handling
```c
rng_RequestState_t Rng_Set_IrqActive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Set_IrqInactive(rng_PeriphId_t rngId);
rng_RequestState_t Rng_Get_IrqState(rng_PeriphId_t rngId, rng_FunctionState_t * const reqState);

rng_RequestState_t Rng_Set_IrqPriority(rng_PeriphId_t rngId, rng_IrqPrio_t irqPrio);
rng_RequestState_t Rng_Get_IrqPriority(rng_PeriphId_t rngId, rng_IrqPrio_t * const irqPrio);
```

---

## 🛠 CMake Integration

1. Include `Rng_Lib` in your CMake library.
2. Include `Rng_Port.h` in your project.
3. Link against the Rng module implementation files.
4. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
