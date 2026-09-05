# RNG Module

This module provides an abstraction for controlling and retrieving data from the hardware Random Number Generator (RNG).  
It offers initialization, configuration, execution control, error handling, and data retrieval functionality.

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
- `rng_ClkSource_t` – RNG clock source selection.  
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
