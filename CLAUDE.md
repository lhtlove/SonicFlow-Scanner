# CLAUDE.md - SonicFlow-Scanner Project Guide

## Project Overview

**SonicFlow-Scanner** is an embedded ultrasound scanning system built on a dual-microcontroller architecture using STM32 microcontrollers. The project combines precise motor control, signal generation, and data acquisition for ultrasound scanning applications.

### Key Technologies
- **Hardware Platform**: Dual STM32 microcontrollers (STM32F1 + STM32G4)
- **Programming Languages**: C and C++ (mixed codebase)
- **HAL Library**: STM32 Hardware Abstraction Layer (HAL)
- **Development Tool**: STM32CubeMX for hardware configuration
- **Build System**: IDE-based (likely STM32CubeIDE or Keil)
- **Communication**: CAN/FDCAN bus for inter-controller messaging

---

## Repository Structure

```
SonicFlow-Scanner/
├── Core_SMC/                    # Stepper Motor Controller (STM32F103C8)
│   ├── Inc/                     # Header files
│   │   ├── main.h              # Main header with peripheral handles
│   │   ├── stm32f1xx_hal_conf.h # HAL configuration
│   │   └── stm32f1xx_it.h      # Interrupt handlers header
│   ├── Src/                     # Source files
│   │   ├── main.cpp            # Main application (C++)
│   │   ├── stm32f1xx_hal_msp.c # MSP initialization
│   │   ├── stm32f1xx_it.c      # Interrupt handlers
│   │   ├── syscalls.c          # System calls
│   │   ├── system_stm32f1xx.c  # System initialization
│   │   └── sysmem.c            # Memory management
│   └── Startup/
│       └── startup_stm32f103c8tx.s # Startup assembly code
│
├── Core_SonicFlow/              # Main Controller (STM32G431CB)
│   ├── Inc/                     # Header files
│   │   ├── main.h              # Main header
│   │   ├── stm32g4xx_hal_conf.h # HAL configuration
│   │   └── stm32g4xx_it.h      # Interrupt handlers header
│   ├── Src/                     # Source files
│   │   ├── main.c              # Main application (C)
│   │   ├── stm32g4xx_hal_msp.c # MSP initialization
│   │   ├── stm32g4xx_it.c      # Interrupt handlers
│   │   ├── syscalls.c          # System calls
│   │   ├── system_stm32g4xx.c  # System initialization
│   │   └── sysmem.c            # Memory management
│   └── Startup/
│       └── startup_stm32g431cbtx.s # Startup assembly code
│
├── peripheral_SMC/              # Peripheral drivers for SMC
│   ├── Inc/                     # Peripheral headers
│   │   ├── can.hpp             # CAN communication
│   │   ├── encoders.hpp        # Encoder interface
│   │   ├── motors.hpp          # Motor control (stepper & DC)
│   │   └── stepper_setting.hpp # TMC stepper driver config
│   └── Src/                     # Peripheral implementations
│       ├── can.cpp
│       ├── encoders.cpp
│       ├── motors.cpp
│       └── stepper_setting.cpp
│
├── SMC.ioc                      # STM32CubeMX project for SMC
├── Sonic Flow.ioc               # STM32CubeMX project for SonicFlow
└── README.md                    # Project readme
```

---

## Hardware Architecture

### Core_SMC (Stepper Motor Controller) - STM32F103C8

**Purpose**: Dedicated motor control and positioning system

**Peripherals**:
- **CAN**: Inter-controller communication (500 kbps)
- **I2C**: MT6701 absolute magnetic encoder communication
- **UART3**: TMC stepper driver configuration (115200 baud)
- **TIM1**: Microsecond delay timer (prescaler: 16-1, 1µs resolution)
- **TIM2**: Stepper motor pulse generation
- **TIM3/TIM4**: DC motor control timers
- **GPIO**: Motor direction, step pulses, encoder inputs

**Key Functions**:
- Stepper motor control with configurable speed and direction
- Dual DC motor control with encoder feedback
- Position tracking via I2C absolute encoder (MT6701)
- CAN-based command reception from main controller
- TMC stepper driver configuration (UART interface)

**Motor Control Features**:
- Stepper: Step/direction interface, configurable microstepping
- DC Motors: PWM speed control, quadrature encoder feedback
- Position tracking and closed-loop control capability

### Core_SonicFlow (Main Controller) - STM32G431CB

**Purpose**: Signal generation, data acquisition, and system coordination

**Peripherals**:
- **FDCAN**: CAN FD communication for high-speed data transfer
- **ADC1**: 12-bit signal acquisition (OPAMP buffered)
- **OPAMP1/2/3**: Signal conditioning (PGA mode, configurable gain)
- **SPI2**: AD9833 DDS waveform generator control
- **TIM1**: Microsecond delay timer (prescaler: 16-1)
- **GPIO**: Multiplexer control, receiver channel selection

**Key Functions**:
- Ultrasound signal generation via AD9833 DDS (up to 300 kHz sine waves)
- Multi-channel signal acquisition with programmable gain amplification
- Channel multiplexing for multi-element scanning
- CAN FD communication for control and data transfer
- Frequency Tuning Word (FTW) calculation for precise frequency control

**Signal Generation Features**:
- AD9833 DDS configuration via SPI
- Sine wave generation with programmable frequency
- 16 MHz reference clock with 28-bit FTW resolution
- Phase and frequency registers control

---

## Key Conventions & Code Patterns

### STM32CubeMX Integration

The project uses STM32CubeMX for hardware configuration. **IMPORTANT**:

1. **USER CODE sections**: All custom code must be placed within designated sections:
   ```c
   /* USER CODE BEGIN [section_name] */
   // Your custom code here
   /* USER CODE END [section_name] */
   ```

2. **Never modify generated code** outside USER CODE sections - it will be overwritten when regenerating from .ioc files

3. **Common USER CODE sections**:
   - `USER CODE BEGIN Includes` - Additional headers
   - `USER CODE BEGIN PV` - Private variables
   - `USER CODE BEGIN PFP` - Private function prototypes
   - `USER CODE BEGIN 0` - User code before main loop
   - `USER CODE BEGIN 2` - Initialization code
   - `USER CODE BEGIN 3` - Main loop code
   - `USER CODE BEGIN 4` - Additional functions

### Coding Style

**Language Usage**:
- Core_SMC: **C++ (.cpp)** for main.cpp, peripheral libraries use C++
- Core_SonicFlow: **C (.c)** for main.c
- Peripheral drivers: Primarily C++ with HAL C library integration

**Naming Conventions**:
- Peripheral handles: `h[peripheral][number]` (e.g., `hcan`, `htim1`, `hspi2`)
- Constants: `UPPER_CASE_WITH_UNDERSCORES`
- Functions: `Module_Action()` format (e.g., `Stepper_Move()`, `DC_Init()`)
- Struct typedefs: PascalCase (e.g., `Stepper`, `DCM`, `CANHandler`)

**Header Guards**:
- Use `#ifndef MODULE_INC_` pattern
- Example: `#ifndef MOTORS_INC_`

### Peripheral Abstraction Patterns

**Motor Control**:
```cpp
// Stepper motor structure
typedef struct {
    TIM_HandleTypeDef *htim_stepper;
    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    uint32_t steps_to_go;
    uint32_t current_steps;
    uint8_t dir;
    uint8_t is_running;
} Stepper;

// Usage pattern
void Stepper_Init(TIM_HandleTypeDef *htim);
void Stepper_Move(uint32_t steps, uint8_t dir);
void Stepper_SetSpeed(uint32_t steps_per_second);
```

**CAN Communication**:
```cpp
// Device ID and masking
#define DEVICE_ID 0x001
#define MASK_SINGLE 0x7FF    // Listen only to specific ID
#define MASK_PARTICULAR 0x700 // Listen to device group
#define MASK_ALL 0x000       // Listen to all messages

// Initialization pattern
CAN_INIT(&hcan, DEVICE_ID, MASK_SINGLE);
```

**TMC Stepper Driver Configuration**:
```cpp
// Configuration structures use bitfields
union GlobalConfig {
    struct {
        uint32_t i_scale_analog : 1;
        uint32_t internal_rsense : 1;
        // ... more config bits
    };
    uint32_t bytes;
};

// Write to driver via UART
write(ADDRESS_GCONF, global_config_.bytes, &huart3);
```

### Interrupt Handling

**Timer Callbacks**:
```cpp
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == stepper1.htim_stepper->Instance)
        Stepper_TIM_Interrupt();
    else if(htim->Instance == DCL.htim_dc->Instance)
        DC_TIM_Interrupt(&DCL);
    else if(htim->Instance == DCR.htim_dc->Instance)
        DC_TIM_Interrupt(&DCR);
}
```

**Pattern**: Check timer instance, call appropriate handler

---

## Development Workflow

### Building the Project

1. **IDE Setup**: Project is designed for STM32CubeIDE or compatible ARM GCC toolchain
2. **STM32CubeMX**: Hardware configuration stored in `.ioc` files
3. **Build Process**:
   - Open respective `.ioc` file in STM32CubeMX
   - Generate code if hardware configuration changes
   - Build using IDE's build system
   - Each core (SMC and SonicFlow) compiles separately

### Modifying Hardware Configuration

1. Open `.ioc` file in STM32CubeMX (SMC.ioc or Sonic Flow.ioc)
2. Modify peripheral settings, pin assignments, clock configuration
3. Generate code (preserves USER CODE sections)
4. Review changes in `main.h`, `main.c/cpp`, and `*_it.c` files
5. Add custom initialization in USER CODE BEGIN 2 section
6. Add runtime code in USER CODE BEGIN 3 or appropriate callbacks

### Adding New Peripheral Drivers

**For SMC peripherals**, follow this pattern:

1. Create header file in `peripheral_SMC/Inc/[module].hpp`
   ```cpp
   #ifndef MODULE_INC_
   #define MODULE_INC_

   #include "stm32f1xx_hal.h"

   // Function declarations
   void Module_Init(/* params */);
   void Module_Process(/* params */);

   #endif
   ```

2. Create implementation in `peripheral_SMC/Src/[module].cpp`
3. Include in `Core_SMC/Src/main.cpp` within USER CODE BEGIN Includes
4. Initialize in USER CODE BEGIN 2
5. Use in main loop or callbacks

### Testing Motor Control

**Stepper Motor Test Pattern**:
```cpp
Stepper_Init(&htim2);
stepper_encoder_init(&hi2c1);
Stepper_SetSpeed(15);  // steps per second
Stepper_Move(100000, 0);  // steps, direction
```

**DC Motor Test Pattern**:
```cpp
DC_Init(&htim3, 0);  // timer, motor ID
dc_encoder_reset();
DC_Move(0, 1200, 0, 1);  // id, counts, direction, back_flag
```

### Signal Generation Testing

**AD9833 Sine Wave Pattern**:
```c
sineWave(CalcFTW(300000));  // Generate 300 kHz sine wave
```

**Frequency Tuning Word Calculation**:
```c
uint32_t FTW = (0x10000000 / 16000000) * frequency;
```

---

## Common Tasks for AI Assistants

### 1. Adding New Motor Control Features

**Context**: Motor control is centralized in peripheral_SMC module
**Files to modify**:
- `peripheral_SMC/Inc/motors.hpp` - Add function declarations
- `peripheral_SMC/Src/motors.cpp` - Implement functionality
- `Core_SMC/Src/main.cpp` - Add calls in USER CODE sections

**Important**:
- Use timer interrupts for precise timing
- Update motor state machines in interrupt handlers
- Maintain backward compatibility with existing motor structures

### 2. Implementing CAN Communication Handlers

**Context**: CAN handlers process incoming messages
**Pattern**:
```cpp
void custom_can_handler(uint8_t* data, uint8_t len) {
    // Parse data
    // Execute actions (motor commands, configuration)
    // Send response if needed via CAN_Tx()
}

// Register handler
CANHandler handlers[] = {
    {.id = 0x001, .handler = custom_can_handler},
    // ... more handlers
};
```

**Files**: `peripheral_SMC/Inc/can.hpp`, `peripheral_SMC/Src/can.cpp`

### 3. Modifying Signal Acquisition

**Context**: ADC with OPAMP gain stages on SonicFlow core
**Files to modify**:
- `Core_SonicFlow/Src/main.c` - ADC configuration and processing
- Adjust OPAMP gain in `MX_OPAMP[1-3]_Init()` functions

**Gain Settings**:
- OPAMP1/2: `OPAMP_PGA_GAIN_8_OR_MINUS_7` (8x gain)
- OPAMP3: `OPAMP_PGA_GAIN_2_OR_MINUS_1` (2x gain)

### 4. Debugging Timing Issues

**Microsecond Delay Function**:
```c
void delay_us(uint16_t time) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while((__HAL_TIM_GET_COUNTER(&htim1)) < time);
}
```

**Timer Configuration**:
- TIM1: Prescaler = 16-1 → 1 MHz tick rate (1µs resolution)
- Period: 65535 (max delay ~65ms)

**Important**: Never use blocking delays in interrupt handlers

### 5. Encoder Position Reading

**I2C Absolute Encoder (MT6701)**:
```cpp
#define MT6701_ADDR (0x06 << 1)  // I2C address
#define REG_ANGLE_HIGH 0x03
#define REG_ANGLE_LOW 0x04

float angle_rad = stepper_encoder_angle(&hi2c1, RAD);
```

**Quadrature Encoders (DC motors)**:
```cpp
// Update in GPIO interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == E1_B_Pin) dc_encoder_update(0, DCL.dir);
    if (GPIO_Pin == E2_B_Pin) dc_encoder_update(1, DCR.dir);
}
```

---

## Critical Warnings

### 1. STM32CubeMX Code Regeneration
- **NEVER** modify code outside USER CODE sections
- Always backup before regenerating from .ioc files
- Custom code in non-USER sections WILL BE LOST

### 2. Interrupt Safety
- Keep ISRs short and fast
- Use flags for main loop processing
- Avoid HAL_Delay() in interrupts
- Be cautious with floating-point in ISRs (slower on Cortex-M)

### 3. CAN Bus Communication
- Verify bit timing matches all devices on bus (500 kbps standard)
- Check termination resistors (120Ω at each end)
- Use appropriate mask for filtering unwanted messages
- Handle bus-off conditions and errors

### 4. Motor Control Safety
- Implement limit switches or software position limits
- Add overcurrent protection checks
- Emergency stop functionality should bypass normal state machines
- Verify direction before high-speed moves

### 5. TMC Driver Configuration
- UART communication is unidirectional write-only in current setup
- Verify motor current settings don't exceed driver specs
- CRC calculation required for reliable communication
- Allow 100µs delay between write operations (`TMC_WRITE_READ_DELAY`)

---

## Hardware-Specific Notes

### STM32F103C8 (Core_SMC) Limitations
- 64KB Flash, 20KB RAM
- No FPU - avoid floating-point in time-critical paths
- Limited timer resources - share wisely
- 72 MHz max clock (currently running at HSE 16 MHz with no PLL)

### STM32G431CB (Core_SonicFlow) Capabilities
- 128KB Flash, 32KB RAM
- Hardware FPU - floating-point efficient
- Advanced timers with better PWM resolution
- FDCAN supports CAN FD for higher bandwidth
- Multiple OPAMPs for analog signal conditioning
- Currently running at HSE 16 MHz (PLL disabled)

---

## File Modification Guidelines

### High-Frequency Changes
These files are regularly modified for feature development:
- `Core_SMC/Src/main.cpp` (USER CODE sections only)
- `Core_SonicFlow/Src/main.c` (USER CODE sections only)
- `peripheral_SMC/Src/*.cpp` - peripheral implementations
- `peripheral_SMC/Inc/*.hpp` - peripheral interfaces

### Low-Frequency Changes
Modify only when hardware configuration changes:
- `*.ioc` files - via STM32CubeMX GUI
- `stm32f1xx_hal_conf.h` / `stm32g4xx_hal_conf.h` - HAL config
- Startup assembly files - rarely needed

### Never Modify Directly
- Generated initialization functions (MX_*_Init)
- System files (system_stm32*.c, syscalls.c, sysmem.c)
- HAL library files
- Code outside USER CODE sections

---

## Debugging Tips

### Common Issues

**Issue**: CAN communication not working
- Check: Bit timing configuration matches network
- Check: CAN transceiver high-speed mode (CAN_SPEED pin)
- Check: Filter configuration and device ID
- Verify: Termination resistors present

**Issue**: Stepper motor not moving
- Check: Timer started (`HAL_TIM_Base_Start_IT(&htim2)`)
- Check: TMC driver configuration written via UART
- Check: GPIO pins toggling (oscilloscope/logic analyzer)
- Verify: Motor power supply and current settings

**Issue**: Encoder reading errors
- Check: I2C communication success (HAL_OK return)
- Check: Pull-up resistors on I2C lines
- Verify: MT6701 I2C address (0x06 default)
- Check: Encoder power supply

**Issue**: ADC readings incorrect
- Check: OPAMP gain settings match signal levels
- Check: ADC reference voltage (VREF+)
- Verify: Input signal within valid range after amplification
- Check: Sampling time adequate for source impedance

### Debug Output

**UART3 on SMC**:
- Currently used for TMC driver configuration
- Can repurpose for debug with conditional compilation
- Baud rate: 115200

**Debugging without UART**:
- Toggle GPIO pins to signal events
- Use timer capture for timing measurements
- Monitor variables via debugger (ST-Link)

---

## Additional Resources

### STM32 Documentation
- STM32F103C8 Reference Manual (RM0008)
- STM32G431CB Reference Manual (RM0440)
- STM32 HAL Driver Documentation
- STM32CubeMX User Manual

### Component Datasheets
- TMC2209 Stepper Driver Datasheet
- MT6701 Magnetic Encoder Datasheet
- AD9833 DDS Waveform Generator Datasheet

### Development Tools
- STM32CubeIDE (recommended IDE)
- STM32CubeMX (hardware configuration)
- STM32CubeProgrammer (flashing/debugging)
- ST-Link debugger interface

---

## Project Status & Notes

**Current State**:
- Basic motor control implemented and tested
- CAN communication framework in place
- Signal generation functional (AD9833)
- Main control loops contain test/debug code

**Test Code Present**:
- Core_SMC main loop has GPIO toggle test code (lines 193-197)
- Many sections commented out for testing individual features
- Production deployment will require cleanup of test code

**Development Focus Areas**:
- Motor control refinement and closed-loop positioning
- CAN protocol definition for inter-controller messaging
- ADC data acquisition and processing pipeline
- System integration and synchronization

---

## Quick Reference

### Pin Mappings (SMC)
- STEP_Pin: PB0 (Stepper step pulse)
- DIR_Pin: PB1 (Stepper direction)
- Motor1_1/2: PA0/PA1 (DC motor 1 H-bridge)
- Motor2_1/2: PA4/PA5 (DC motor 2 H-bridge)
- E1_A/B: PA11/PA12 (Encoder 1)
- E2_A/B: PB4/PB5 (Encoder 2)

### Pin Mappings (SonicFlow)
- FG_FSYNC: PB12 (AD9833 chip select)
- SEL_IN1-4: PB13-15, PB9 (Multiplexer control)
- RECV_A0/A1: PA2/PA3 (Receiver channel select)
- RECV_EN: PA4 (Receiver enable)

### Key Constants
```c
// CAN
#define DEVICE_ID 0x001
#define MASK_SINGLE 0x7FF

// Encoder
#define MT6701_ADDR (0x06 << 1)

// TMC Registers
#define ADDRESS_GCONF 0x00
#define ADDRESS_CHOPCONF 0x6C
#define ADDRESS_IHOLD_IRUN 0x10

// Timing
TIM1 prescaler: 16-1 (1 MHz, 1µs ticks)
```

---

*Last Updated: 2024-11-24*
*Repository: SonicFlow-Scanner*
*Target Platform: STM32F103C8 + STM32G431CB*
