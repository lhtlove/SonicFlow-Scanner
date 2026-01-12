# Legacy Code Directory

This directory contains **outdated proof-of-concept firmware** from the initial development phase of the SonicFlow Scanner project. The code here is preserved for reference but is no longer actively maintained.

## Contents

### Core_SMC
STM32F103C8-based carriage controller firmware (early version)
- **Hardware**: Custom 44×23mm controller board
- **Purpose**: Stepper motor control, dual DC motors with encoders, CAN communication
- **Status**: Superseded by updated firmware architecture
- Includes: `SMC.ioc` (STM32CubeMX configuration file)

### Core_SonicFlow
STM32G431CB-based sensor module firmware (early version)
- **Hardware**: Custom 28×52mm sensor board
- **Purpose**: AD9833 DDS control, ultrasonic TX/RX, signal processing
- **Status**: Superseded by updated firmware architecture
- Includes: `Sonic Flow.ioc` (STM32CubeMX configuration file)

### peripheral_SMC
Peripheral driver libraries for motor control and CAN communication
- Motor abstraction layer (stepper and DC motors)
- TMC2209 stepper driver configuration
- MT6701 magnetic encoder interface
- CAN bus communication handlers

---

## Why This Code Is Preserved

This legacy firmware represents the **functional proof-of-concept** that validated:
- Custom DC motor positioning algorithm (phase-based, not PID)
- PCB slip ring reliability for CAN bus communication
- Multi-motor coordination and control
- STM32 dual-controller architecture

While the implementation here worked, the project has evolved toward a more maintainable architecture with improved modularity and documentation.

---

## Current Development

Active development has moved to the main repository structure:
- `/firmware/` - Current firmware implementations
- `/hardware/` - PCB designs and mechanical CAD files
- `/software/` - Host control and reconstruction algorithms

For up-to-date code and documentation, see the main README.md in the repository root.

---

## Notes for Developers

**If you're looking to understand the project**, start with:
1. Main repository README.md
2. `/docs/` directory for current documentation
3. Current firmware in `/firmware/` (when available)

**If you need to reference legacy implementations**:
- Motor control algorithms: `peripheral_SMC/Src/motors.cpp`
- TMC driver config: `peripheral_SMC/Src/stepper_setting.cpp`
- CAN protocol: `peripheral_SMC/Src/can.cpp`
- Encoder handling: `peripheral_SMC/Src/encoders.cpp`

---

*This code is retained for historical reference and to document the development journey. It demonstrates working implementations but may not follow current best practices or architecture.*
