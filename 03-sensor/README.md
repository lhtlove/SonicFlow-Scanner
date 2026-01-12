# Sensor Module

High-frequency ultrasonic transceiver for time-of-flight measurement and signal processing.

## Overview

The sensor module is the core measurement system, featuring 300kHz ultrasonic operation with DDS-based waveform generation and phase-accurate time-of-flight measurement. It combines transmission, reception, amplification, and signal processing in a compact package.

## Key Specifications

- **Operating Frequency**: 300 kHz (research-grade range)
- **Waveform Generation**: DDS (AD9837)
- **Controller**: STM32G431 (with hardware FPU)
- **Board Dimensions**: 28×52 mm
- **TX Channels**: 4 (switchable)
- **RX Channels**: 4 (switchable)
- **ToF Measurement**: Phase-based
- **Theoretical Precision**: ~100 ¼m (with thermal compensation)

## Directory Contents

### `/hardware`
PCB designs for the sensor board:
- STM32G431 sensor board (KiCad project)
- AD9837 DDS circuit
- BJT push-pull transducer driver
- Two-stage amplifier with Bessel filtering
- TX/RX switching circuitry
- 4-channel multiplexing

### `/firmware`
STM32G431 firmware for signal processing:
- DDS configuration and control
- TX/RX timing and switching
- ADC sampling and processing
- Phase detection algorithms
- CAN communication
- See `/06-legacy/Core_SonicFlow/` for reference implementation

### `/model`
3D CAD files for sensor housing:
- Sensor enclosure (STL, STEP)
- Transducer mounting brackets
- Cable strain relief
- Mounting interface for carriage

## Key Features

### Signal Generation
- **AD9837 DDS**: Precise frequency control with 28-bit resolution
- **16 MHz Reference Clock**: Enables accurate frequency synthesis
- **BJT Push-Pull Driver**: High-voltage output for transducer excitation
- **Programmable Phase**: For measurement calibration

### Signal Reception
- **Two-Stage Amplification**: Captures microvolt-level signals
- **Bessel Filtering**: Preserves waveform shape for phase detection
- **4-Channel Multiplexing**: Multiple transducers per board
- **High Signal Integrity**: Careful PCB layout minimizes interference

### Processing
- **STM32G431**: Hardware FPU for efficient DSP operations
- **Phase-Based ToF**: More accurate than simple threshold detection
- **Real-Time Processing**: Minimizes latency for feedback control

## Technical Highlights

### Signal Integrity Challenge
300kHz ultrasonic reception requires capturing microvolt signals while high-voltage TX signals share the same board. Solution:
- Multiple filter iterations
- Bessel filter configuration for clean capture
- Careful ground plane management
- TX/RX isolation switching

### Thermal Compensation
The system inverts the traditional temperature-sensitivity weakness:
1. Measures ToF between known positions
2. Calculates sound velocity variations
3. Builds tomographic thermal map
4. Uses map to compensate proximity measurements

## Assembly Notes

- Requires careful SMD soldering (fine-pitch components)
- Transducers must be mechanically isolated from board vibrations
- Test TX waveform quality before full integration
- Verify RX amplifier gain stages individually
- See `/05-docs/` for complete build and testing guide
