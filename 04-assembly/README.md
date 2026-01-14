# Assembly Documentation

Comprehensive assembly guides and documentation for building the complete SonicFlow Scanner system.

## Overview

This directory contains step-by-step assembly instructions, integration guides, and troubleshooting documentation for the complete scanner system. Follow the assembly order carefully to avoid having to disassemble completed sections.

## Directory Contents

### `/hardware`
Assembly-related hardware documentation:
- Fastener specifications and BOMs
- Connector pinouts and wiring diagrams
- Calibration fixture designs
- Testing jigs

### `/firmware`
System integration firmware:
- Multi-controller coordination
- CAN protocol definitions
- Calibration routines
- System-level test programs

### `/model`
Assembly aid models:
- Alignment jigs and fixtures
- Assembly sequence visualizations
- Exploded view diagrams
- Tool templates

## Recommended Assembly Order

### 1. Motion Shell Assembly
- Print and prepare shell segments
- Install slip ring PCBs
- Assemble gear tracks
- Connect and test rotation mechanism
- **Verify**: Smooth rotation, slip ring continuity

### 2. Carriage Module Assembly
- Assemble carriage structure
- Install controller board
- Mount motors and encoders
- Route and secure cables
- **Verify**: Motor control, encoder readings via debug interface

### 3. Sensor Module Assembly
- Solder sensor board
- Test TX waveform generation
- Test RX amplification
- Install in housing
- **Verify**: Signal quality on oscilloscope

### 4. System Integration
- Install carriage module on motion shell
- Connect slip ring contacts
- Install sensor module on carriage
- Complete cable routing
- **Verify**: CAN communication between all controllers

### 5. Calibration
- Position calibration (encoders and steppers)
- Sensor alignment calibration
- Timing calibration for ToF measurements
- **Verify**: Consistent measurements at known positions

## Tools Required

### For Mechanical Assembly
- 3D printer (250mm+ build volume)
- Hex key set (M2-M5)
- Small screwdrivers
- Calipers for measurement
- Alignment fixtures (included in `/model`)

### For Electronics
- Soldering station with fine tip
- Hot air station (for QFN packages)
- Multimeter
- Oscilloscope (recommended: 100 MHz+)
- Logic analyzer (for CAN bus debugging)
- ST-Link programmer

### For Testing
- Power supply (5V, 2A minimum)
- CAN bus analyzer (optional but helpful)
- Function generator (for testing)
- Test transducers or loads

## Safety Notes

- **High Voltage**: TX driver generates high-voltage pulses - avoid contact
- **Moving Parts**: Motion shell has pinch points during rotation
- **Soldering**: Use proper ventilation for lead-based solder
- **Power Supply**: Verify polarity before connecting boards

## Troubleshooting

Common issues and solutions will be documented as they're discovered during builds. Check `/05-docs/technical-notes/` for detailed troubleshooting guides.

## Documentation Links

Detailed assembly guides for each subsystem:
- Motion Shell Assembly ’ `/01-motion-shell/README.md`
- Carriage Module Assembly ’ `/02-carriage-module/README.md`
- Sensor Module Assembly ’ `/03-sensor/README.md`
- System Integration ’ See `/05-docs/` for comprehensive guides

## Testing and Validation

### Unit Tests
- Each module should be tested independently before integration
- Firmware test programs available in respective `/firmware` directories

### Integration Tests
- CAN bus communication test
- Multi-axis motion coordination test
- End-to-end ToF measurement test

### Calibration Procedures
- Position accuracy calibration
- Timing calibration for phase measurements
- Thermal compensation validation

## Support

For build questions or issues:
- Check troubleshooting section in `/05-docs/`
- Review legacy firmware in `/06-legacy/` for working examples
- Open an issue on GitHub with photos/logs
