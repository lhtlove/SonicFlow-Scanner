# Motion Shell

Ring-shaped mechanical framework providing continuous rotation and tilt capabilities for the scanning system.

## Overview

The motion shell is the primary structural component that houses the carriage module and enables multi-axis scanning motion. It features a 444mm outer diameter with integrated gear tracks and custom PCB slip rings for continuous power and communication during infinite rotation.

## Key Specifications

- **Outer Diameter**: 444 mm
- **Rotation**: Continuous 360° (infinite)
- **Tilt Range**: 0-90°
- **Material**: FDM-printed segments
- **Communication**: CAN bus through PCB slip rings

## Directory Contents

### `/hardware`
PCB designs for the motion shell components:
- Slip ring PCB designs (ENIG-plated)
- Connector boards
- Power distribution

### `/firmware`
Firmware for motion shell controllers (if applicable):
- Shell rotation control
- Tilt mechanism control
- Position feedback

### `/model`
3D CAD files for mechanical components:
- Shell segments (STL, STEP)
- Gear track assemblies
- Slip ring mechanical housing
- Mounting brackets

## Features

- **Segmented Design**: Designed for FDM printing in manageable segments
- **Fully Connected Rails**: Integrated carriage rails with gear tracks
- **PCB Slip Rings**: Custom ENIG-plated design for reliable power/data transfer
- **Stepper-Driven**: Precise positioning via gearbox-coupled steppers
- **Half-Ring Folding**: Access capability for module installation/maintenance

## Assembly Notes

- Segments bolt together to form complete ring
- Slip ring requires careful alignment during assembly
- Gear tracks must be aligned for smooth carriage travel
- See main assembly documentation in `/05-docs/` for detailed instructions
