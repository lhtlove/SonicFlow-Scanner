# Carriage Module

Compact sensor positioning unit with 3-axis motion control (linear rail + dual-axis sensor orientation).

## Overview

The carriage module is a highly integrated mechatronic system that positions the ultrasonic sensor module within the motion shell. It combines linear motion along the shell's circumference with dual-axis rotational control for precise sensor orientation.

## Key Specifications

- **Dimensions**: 46×30×77 mm
- **Linear Motion**: Full circumference travel on motion shell
- **Sensor Orientation**: ±45° on two axes
- **Motors**: 1× NEMA8 stepper, 2× N20 geared DC motors
- **Controller**: Custom STM32F103-based board (44×23 mm)
- **Communication**: CAN bus via slip ring contacts

## Directory Contents

### `/hardware`
PCB designs for the carriage controller:
- STM32F103 controller board (KiCad project)
- Motor driver circuits
- Encoder interface boards
- Connector layouts

### `/firmware`
STM32F103 firmware for motion control:
- Stepper motor control (linear positioning)
- DC motor positioning algorithm (phase-based, not PID)
- Magnetic encoder reading
- CAN protocol implementation
- See `/06-legacy/Core_SMC/` for reference implementation

### `/model`
3D CAD files for mechanical components:
- Main carriage body (STL, STEP)
- Motor mounts
- Sensor mounting bracket
- Encoder mounting hardware
- Cable management components

## Key Features

- **Tool-Free Installation**: Slip ring contacts for easy module swapping
- **Compact Design**: Three motors + controller in 46×30×77 mm envelope
- **Custom Algorithm**: Phase-based DC motor positioning (solves PID stall issues)
- **Dual Encoders**: Magnetic encoders on both DC motors for closed-loop control
- **Integrated Controller**: Custom PCB fits within carriage envelope

## Technical Highlights

### DC Motor Positioning Algorithm
Standard PID control failed due to torque loss at low PWM. Developed a phase-based algorithm that:
- Maintains near-full PWM output
- Uses encoder feedback to time deceleration phases
- Achieves precise positioning without stalling

### Integration Challenges
- Fitting three motors, two encoders, and control electronics in minimal space
- Managing heat dissipation in compact envelope
- Cable routing through moving joints
- Maintaining assembly access for debugging

## Assembly Notes

- Controller board mounts directly on carriage body
- DC motors require careful alignment for encoder accuracy
- Cable routing must allow full range of motion
- Test encoder functionality before final assembly
- See `/05-docs/` for complete assembly guide
