# SonicFlow Scanner

**CT-Inspired Modular Ultrasonic 3D Scanning System for Additive Manufacturing Feedback**

A modular scanning system that creates 3D thermal distribution maps and performs geometry scanning using ultrasonic measurement principles adapted from computed tomography (CT). Designed to bring comprehensive environmental feedback to 3D printing—enabling a make-check-change workflow rather than blind fabrication.

![Project Status](https://img.shields.io/badge/status-in%20development-yellow)
[![License: CC BY-NC-ND 4.0](https://img.shields.io/badge/License-CC%20BY--NC--ND%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-nd/4.0/)

> **📄 [View Comprehensive Project Overview](https://docs.google.com/document/d/1DoINSkOtGRSBR8vOlWFI2wLNMxFoE6zbKSt7RHld4zc/edit?usp=sharing)**
> Detailed documentation covering design decisions, technical challenges, and development process.
>
> **🔧 [View Interactive 3D Assembly](https://a360.co/4szlqWj)**
> Explore the full system design in 3D (Autodesk Viewer)

---

## The Vision

CAD software lets you design, inspect, modify, and iterate seamlessly. You can undo operations, edit specific features, and refine continuously. 3D printing should work the same way.

Current printers operate essentially blind—they execute instructions without perceiving what they've created or how the environment affects the process. This project builds the feedback infrastructure to change that: a single scanner that captures multiple volumetric environmental data types, enabling not just error detection, but eventually iterative physical editing.

---

## Core Innovation

### The Key Insight

Ultrasonic sensors have a traditional weakness: temperature sensitivity affects sound speed, degrading measurement accuracy. This project **inverts that limitation into a feature**.

By measuring time-of-flight between known positions across multiple angles, the system:
1. Calculates average sound velocity along each measurement path
2. Builds a sinogram from multi-angle measurements
3. Applies inverse Radon transformation to reconstruct a **2D thermal slice**
4. Stacks slices into a **3D thermal map**
5. Uses the thermal map to **compensate for sound speed variations** in proximity measurements

**Result:** A single, cost-effective sensor type provides both environmental awareness (thermal mapping) AND high-precision geometric measurement (calibrated proximity).

```
[Sensor A] ----sound wave---- [Sensor B]
        \                    /
         \  measured: ToF   /
          \ known: distance/
           → calculated: sound speed
                  ↓
        [Multi-angle collection]
                  ↓
            [Sinogram]
                  ↓
      [Inverse Radon Transform]
                  ↓
    [2D Sound Velocity = Temperature Map]
                  ↓
   [Calibrated Proximity Measurement]
                  ↓
         [3D Point Clouds]
```

---

## System Architecture

### Three Core Components

| Component | Description | Key Specs |
|-----------|-------------|-----------|
| **Motion Shell** | Ring-shaped mechanical framework | 444mm OD, continuous 360° rotation, 90° tilt capability |
| **Carriage Module** | Compact sensor positioning unit | 46×30×77mm, 3-axis motion (linear + dual orientation) |
| **Sensor Module** | High-frequency ultrasonic transceiver | 300kHz DDS-based, phase-accurate ToF measurement |

### Motion Shell
![IMG_0400](https://github.com/user-attachments/assets/183e354e-87f1-424b-afd7-a2b4205331a9)
<p align="center">
  <img src="https://github.com/user-attachments/assets/183e354e-87f1-424b-afd7-a2b4205331a9" alt="Motion Shell" width="600">
</p>

- Fully connected carriage rails with integrated gear tracks
- Custom ENIG-plated PCB slip rings for continuous power and CAN bus during infinite rotation
- Stepper-driven gearboxes for precise positioning
- Designed for segmented FDM printing
- Half-ring folding capability for module access

### Carriage Module

<p align="center">
  <img src="docs/images/carriage-module-exploded.jpg" alt="Carriage Module Exploded View" width="500">
</p>

- NEMA8 stepper for linear rail positioning
- Two N20 geared DC motors with magnetic encoders for dual-axis sensor orientation
- Custom STM32F103-based controller board (44×23mm)
- Tool-free installation via slip ring contacts
- Custom DC motor positioning algorithm (phase-based, not PID)

### Sensor Module

<p align="center">
  <img src="docs/images/sensor-board.jpg" alt="Sensor Board" width="400">
</p>

- 300kHz operation (research-grade frequency range)
- AD9837 DDS for precise waveform generation
- BJT push-pull buffer for high-voltage transducer drive
- 4-channel TX/RX switching
- Two-stage amplifier with Bessel filtering for waveform preservation
- STM32G431 for signal processing

---

## Repository Structure

```
SonicFlow-Scanner/
├── README.md
├── docs/
│   ├── images/                    # Project photos and renders
│   ├── schematics/               # Circuit diagrams
│   └── technical-notes/          # Design decisions and calculations
├── hardware/
│   ├── mechanical/
│   │   ├── motion-shell/         # Shell CAD files (STEP, STL)
│   │   ├── carriage-module/      # Carriage CAD files
│   │   └── sensor-housing/       # Sensor enclosure CAD
│   └── electronics/
│       ├── carriage-controller/  # STM32F103 board (KiCad)
│       ├── sensor-board/         # STM32G431 board (KiCad)
│       └── slip-ring/            # Custom PCB slip ring design
├── firmware/
│   ├── carriage-controller/      # Motion control firmware
│   ├── sensor-module/            # Ultrasonic TX/RX firmware
│   └── shared/                   # Common libraries (CAN protocol, etc.)
├── software/
│   ├── reconstruction/           # Tomographic reconstruction algorithms
│   └── host/                     # PC control interface
└── bom/                          # Bill of materials
```

---

## Technical Specifications

### Motion System
| Parameter | Value |
|-----------|-------|
| Shell outer diameter | 444 mm |
| Shell rotation | Continuous 360° |
| Tilt range | 0-90° |
| Carriage linear travel | Full circumference |
| Sensor orientation | ±45° (each axis) |
| Communication | CAN bus (through slip ring) |

### Ultrasonic System
| Parameter | Value |
|-----------|-------|
| Operating frequency | 300 kHz |
| Waveform generation | DDS (AD9837) |
| ToF measurement | Phase-based |
| Theoretical precision | ~100 μm (with thermal compensation) |
| TX channels | 4 (switchable) |
| RX channels | 4 (switchable) |

### Controller Boards
| Board | MCU | Dimensions |
|-------|-----|------------|
| Carriage Controller | STM32F103 | 44×23 mm |
| Sensor Board | STM32G431 | 28×52 mm |

---

## Current Status

### ✅ Completed
- Motion shell mechanical design and assembly
- Carriage module with full motion control
- CAN bus network integration across slip rings
- Sensor board hardware design
- Individual TX and RX validation

### 🔄 In Progress
- Sensor interface integration debugging
- TX/RX switching circuit validation
- ToF measurement accuracy verification

### 📋 Planned
- Tomographic reconstruction pipeline implementation
- Thermal compensation validation
- Printer firmware integration
- Multi-slice 3D scanning demonstration

---

## Key Technical Challenges Solved

### 1. DC Motor Precision Positioning
Standard PID control failed—reducing PWM to slow down also reduces torque, causing stalls. Developed a **phase-based algorithm** that maintains near-full output while using encoder feedback to time deceleration.

### 2. Custom PCB Slip Rings
Commercial slip rings cost hundreds of dollars. Created ENIG-plated PCB rings with spring-loaded ball pin contacts. Key challenge: finding optimal contact pressure (too light = dropout, too heavy = gold wear-through).

### 3. High-Frequency Signal Integrity
300kHz ultrasonic reception requires capturing microvolt signals while high-voltage TX signals share the same board. Multiple filter iterations before Bessel configuration achieved clean signal capture.

### 4. Compact Mechatronics
Fitting three motors, encoders, and control electronics into a 46×30×77mm envelope while maintaining assembly access and thermal management.

---

## Building This Project

### Prerequisites
- FDM 3D printer (250mm+ build volume recommended)
- PCB fabrication (JLCPCB, PCBWay, etc.)
- Basic SMD soldering equipment
- ST-Link programmer

### Hardware
See `/bom` for complete bill of materials.

### Firmware
```bash
# Clone repository
git clone https://github.com/lhtlove/SonicFlow-Scanner.git
cd SonicFlow-Scanner

# Build carriage controller firmware
cd firmware/carriage-controller
make

# Build sensor module firmware
cd ../sensor-module
make
```

---

## Documentation

- [Motion Shell Assembly Guide](docs/assembly-motion-shell.md)
- [Carriage Module Assembly Guide](docs/assembly-carriage.md)
- [Electronics Build Guide](docs/electronics-build.md)
- [Firmware Flashing Guide](docs/firmware-flashing.md)
- [CAN Protocol Reference](docs/can-protocol.md)
- [Reconstruction Algorithm Notes](docs/reconstruction-theory.md)

---

## Media

### Project Photos
See the `/docs/images` directory for:
- Build progress photos
- Assembly documentation
- Test setup images
- Oscilloscope captures

### Videos
- [Motion Shell Rotation Demo](link)
- [Carriage Module Movement Test](link)
- [System Integration Test](link)

---

## Background

This project represents two years of independent development as a high school student, motivated by the need for reliable 3D printing feedback in hardware projects. The work spans mechanical design, electronics, firmware, and signal processing—all developed without institutional resources or formal mentorship.

The core concept emerged from recognizing that a traditional sensor weakness (temperature sensitivity) could be inverted into a capability through tomographic reconstruction principles borrowed from medical CT imaging.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License** (CC BY-NC-ND 4.0).

**What this means:**
- ✅ **You CAN:** Share and redistribute this project with proper attribution
- ❌ **You CANNOT:** Use this project or its designs for commercial purposes
- ❌ **You CANNOT:** Create modified or derivative versions for distribution

This license protects the 3D models, hardware designs, and other creative works from commercial exploitation while allowing the community to learn from and reference the work.

See the [LICENSE](LICENSE) file for full legal details.

---

## Acknowledgments

- Computed tomography principles that inspired the measurement approach
- The open-source 3D printing community
- Online resources for STM32 development and CAN bus implementation

---

## Contact

**Huitak Lee**
- GitHub: [@lhtlove](https://github.com/lhtlove)

---

*This project is part of my maker portfolio submission. For questions about the technical implementation or collaboration opportunities, please open an issue or reach out directly.*
