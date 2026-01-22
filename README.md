# 3DC — 3D Compiler

**CT-Inspired Ultrasonic Scanning System for Additive Manufacturing**

---

## Why This Exists

CAD software enables seamless design iteration: inspect geometry, modify features, undo operations, refine continuously. Physical fabrication lacks this capability entirely. Current 3D printers execute instructions without perceiving what they create or how environmental conditions affect the process.

Existing feedback solutions address specific failure modes—filament jams, bed adhesion issues, layer shifts—by detecting known problem signatures. This project takes a different approach: building comprehensive environmental awareness that captures multiple volumetric data types from a single sensor system. The goal extends beyond error detection toward enabling iterative physical editing—make-check-change workflows instead of make-check-discard-remake cycles.

---

![Project Status](https://img.shields.io/badge/status-in%20development-yellow)
[![License: CC BY-NC-ND 4.0](https://img.shields.io/badge/License-CC%20BY--NC--ND%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-nd/4.0/)

> **📄 [View Comprehensive Project Overview](https://docs.google.com/document/d/1DoINSkOtGRSBR8vOlWFI2wLNMxFoE6zbKSt7RHld4zc/edit?usp=sharing)**
> Detailed documentation covering design decisions, technical challenges, and development process.
>
> **🔧 [View Interactive 3D Assembly](https://a360.co/4szlqWj)**
> Explore the full system design in 3D (Autodesk Viewer)

---

## The Core Concept

### Inverting the Ultrasonic Weakness

Ultrasonic sensors have a well-documented limitation: sound propagation speed depends on temperature. For precision distance measurement, this temperature dependence introduces errors when thermal conditions vary along the measurement path.

This project inverts that limitation into a feature.

Sound travels at different speeds depending on temperature—approximately 0.6 m/s faster per degree Celsius. Rather than treating this as an error source requiring compensation, the system measures it directly. By collecting time-of-flight data between known positions across multiple angles—the same geometric principle underlying medical CT scanning—the system reconstructs what causes those speed variations.

**The reconstruction approach**: Computed tomography traditionally measures how much of a beam attenuates (as in X-ray CT). Ultrasonic CT instead measures how fast the beam travels across a known distance, directly yielding the temperature of the traversed path. With measurements collected across multiple angles on a circular scanning geometry, the data feeds into inverse Radon transformation, producing a 2D map of sound propagation speed at specific locations within that layer—equivalently, a temperature map. Stacking multiple slices produces volumetric thermal distribution.

```
        ┌─────────────────────────────────────────────────────────────┐
        │                    MEASUREMENT PHASE                        │
        └─────────────────────────────────────────────────────────────┘
                                    
        [Sensor A] ─────────── sound wave ─────────── [Sensor B]
                 \                                    /
                  \     measured: time-of-flight     /
                   \    known: distance             /
                    ─────────────────────────────────
                                  │
                                  ▼
                      calculated: sound speed
                      (varies with temperature)
                                  │
        ┌─────────────────────────────────────────────────────────────┐
        │                  TOMOGRAPHIC RECONSTRUCTION                 │
        └─────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                    ┌─────────────────────────┐
                    │  Multi-angle Collection │
                    │  (rotate shell, repeat) │
                    └─────────────────────────┘
                                  │
                                  ▼
                    ┌───────────────────────┐
                    │       Sinogram        │
                    │  (angle vs. position) │
                    └───────────────────────┘
                                  │
                                  ▼
                    ┌─────────────────────────┐
                    │ Inverse Radon Transform │
                    └─────────────────────────┘
                                  │
                                  ▼
        ┌────────────────────────────────────────────────────────┐
        │                      DUAL OUTPUT                       │
        └────────────────────────────────────────────────────────┘
                                  │
                ┌─────────────────┴─────────────────┐
                ▼                                   ▼
    ┌───────────────────────┐           ┌───────────────────────┐
    │  2D Sound Speed Map   │           │  Calibrated Proximity │
    │         ↓             │           │     Measurement       │
    │  2D Temperature Map   │           │                       │
    │         ↓             │◄──────────│  (sound speed along   │
    │  Stack → 3D Thermal   │ provides  │   measurement path    │
    │      Distribution     │ correction│   known from map)     │
    └───────────────────────┘           └───────────────────────┘
```

### Dual-Output Architecture

The critical realization was how this temperature data could calibrate proximity measurements. By comparing the computed propagation speed map against the expected path of any subsequent measurement, the system predicts speed variations along that trajectory. This eliminates errors caused by thermal gradients.

A single sensor type provides two outputs:
1. **3D Temperature Distribution**: Multi-angle ToF measurements reconstructed via inverse Radon transform
2. **Calibrated Proximity Measurement**: Distance calculations using path-specific sound speed from the thermal map

The traditional weakness of ultrasonic sensing becomes environmental awareness. Temperature sensitivity, normally a source of error, becomes a source of information.

### Why Not Optical Methods?

The initial interest in ultrasonic scanning came from its immunity to reflective surfaces and transparent objects—fundamental limitations of optical methods like LiDAR. These failure modes matter for 3D printing feedback where materials and surface conditions vary unpredictably.

Phased arrays were eliminated early. They focus on beam control and object detection rather than pure time-of-flight measurement with the precision required for tomographic reconstruction. Mechanical positioning of high-directivity transceivers provides the measurement geometry this application demands.

---

## System Architecture

Three independent modules handle different functions in the measurement pipeline.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              SYSTEM OVERVIEW                                 │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐     │
│    │  MOTION SHELL   │      │ CARRIAGE MODULE │      │  SENSOR MODULE  │     │
│    │                 │      │                 │      │                 │     │
│    │  Ring framework │ ───► │ Sensor positioning ─►  │  Ultrasonic ToF │     │
│    │  + rotation     │      │ along rail      │      │  measurement    │     │
│    └─────────────────┘      └─────────────────┘      └─────────────────┘     │
│                                                                              │
│                         ▼ CAN Bus @ 1Mbps ▼                                  │
│              (through custom ENIG-plated PCB slip rings)                     │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

<table>
<tr>
<td width="33%" valign="top">

### 📐 Motion Shell

**Ring-shaped mechanical framework**

| | |
|---|---|
| Diameter | 444mm |
| Rotation | 360° continuous |
| Tilt | 0-90° foldable |
| Motors | NEMA 17 + TMC2209 |

**Key innovations:**
- Custom PCB slip rings (ENIG-plated) for power + CAN through infinite rotation
- Foldable design for module access and hemispherical scanning
- Dual-controller master-slave architecture

<br>

> **[📖 Full Documentation →](01-motion-shell/README.md)**
> 
> *Design evolution, slip ring development, gear systems, firmware architecture*

</td>
<td width="33%" valign="top">

### ⚙️ Carriage Module

**Compact 3-axis sensor positioning**

| | |
|---|---|
| Size | 46×30×77mm |
| Linear | NEMA 8 stepper |
| Orientation | ±60° dual-axis |
| Controller | STM32F103 |

**Key innovations:**
- Automotive-inspired rod mechanism for orientation control
- Custom DC motor algorithm (FULL_SPEED→COAST→BRAKE) replacing failed PID
- ±12 count accuracy with hobby-grade components

<br>

> **[📖 Full Documentation →](02-carriage-module/README.md)**
> 
> *DC motor control problem, rod mechanism, hardware iterations*

</td>
<td width="33%" valign="top">

### 📡 Sensor Module

**300kHz ultrasonic transceiver**

| | |
|---|---|
| Frequency | 300kHz DDS |
| Drive | ±22V bipolar |
| Channels | 4 TX / 4 RX |
| MCU | STM32G431 |

**Key innovations:**
- Research-grade frequency for sub-millimeter measurement resolution
- Bessel filter topology preserving waveform shape for phase-based ToF
- Custom TX/RX signal chains

<br>

> **[📖 Full Documentation →](03-sensor/README.md)**
> 
> *Signal chain design, DDS waveform generation, filter selection*

</td>
</tr>
</table>

| ▶️ Complete Demo of Working Parts (Video) |
|:--:|
| [![Overall Tested](https://img.youtube.com/vi/h6HlKs1gbwU/maxresdefault.jpg)](https://youtu.be/h6HlKs1gbwU) |

---

### Design Highlights

Each module documentation covers full design evolution. Here's what makes each interesting:

**Motion Shell** — The fundamental challenge was enabling full rotation while maintaining electrical connectivity. Commercial slip rings cost hundreds of dollars. Custom PCB slip rings with ENIG plating and spring-loaded ball contacts solved both cost and bulk—but finding the right contact pressure required extensive iteration: too light and connections drop out, too heavy and the gold plating wears through.

**Carriage Module** — Standard PID control failed completely for DC motor positioning. Reducing PWM to slow down also reduces torque, causing stalls before reaching target. The solution was a three-phase algorithm that maintains near-full torque while using encoder feedback to time the transition to coasting, then braking. Coasting removes the uncertainty of how driving force interacts with mechanical loads.

**Sensor Module** — The 300kHz operating frequency (research-grade, not consumer sensor territory) provides sub-millimeter resolution potential. Bessel filter topology was chosen specifically because unlike Butterworth or Chebyshev configurations, Bessel filters maintain signal shape—essential for phase-based ToF measurement rather than simple threshold detection.

---

## CAN Bus Network Architecture

All modules communicate via CAN bus at 1Mbps, routed through the custom slip rings during rotation.

**Device Addressing:**

| Axis/Module | Master ID | Slave ID | Reports To |
|-------------|-----------|----------|------------|
| Rotation    | 0x200     | 0x201    | 0x700      |
| Tilting     | 0x100     | 0x101    | 0x700      |
| Carriage    | 0x300+n   | —        | 0x700      |
| Sensor      | 0x400+n   | —        | 0x700      |

**Protocol format:** 8-byte frames with handler-based dispatch
```
Byte 0: [4-bit handler ID][4-bit command]
Bytes 1-7: Command-specific payload
```

Master-slave synchronization enables dual-motor drive for rotation and tilt axes. Masters forward commands to slaves, and both motors move together. Completion notifications propagate back to the system controller.

---

## Current Status

### Completed

- Motion shell mechanical design and full assembly
- Foldable ring with ENIG-plated PCB slip rings validated
- Carriage modules with full 3-axis motion control
- DC motor positioning algorithm achieving ±12 encoder count accuracy
- CAN bus network operating through slip ring contacts during rotation
- Sensor board hardware with individual TX/RX validation
- 300kHz waveform generation and amplified transducer drive
- Waveform-preserving signal reception and ADC capture

### In Progress

**TX/RX Channel Switching:** The analog switch configuration for single-transducer echo mode has implementation errors preventing proper changeover between transmit and receive phases. The fix is identified but not yet implemented.

**Inter-Board Timing Synchronization:** Face-to-face two-sensor ToF measurement requires sub-microsecond timing coordination between separate boards. CAN bus communication introduces latency and jitter that degrades measurement accuracy. At 343 m/s sound speed, each microsecond of timing uncertainty corresponds to ~0.34mm distance error.

Potential solutions under consideration:
- Hardware trigger line for sub-microsecond sync
- Bidirectional measurement with averaging to cancel systematic delays
- Timer synchronization with post-processing correction
- Statistical averaging with outlier rejection

### Planned

1. **Debug TX/RX switching** for echo mode operation
2. **Implement hardware trigger** for two-board synchronization
3. **Validate ToF accuracy** against known distances
4. **Integrate sensor with carriage** for positioned measurements
5. **Implement sinogram data collection** across multiple angles
6. **Apply inverse Radon transform** for tomographic reconstruction
7. **Generate thermal distribution maps** and validate against known temperature distributions
8. **Calibrated proximity measurement** using computed thermal maps
9. **Printer firmware integration** for real-time feedback during printing

---

## The Ultimate Vision

The furthest goal remains unchanged: not just error detection, but enabling iterative editing of physical objects.

Current 3D printing treats fabrication as a one-shot process. If something goes wrong, you stop, discard, and restart. With comprehensive environmental awareness and geometric feedback, a different workflow becomes possible: detect deviations during printing, intervene at the layer level, potentially even modify or correct in-place.

The mechanical infrastructure now exists. What remains is proving the sensing concept works, then building the software layer that lets a printer act on what it perceives.

---

## Background

This project represents two years of independent development, motivated by the need for reliable 3D printing feedback in hardware work. The work spans mechanical design, custom electronics, embedded firmware, and signal processing—all developed without institutional resources, collaborators, or formal mentorship.

The core concept emerged from recognizing that a traditional sensor weakness (temperature sensitivity) could be inverted into a capability through tomographic reconstruction principles borrowed from medical CT imaging. Rather than building yet another failure detector, the goal is comprehensive environmental awareness enabling iterative physical fabrication.

---

## Repository Structure

```
3DC/
├── README.md
├── 01-motion-shell/
│   ├── README.md
│   ├── hardware/
│   │   ├── mechanical/          # CAD files (STEP, STL)
│   │   └── electronics/         # Stepper controller PCB
│   └── firmware/
│       ├── CP_Rotary/           # Rotation axis firmware
│       └── CP_Rotator/          # Tilt axis firmware
├── 02-carriage-module/
│   ├── README.md
│   ├── hardware/
│   │   ├── mechanical/          # CAD files
│   │   └── electronics/         # Module controller PCB
│   └── firmware/
│       └── CC_Module/           # Carriage controller firmware
├── 03-sensor-module/
│   ├── README.md
│   ├── hardware/
│   │   └── electronics/         # Sensor board PCB
│   └── firmware/
│       └── SF_Sensor/           # Sensor firmware
└── 04-docs/
    ├── images/
    └── references/
```

---

## License

**Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International** (CC BY-NC-ND 4.0)

- Share and redistribute with proper attribution
- No commercial use
- No modified or derivative versions for distribution

---

## Contact

**Huitak Lee**  
GitHub: [@lhtlove](https://github.com/lhtlove)
