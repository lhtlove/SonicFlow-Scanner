# 3DC — 3D Compiler

**CT-Inspired Ultrasonic Scanning System for Additive Manufacturing**

---

## Why This Exists

CAD software enables seamless design iteration: inspect geometry, modify features, undo operations, refine continuously. Physical fabrication lacks this capability entirely. Current 3D printers execute instructions without perceiving what they create or how environmental conditions affect the process.

Existing feedback solutions address specific failure modes—filament jams, bed adhesion issues, layer shifts—by detecting known problem signatures. This project takes a different approach: building comprehensive environmental awareness that captures multiple volumetric data types from a single sensor system. The goal extends beyond error detection toward enabling iterative physical editing—make-check-change workflows instead of make-check-discard-remake cycles.

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
                    ┌───────────────────────┐
                    │  Multi-angle Collection │
                    │  (rotate shell, repeat) │
                    └───────────────────────┘
                                  │
                                  ▼
                    ┌───────────────────────┐
                    │       Sinogram        │
                    │  (angle vs. position) │
                    └───────────────────────┘
                                  │
                                  ▼
                    ┌───────────────────────┐
                    │ Inverse Radon Transform│
                    └───────────────────────┘
                                  │
                                  ▼
        ┌─────────────────────────────────────────────────────────────┐
        │                      DUAL OUTPUT                            │
        └─────────────────────────────────────────────────────────────┘
                                  │
                ┌─────────────────┴─────────────────┐
                ▼                                   ▼
    ┌───────────────────────┐           ┌───────────────────────┐
    │  2D Sound Speed Map   │           │  Calibrated Proximity │
    │         ↓             │           │     Measurement       │
    │  2D Temperature Map   │           │                       │
    │         ↓             │◄──────────│  (sound speed along   │
    │  Stack → 3D Thermal   │  provides │   measurement path    │
    │      Distribution     │  correction   known from map)    │
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

Three independent modules handle different functions in the measurement pipeline. Each module has detailed documentation covering design evolution, technical specifications, and firmware architecture.

### Motion Shell

The outer ring provides the mechanical framework for scanning operations.

The fundamental challenge was enabling full rotation while maintaining electrical connectivity and allowing sensor access. Commercial slip rings cost hundreds of dollars and add bulk. The solution: custom ENIG-plated PCB rings with spring-loaded ball pin contacts, carrying power and CAN bus signals through continuous rotation.

The design evolved through multiple iterations. The earlier version had separate rail sectors with internal motors between them. Testing revealed critical problems: module replacement required full disassembly, and the planned tilting method (rotating the entire shell) would collide with printer components.

The current foldable design resolves both issues. Half the shell folds 0-90°, enabling module access when tilted, hemispherical scanning coverage when partially tilted, and standard circular scanning when flat. Modules move freely through fully connected rails with no sector divisions.

**Key specifications:**
- 444mm outer diameter
- Continuous 360° rotation via 360:31 gear reduction
- 0-90° tilt via 1:12 planetary gear reduction (1:3 spur × 1:1 bevel × 1:4 planetary)
- NEMA 17 stepper motors with TMC2209 drivers
- Dual-controller master-slave architecture per axis
- Physical homing switch (rotation) and StallGuard sensorless homing (tilt)

→ [Motion Shell Documentation](01-motion-shell/README.md)

### Carriage Module

Compact sensor positioning units that travel along the shell's internal rail system.

CT reconstruction requires specific beam geometries—fan beam, parallel beam, etc. The 300kHz transceivers have approximately 10° directivity with steep attenuation beyond 1-2° misalignment. This demands accurate sensor orientation, not just positioning. Simply rotating the sensor module was the clear solution; the challenge was achieving rotation within extreme space constraints.

Direct motor placement failed. Placing a motor beside the main stepper made the module too wide for full rotation inside the shell. Stacking motors made the shell too tall. The solution borrowed from automotive steering: two rods controlling orientation through differential length, with the angle determined by arctan(rod_extension / 8mm_spacing).

The DC motors (N20 with 150:1 gearboxes and 12-CPR encoders) presented unexpected control problems. Standard PID failed because reducing PWM to slow down also reduces torque, causing stalls before reaching target position. The gears wouldn't move until sufficient torque built up, but by then it was too late to stop precisely.

The solution: a three-phase algorithm (FULL_SPEED → COAST → BRAKE) that maintains near-full output while using encoder feedback to time transitions. Coasting removes the uncertainty of how driving force interacts with mechanical loads, making braking predictable. Through testing, minimum movement is 12 encoder counts, and positioning error stays within ±12 counts—less than 1° orientation error, smaller than mechanical tolerances in the 3D-printed structure.

**Key specifications:**
- 46×30×77mm envelope
- NEMA 8 stepper for rail positioning (60:1 effective gear ratio, 32× microstepping)
- Dual N20 DC motors for orientation control (±60° range)
- Custom STM32F103-based controller (44×23mm)
- Tool-free installation via slip ring contacts
- CAN bus communication at 1Mbps

→ [Carriage Module Documentation](02-carriage-module/README.md)

### Sensor Module

High-frequency ultrasonic transceiver designed for phase-based ToF measurement.

Finding sensors meeting the requirements—individual transceiver units with high directivity, 300kHz frequency, and detection range covering the scanning zone—proved difficult. Nothing commercially available fully satisfied the conditions. Designing the driving and reception circuitry from scratch was necessary, but also advantageous: the circuit could target specific requirements rather than adapting around existing module limitations.

The 300kHz operating frequency (research-grade range, not consumer sensor territory) provides ~1.1mm wavelength in air, supporting sub-millimeter resolution with phase detection. The transmit chain uses AD9837 DDS for precise waveform generation, TLE2142 for amplification, and a BJT push-pull buffer delivering ±22V to drive transducers. The receive chain routes through DG409 multiplexer to two-stage TLE2142 amplification with Bessel filtering, then OPA863 final stage to STM32G431's ADC.

Bessel filter topology was chosen specifically for waveform preservation. Unlike Butterworth or Chebyshev configurations, Bessel filters maintain signal shape—essential for phase-based ToF measurement rather than simple threshold detection.

**Key specifications:**
- 28×52mm board dimensions
- 300kHz DDS-generated waveform (AD9837, 0.1Hz resolution)
- ±22V bipolar drive (MT3608 boost + TC7660 charge pump)
- 4-channel TX/RX switching (DG412 TX, DG409 RX)
- Bessel-filtered reception for waveform preservation
- STM32G431 with 12-bit ADC up to 4 MSPS
- CAN bus communication

→ [Sensor Module Documentation](03-sensor-module/README.md)

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
