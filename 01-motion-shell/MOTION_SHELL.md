# Motion Shell

A foldable ring-shaped mechanical framework for the 3DC modular ultrasonic scanner system, enabling CT-inspired multi-angle measurements.

## The Narrative

After deciding the basis of sensor control and positioning, it became clear that this had to be in a modular format. Each measuring unit had to move independently inside a dedicated rail or track, allowing it to position itself anywhere along the rail.

The earlier version of this shell was made to meet these requirements. The overall intention of the design didn't differ much from the modern one. The main shell rotates infinitely on a dedicated circular rail, providing speed and robustness, and each sensor unit moves inside the shell to precisely position and orient sensors. The disparity was that it had separate sectors of inner rails, and the motors that move the shell were located inside it, between the two sectors.

I was aware that a sensor carriage unit's positioning capability would be limited to ±60 degrees, so a fully connected rail wouldn't help much with measurement but would only add complexity for homing each module, as there would be no limit without blockades. Also, it was clear that this kind of sensor positioning only enables measurement on the XY plane. It can't see anything that can be observed from different angles. So I thought that fully rotating the external motion axis would make it possible to measure proximity from tilted angles. So I didn't see much of a problem at that point.

![Earlier Design](images/earlier_design.jpg)
*Earlier version with separate rail sectors and internal motors*

However, as testing happened, the design revealed plenty of limitations. First of all, it was highly inconvenient to place and remove carriage modules. I had to disassemble the whole shell to replace and insert modules. This was a barrier both for testing and practical use. Another problem was that the tilting method was never going to work; there are multiple linear rods and screws utilized by the motion shell itself and possibly the print bed, which is the platform of the main measurement. Rotating the whole shell was unlikely since avoiding collisions with those parts is physically impossible.

So I had to think of an improved design that simultaneously solves the tilting capability and the difficulty of replacements.

What I eventually came up with was folding half of the shell and removing the division of sectors. The motors that power the shell rotation are externalized for this reason. Now the modules can move infinitely inside the shell, and the shell itself can fold from 0 to 90 degrees. When it's at 0°, it operates as intended: modules move freely and orient sensors at required locations. When I need measurement from the top, or angles other than just on the XY plane, some sensors can move to the folding side of the shell, and then it can tilt to the needed angle. If I need to replace, remove, or insert modules, the shell can be tilted to 90° and they can be easily removed from the rigid side. Module homing is still possible: simply put modules on the folding side, tilt to 90°, and move the modules to the end of each side. The rigid side's ceiling makes a proper blockade—a reference for homing. The concerns around collisions on full tilt were fully resolved too. Now it only has to beware of components above the shell (e.g., printer heads).

![Modern Folding Design](images/folding_design.jpg)
*Modern foldable design with continuous rail and external motors*

With this improvement, the critical problems were all resolved and made an excellent breakthrough that conventional CT device structures didn't offer. The attempt to create a framework for modular sensor units with ultrasonic measurement turned out to be successful.

---

## Technical Specifications

All designed parts other than off-the-shelf components like steppers and fasteners are 3D printed with an FDM printer using PLA + carbon fiber mixed material.

### Overall Dimensions

- Outer diameter: ~444mm
- Shell height: Variable based on configuration
- Rail cross-section: Accommodates carriage modules (46×30×77mm)

### Exploded Overview

![Exploded View](images/exploded_overview.jpg)
*Complete system breakdown showing all major components*

### Bearing-Based Traveling Units

![Bearing Units](images/bearing_units.jpg)
*Bearing assemblies enabling smooth rotation*

### Rail System with Slip Rings

![Rail System](images/rail_system.jpg)
*Rail system assembled with slip ring boards*

The rail features:
- Continuous circular track for carriage module travel
- Integrated gear teeth (round rack) for positioning
- ENIG-plated PCB slip rings for power and CAN bus communication during infinite rotation

### Slip Ring Contact System

![Slip Ring Contacts](images/slip_ring_contacts.jpg)
*Slip ring contacts soldered to spring-loaded ball pins*

The carriage modules interface with the shell's slip ring using spring-loaded ball pins, enabling:
- Tool-free installation and removal
- Reliable electrical contact during rotation
- 4-channel communication (power, ground, CAN-H, CAN-L)

---

## Tilting System

![Tilting System Internals](images/tilting_internals.jpg)
*Inside of the tilting system*

### Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Gear Stage 1 | 1:3 reduction spur gears |
| Gear Stage 2 | 1:1 bevel gears |
| Gear Stage 3 | 1:4 reduction planetary gears |
| **Total Reduction** | **1:12** |
| Motor | NEMA 17 (20mm height) |
| Tilt Range | 0° to 90° |
| Motor Rotations for 90° | 3 rotations |

### Heat Dissipation Design

![Heat Dissipation](images/heat_dissipation.jpg)
*Heat dissipation design showing gearbox internals and real assembly*

---

## Rotation System

![Rotation System Overview](images/rotation_system.jpg)
*Rotation system overview*

### Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Total Reduction | 360:31 (teeth ratio) |
| Motor | NEMA 17 |
| Motor Rotations per Shell Revolution | 360/31 ≈ 11.6 |
| Shell Support | Z-axis shaft bushings and screw nuts |
| Homing | Physical switch on PA12 |

![External Motion Axis](images/external_axis.jpg)
*External motion axis gearbox detail*

### Position Indication

![Position Indication](images/position_indication.jpg)
*Frame structure with homing mechanism*

- Rotational indicators click the switch to indicate positions
- Frame structure ensures no click loss during operation

---

## Controller Circuit Board

![Controller Board](images/controller_board.jpg)
*The controller circuit board overview*

The same controller boards are used across both the tilting and rotation systems, providing standardization and simplified maintenance.

### Circuit Specifications

| Component | Value/Part |
|-----------|------------|
| **MCU** | STM32F103C8T6 (LQFP48) |
| **Clock** | 16MHz HSE crystal |
| **Stepper Driver** | TMC2209-LA-T |
| **CAN Transceiver** | SN65HVD230DR |
| **Magnetic Encoder** | MT6701CT-STD |
| **Power Input** | 7-30V DC |
| **Logic Voltage** | 3.3V (AMS1117 regulator) |
| **Motor Voltage** | 6V (TPS54331DR buck converter) |

### Power Supply Section

| Component | Value | Description |
|-----------|-------|-------------|
| U22 | TPS54331DR | Buck converter (7-30V to 6V) |
| L4 | 6.8µH | Power inductor |
| C72, C73 | 47µF each | Output capacitors |
| D2 | SS34 | Schottky diode |
| R19 | 332kΩ | Feedback divider (top) |
| R10 | 68.1kΩ | Feedback divider (bottom) |
| C69 | 10nF | Compensation |
| C70 | 1nF | Compensation |
| C71 | 47pF | Compensation |

### TMC2209 Stepper Driver Configuration

| Parameter | Setting |
|-----------|---------|
| Communication | Single-wire UART |
| Microstepping | 16x (with 256x interpolation) |
| Mode | StealthChop |
| Run Current | 31 (5-bit, max) |
| Hold Current | 8 |
| Stall Current | 16 |
| CoolStep | Enabled (semin=5, semax=6) |
| StallGuard Threshold | 50 |

### CAN Bus Interface

| Parameter | Value |
|-----------|-------|
| Transceiver | SN65HVD230DR |
| Termination | 120Ω (R9) |
| Baud Rate | 1 Mbps |
| Bus Capacitors | 4.7µF (C22, C67) |

### I2C Interface (Encoder)

| Parameter | Value |
|-----------|-------|
| Pull-up Resistors | 4.7kΩ (R11, R12) |
| Encoder | MT6701CT-STD |

### Indicator LEDs

- LED1: TX activity (VCP_TX)
- LED2: Status indicator
- LED3: Status indicator
- All with 10Ω series resistors (R25, R26, R27)

![Controller Board Render](images/controller_board_render.jpg)
*3D render of the controller PCB*

---

## Firmware Architecture

### CAN Bus Network Architecture

The firmware follows a **master-slave CAN bus architecture** with two independent subsystems:

**Rotary System (Shell Rotation)**
- Master: `0x200` — receives commands, coordinates motion
- Slave: `0x201` — mirrors master commands for synchronized dual-motor drive

**Rotator System (Tilting)**
- Master: `0x100` — receives commands, coordinates motion
- Slave: `0x101` — mirrors master commands

Each master forwards relevant commands to its slave via CAN, ensuring both motors in each pair operate synchronously.

### CAN Protocol

**Message Format** (8-byte payload):
```
Byte 0: [4-bit handler ID][4-bit command]
Bytes 1-7: Command-specific data
```

**Handler IDs:**
- `0x1`: Motor control (move, stop, enable, speed, current)
- `0x2`: Rotator/Rotary high-level commands (home, move to angle)
- `0x3`: Debug/prompt

**Motor Commands (0x1X):**

| Cmd | Function | Payload |
|-----|----------|---------|
| 0x10 | Move | int32 steps + direction byte |
| 0x11 | Stop | — |
| 0x12 | Enable | uint8 toggle |
| 0x13 | Reset | — |
| 0x14 | Set Speed | int16 period |
| 0x15 | Set Current | run, stall, hold bytes |
| 0x1E | Query Running | — |
| 0x1F | Query Position | — |

### Motion Control Implementation

**Timer-Based Step Generation**
- TIM4 generates step pulses via interrupt
- Prescaler: 16-1 (1µs tick at 16MHz)
- Period: Dynamically adjusted for speed control

**Acceleration Profile**
- Linear acceleration/deceleration
- `accel_rate`: Controls acceleration steepness
- `accel_th`: Threshold period for deceleration calculation
- Deceleration triggered when remaining steps equals calculated braking distance

**Position Tracking**
- `location_steps`: Signed 32-bit absolute position counter
- `current_steps`: Steps executed in current move
- `steps_to_go`: Target steps for current move

### Rotator (Tilting) Motion Logic

**Step Calculation:**
```
target_steps = (USTEP_RATE × 600 × position_degrees) / 90
```
At 16x microstepping: 9600 microsteps for 90°

**Homing Sequence:**
1. Move positive until mechanical limit (sensorless stall detection via StallGuard)
2. Move negative until opposite limit
3. Calculate midpoint as zero reference

**Stall Detection:**
- Uses TMC2209's StallGuard feature
- DIAG pin triggers EXTI interrupt on stall
- Reduced current during homing for reliable detection

### Rotary (Rotation) Motion Logic

**Step Calculation:**
```
target_steps = (360/31) × USTEP_RATE × 200 × position_degrees / 360
```

**Homing Sequence:**
1. Move negative to find limit switch (PA12 input)
2. Move positive to find opposite reference
3. Calculate center position accounting for 360:31 ratio

### TMC Driver Register Configuration

**GCONF (0x00):**
- `pdn_disable = 1`: UART mode enabled
- `mstep_reg_select = 1`: Microstepping via register
- `multistep_filt = 1`: Pulse filtering enabled

**CHOPCONF (0x6C):**
- `toff = 3`: Chopper off-time
- `hstart = 4`, `hend = 0`: Hysteresis settings
- `mres = 0b0100`: 16x microstepping
- `interpolation = 1`: 256x interpolation enabled

**COOLCONF (0x42):**
- CoolStep current regulation enabled
- `semin = 5`, `semax = 6`: Current scaling thresholds

---

## Assembly Photos

### Complete Assembly

![Complete Assembly](images/complete_assembly.jpg)
*Top-down view of the assembled motion shell with calibration grid*

### External Wiring

![External Wiring](images/external_wiring.jpg)
*External wiring overview*

---

## Key Design Features

1. **Folding Mechanism**: Solves module replacement, collision avoidance, and tilted measurement capability with a single design
2. **Custom Slip Rings**: ENIG-plated PCBs with spring-loaded ball pin contacts—cost-effective compared to commercial solutions
3. **High Gear Reductions**: 1:12 for tilt, 360:31 for rotation—sufficient torque for reliable operation
4. **Standardized Controllers**: Same board design across all axes for simplified development and maintenance
5. **Dual-Motor Synchronization**: CAN bus master-slave architecture ensures precise coordinated motion

---

## Related Documentation

- [Carriage Module](carriage_module.md)
- [Sensor Module](sensor_module.md)
- [Main Project Overview](../README.md)
