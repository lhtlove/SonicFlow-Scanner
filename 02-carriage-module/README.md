## The Narrative

After developing the CT-inspired measurement ideas and dual measurement type basis, it became clear that utilizing high-frequency, high-directivity ultrasonic transceivers with mechanical control rather than acoustic mechanisms such as phased arrays would achieve maximum precision.

Radial and diametric measurement is easily possible when a sensor is located vertically to the tangent line on the circle. If this were all that was needed, no hard thinking would be required. The CT basis always requires certain shapes of measurement beam that can cover a wide range inside the circle—fan beam, parallel beam, etc. For these flat beamforming configurations, chordal measurement is what I needed for this implementation.

The transceiver is really designed for something close to "point-to-point" measurement. Its datasheet indicates a total 10 degrees of directivity, but attenuation happens steeply even when misaligned by 1-2 degrees. So with little room for error, the sensor had to accurately achieve the required orientation. To accomplish chordal measurements, simply rotating the sensor module itself was the clear solution.

### The Design Challenge

The issue was designing that in reality. The concept is easy: a small mechatronic module that has one motor for positioning and another actuator to achieve rotation.

A small stepper motor (NEMA 8) was sufficient for module positioning. However, it was nearly impossible to achieve the rotating function within the required dimensions while also maintaining stability.

Placing any sort of motor next to the main stepper would make the module too wide to fully rotate inside the shell of its current size. Stacking two motors wasn't viable either without making the shell excessively tall. The current dimension of the motion shell isn't small at all, so the direct method of rotation wasn't an option.

The next thing I could think of was using two motors to control two different rods that rotate the object connected with rods by creating a difference in lengths. It's the same idea behind how modern cars' steering mechanisms work.

This method, however, still presented difficult problems. Now I needed two additional motors that are small enough to fit the space, powerful enough to move the rods, and precise enough to position them wherever required.

Stepper motors always seem to be the ideal choice, but I had to be aware that I was already using one of the smallest steppers available to drive the module positioning, and I was looking for something even smaller. Stepper motors and most other precision motors of such size with the required performance were impossible to find. Those that came relatively close were still too expensive to be affordable. And "relatively" is key here—nothing truly met my needs.

Something that met most of these requirements was geared DC motors combined with encoders. I was largely concerned about their lack of precision and controllability but still decided to give it a try.

| Early Prototyping |
|:--:|:--:|
| ![Early prototype](../04-docs/images/m-1.jpeg) | ![Early prototype](../04-docs/images/m-2.jpeg) |

### Component Arrangement

The overall arrangement of core components has been consistent. Around a stepper that moves the module, two N20-size DC motors are placed, which are virtually the only smallest practical DC motors available.

A customized controller board containing essential elements is placed on the stepper. At the top and bottom, geared systems to control the rods are positioned. A gear to convey the stepper rotation is also at the bottom. At the top, spring-loaded ball pins are located to make contact with the slip ring of the shell.

This design, without regard to performance, was the best and most suitable to fit the motion shell without making it excessively large. The thickness between the outer and inner diameter and the height of the shell could be kept to a minimum.

### Early Hardware Issues

The fundamental design of this system was sound. However, most of the problems surfaced as the process to make it actually work began.

Making it move was fine—it was as simple as running a stepper motor and making the pinion engage properly with the round rack. But embodying the steering system that orients sensors to requested angles was far harder than expected.

The hardware was largely clunky in early versions. The idea of using 3D-printed small gears to convey the driving force to the top and bottom rod-rails was acceptable. But the gears had to be kept really small to not interfere with other parts of the module and stay mostly inside its dimensions.

So a sort of forced design was applied: put the smallest bearings between each gear. Also, a bearing-based rail was applied for rod-rails, and all gears were herringbone, which I thought was best practice for 3D-printed gears. All of these design choices soon turned out to be mistakes.

The gears weren't smooth enough, possibly worse than without having bearings. The rod-rails consisting of only a few bearings weren't sufficient to contain the rods in their working range. The herringbone choice wasn't helpful for installing and removing rods. So the core feature was unlikely to be realized in this situation.

### The DC Motor Control Problem

And finally, the performance issue I was concerned about soon surfaced after some testing.

Each motor was equipped with a 150:1 reduction gearbox, and I placed a 12-click-per-rotation magnetic encoder on it. So it takes 1800 clicks to fully rotate the final shaft by 2π.

I initially thought control wouldn't be that difficult as long as I could control a motor's speed and update the encoder counts consistently, and it was already widely known that DC motors can be used for precision work when properly controlled with feedback.

But the story was different here: this design required both RPM and torque at the same time, while these two factors are positively correlated in DC motors.

Using PWM-based speed control essentially means handling the torque simultaneously, but unwillingly. For usual applications, there is room for torque capacity, so PWM-based control works fine.

Most H-bridge DC motor drivers come with active braking features, which provide better braking than leaving a motor coasting. But still, this doesn't magically stop the motor at the wanted moment when the motor already has certain inertia.

PWM-based control surmounts these limitations obviously by reducing speeds before braking. Methods like PID are only for better motion and more logically applying PWM-based control.

For my case, there was simply not much room for torque capacity. At speeds low enough to brake exactly at the wanted moment, the torque wasn't enough to drive the gears. Also, the 3D-printed parts and hand-assembled parts were never consistent, so every assembly slightly differed in characteristics.

It was virtually forced to operate above a certain PWM duty. I tried methods like controlling speeds within the possible PWM range, reaching the possible minimum speed before braking, but none of them turned out to be accurate enough or successful.

Even at that minimum speed, there were always plenty of additional counts after fully stopping. Once the gears start rotating, when the torque becomes strong enough to move the parts, it's highly uncertain where the counts would end up when stopped. The nature of DC drivers, DC geared motors, and high loads was just this way.

With these limitations, the initial attempt to make the steering system was unsuccessful. But a few points to address became apparent: fix hardware faults and come up with a method that can operate the DC motors with minimum error.

| Modern Design - Exploded View |
|:--:|
| ![Modern exploded view](../04-docs/images/c-3.png) |

### Hardware Improvements

For the hardware, all the gears used for rod-rail parts were replaced with basic spur gears for easier replacement of gears and rods.

Also, a single bearing was put into each gear while printing. This way, the inner rings of bearings are properly pinned, and there is far less tolerance.

The rod-rails were integrated into the printed parts. With proper lubrication, they work smoothly enough, with little discrepancy in stability across the extent they move.

### Developing the DC Motor Control Algorithm

I tested the behavior of braking while coasting versus driving. When the motor is coasting, active braking becomes more predictable based on speed. It doesn't get affected by how the driving force interacts with mechanical loads and frictions, thereby removing some uncertainty.

So I implemented a mechanism to make it coast when certain conditions are met. The basic operation starts coasting when the motor reaches maximum speed at max power and a specific amount of encoder counts are remaining.

If the counts to travel were too small to reach maximum speed during operation, it had to start coasting when reaching a certain portion of the total counts depending on the actual number.

This method revealed that the motor could be stopped with far less error when braking. The only thing to handle was what to do when the motor stops too early due to heavier loads.

The solution was simple: keep the speed within a certain range that indicated it's enough to maintain the torque and slow enough not to slip past the target counts.

After some testing, things became clear. It's virtually impossible to make the motor move the gears before it builds up sufficient torque to move it even a little bit. But it's already too late then to stop it without any slips.

However, the method of keeping the speed within the ideal range made it possible to minimize the slips and the resulting error. After finding the ideal constants for this mechanism, it became good enough to apply to any setup I made.

Though there might be some room for improvement as I add features to adjust those constants and make it more logical than the current stage-based control, I still reached a required precision hard to attain with these components and minimum error that is tolerable for the current application.

Through tests, the minimum counts to travel turned out to be 12, and overall error when reaching targets turned out to be mostly within ±12 counts. So at worst, the total error caused to the sensor orientation is lower than 1 degree, which is smaller than mechanical limitations of the 3D-printed structure and good enough not to exceed the sensor's ideal directivity.

| Orientation Control Sequence - Top View |
|:--:|
| ![Orientation control top](../04-docs/images/c-4.png) |

| Orientation Control Sequence - Bottom View |
|:--:|
| ![Orientation control bottom](../04-docs/images/c-5.png) |

### The Rod Mechanism

Finally, the rods had to be designed. (The design of these was complete before hardware improvements, but it became functional after that.)

The idea of controlling two rods' lengths was based on a simple triangle scheme. In usual operations, one of two rods stays still while the other moves.

One side (adjacent) of the triangle is constant: the distance between the two rods' rotational points, which is exactly 8mm. As a rod extends, its length determines the other side (opposite), and the length of the hypotenuse and the angle of theta change accordingly. arctan(l/8) decides the theta here.

The overall scheme is simple as it looks. The sensor orientation is perpendicular to the hypotenuse, and it looks similar to car steering systems.

The main problem was that the length of the hypotenuse changes. Unlike that of a car, the rods only perform linear length adjustments. Without a structure to adapt to the hypotenuse length variations, it was not functional.

So I applied the rail system. But the core problem remained: what if the sensor housing moves along the rail when the orientation is adjusted to something other than 0?

It stays still at 0 since there are blockades at both sides. But when it orients to different directions, one side of the rail becomes loose, so it can shift side to side.

Also, without pinning the housing to stay as far as possible from the module body, the housing always interfered with the body during orientation controls.

So at all costs, I had to make sure that the moving rod pinned the sensor housing and dragged it as far as possible from the module body during operations.

This hardly came to a practical design, but after testing some ideas, the pinning method became clear. A round structure was added to each rod, which only slides into the dedicated slots of the sensor housing when its side is on the move. So it naturally drags that side.

This simultaneously solved the shifting issue and the interference with the main body. With small extensions on both sides after homing, it worked perfectly without touching anything.

| Completed Module |
|:--:|
| ![Completed module](../04-docs/images/c-6.png) |

Through this demanding process, the module became complete with the functions I imagined in initial designs.

Though the components seem cheap and hobby-level, I reached the intended capability with minimum tolerance. While this was only one of multiple important components for this project, it was an important milestone that enabled the possibility of this special mechanical framework—one that could possibly be equipped with something other than the sensor module if used for different purposes.

But at least, it became suitable to perform the measurements I wanted.

---

## Technical Specifications

| Exploded Overview |
|:--:|
| ![Exploded overview](../04-docs/images/c-7.png) |

| Rod Gears Overview - Top | Rod Gears Overview - Internal |
|:--:|:--:|
| ![Rod gears top](../04-docs/images/c-8.png) | ![Rod gears internal](../04-docs/images/c-9.png) |

| Heat Dissipation Design |
|:--:|
| ![Heat dissipation](../04-docs/images/c-10.png) |

### Rod Gear System

| Rod Gears Full Structure - Side View | Rod Gears Full Structure - Front View |
|:--:|:--:|
| ![Rod gears side](../04-docs/images/c-11.png) | ![Rod gears front](../04-docs/images/c-12.png) |

| Extension Rail of the Housing |
|:--:|
| ![Extension rail](../04-docs/images/c-13.png) |

### Orientation Control

| Top & Bottom View of Orientation Control Scheme |
|:--:|
| ![Orientation scheme](../04-docs/images/c-14.png) |

- Depending on the target direction (θ > 0 vs θ < 0), either side of the rod moves
- The round pinning structure of the moving rod holds the housing as it moves
- This design ensures the housing doesn't shift side to side and turns as far as possible from the module body, taking up the least room without interfering with the module body
- The maximum range of function is approximately -68.28 to 68.28 degrees, but -60 to 60 degrees is usually ideal for most stability

| Rod Rails | Module Fitting the Shell |
|:--:|:--:|
| ![Rod rails](../04-docs/images/c-15.png) | ![Module in shell](../04-docs/images/c-16.png) |

---

## Circuit Specifications

| Module Controller Circuit Board - Cross Section |
|:--:|
| ![Board cross section](../04-docs/images/c-17.png) |

| Controller PCB - Front | Controller PCB - Back |
|:--:|:--:|
| ![PCB front](../04-docs/images/c-18.png) | ![PCB back](../04-docs/images/c-19.png) |

| Controller PCB - Labeled |
|:--:|
| ![PCB labeled](../04-docs/images/c-20.png) |

### MCU

**STM32F103C8T6** (LQFP48)
- Clock: 16MHz HSE crystal with PLL ×4 → 64MHz SYSCLK
- MCO output on PA8 provides reference clock
- Peripherals: CAN1, I2C1, TIM1/2/3/4, USART3
- DMA channels for I2C transfers

### Stepper Driver

**TMC2208-LA-T** configured via single-wire UART (USART3 at 115200 baud):

| Register | Address | Value | Purpose |
|----------|---------|-------|---------|
| GCONF | 0x00 | i_scale_analog=1, internal_rsense=1, pdn_disable=1, mstep_reg_select=1, multistep_filt=1 | Enable UART, register microstep |
| CHOPCONF | 0x6C | toff=3, hstart=4, hend=0, tbl=1, mres=0b0011, interpolation=1 | 32× microstepping with 256× interpolation |
| IHOLD_IRUN | 0x10 | ihold=8, irun=16 | Hold 8/32, Run 16/32 of max current |

### Position Feedback

**MT6701QT-STD** 14-bit magnetic rotary encoder
- I2C interface at 100kHz (hi2c1)
- Address: 0x06 (shifted to 0x0C for HAL)
- Registers: 0x03 (angle high), 0x04 (angle low)
- Mounted directly under NEMA 8 stepper shaft

**DC Motor Encoders**: 12-click-per-rotation magnetic encoders
- Software quadrature decoding via GPIO polling
- E1_A/E1_B: PA11/PA12 (left motor)
- E2_A/E2_B: PB4/PB5 (right motor)
- 150:1 gearbox × 12 clicks = 1800 counts per output shaft revolution

### DC Motor Drivers

**Dual DRV8837DSGR** H-bridge drivers
- Motor1: IN1=PA3, IN2=PA4
- Motor2: IN1=PA5, IN2=PA6
- PWM via timer-based GPIO toggling (TIM3, TIM4)
- Active braking: both outputs HIGH
- Coast mode: both outputs LOW

### Power Architecture

- **Input**: 7-30V DC via slip ring contacts (12V nominal)
- **Main regulation**: TPS54331DR buck converter → 6V intermediate
- **Logic supply**: AMS1117-3.3 LDO → 3.3V
- **Motor supply**: 6V rail to TMC2208 VM and DRV8837 VM

### CAN Bus

**SN65HVD230DR** transceiver:
- TX: PA11 (CAN_TX), RX: PA12 (CAN_RX)
- Slope control: RS pin tied to CAN_SPEED (PA7)
- 120Ω termination resistor on bus
- FIFO0 with hardware filtering

CAN timing (1Mbps at 64MHz with prescaler 1):
```c
hcan.Init.Prescaler = 1;
hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
```

### GPIO Pin Mapping

| Pin | Function | Mode |
|-----|----------|------|
| PA3 | Motor1_1 | Output |
| PA4 | Motor1_2 | Output |
| PA5 | Motor2_1 | Output |
| PA6 | Motor2_2 | Output |
| PA7 | CAN_SPEED | Output |
| PA8 | MCO (clock out) | Alternate function |
| PA11 | E1_A (encoder) | Input |
| PA12 | E1_B (encoder) | Input |
| PB0 | STEP | Output |
| PB1 | DIR | Output |
| PB4 | E2_B (encoder) | Input |
| PB5 | E2_A (encoder) | Input |
| PB10 | USART3_TX (TMC) | Alternate function |
| PB11 | USART3_RX (TMC) | Alternate function |

---

## Firmware Architecture

The carriage module firmware (**SMC**) controls stepper positioning and DC motor orientation.

### Project Structure

```
SMC/
├── Core/
│   └── Src/
│       └── main.cpp          # HAL init, encoder polling, timer callbacks
└── peripheral/
    ├── Inc/
    │   ├── can.hpp           # CAN protocol definitions, handler types
    │   ├── motors.hpp        # Stepper and DC motor control interfaces
    │   ├── encoders.hpp      # Magnetic encoder reading
    │   ├── smc.hpp           # High-level module control API
    │   └── stepper_setting.hpp  # TMC2208 register maps and UART protocol
    └── Src/
        ├── can.cpp           # CAN TX/RX, command handlers, data serialization
        ├── motors.cpp        # Timer-based step generation, DC motor state machine
        ├── encoders.cpp      # I2C encoder angle calculation
        ├── smc.cpp           # Homing sequences, orientation control
        └── stepper_setting.cpp  # TMC2208 configuration and communication
```

### CAN Bus Network

Module operates as a slave device on the CAN network:

| Device | ID | Report to |
|--------|-----|-----------|
| Carriage Module | 0x304 | 0x700 (Master) |

Filter configuration uses exact-match mode (mask 0x7FF) to receive only addressed messages.

### CAN Protocol

8-byte frame format with handler-based dispatch:
```
Byte 0: [4-bit handler ID][4-bit command]
Bytes 1-7: Command-specific payload
```

Handlers are registered at initialization:
```c
handlers[0].id = 0x1; handlers[0].handler = stp;   // Stepper control
handlers[1].id = 0x2; handlers[1].handler = dc;    // DC motor control
handlers[2].id = 0x3; handlers[2].handler = smc;   // High-level module control
```

**Handler 0x1 - Stepper Control:**

| Cmd | Name | Payload | Description |
|-----|------|---------|-------------|
| 0x11 | Reset | — | Reset location to 0 |
| 0x12 | Move | uint8[1] dir, int32[2-5] steps | Move specified steps |
| 0x13 | Stop | — | Immediate stop |
| 0x14 | SetSpeed | int32[1-4] period | Timer period |
| 0x16 | SetCurrent | uint8[1] run, [2] hold | TMC current settings |
| 0x17 | SetMicrostep | uint8[1] mres | Microstepping resolution |

**Handler 0x2 - DC Motor Control:**

| Cmd | Name | Payload | Description |
|-----|------|---------|-------------|
| 0x21 | Move | uint8[1] id, [2] dir, int16[3-4] counts | Move motor to position |
| 0x22 | Reset | uint8[1] id | Reset encoder and home motor |

**Handler 0x3 - High-Level Module Control:**

| Cmd | Name | Payload | Description |
|-----|------|---------|-------------|
| 0x31 | Setup | uint8[1] id | Initialize module with position preset |
| 0x32 | ModuleMove | float[1-4] degrees | Move module to angle |
| 0x33 | DCHome | — | Home both DC motors |
| 0x34 | DCOrient | float[1-4] radians | Set sensor orientation |

### Software Quadrature Decoding

DC motor encoders are decoded via main loop polling using state transition table:

```c
E1A = HAL_GPIO_ReadPin(E1_A_GPIO_Port, E1_A_Pin);
E1B = HAL_GPIO_ReadPin(E1_B_GPIO_Port, E1_B_Pin);
curE1 = (E1A << 1) | E1B;

if(curE1 != lastE1) {
    switch((lastE1 << 2) | curE1) {
      case 0b0010: case 0b1011: case 0b1101: case 0b0100:
        DCL.countE--;  // Reverse direction
        break;
      case 0b0001: case 0b0111: case 0b1110: case 0b1000:
        DCL.countE++;  // Forward direction
        break;
    }
    lastE1 = curE1;
}
```

Speed measurement uses TIM2 as a free-running counter:
```c
uint32_t speedTim(uint8_t id) {
    return ((id ? speedRE : speedLE) > __HAL_TIM_GET_COUNTER(&htim2) ?
            __HAL_TIM_GET_COUNTER(&htim2) + 65536 :
            __HAL_TIM_GET_COUNTER(&htim2)) - (id ? speedRE : speedLE);
}
```

### DC Motor Control Algorithm

The DC motor control algorithm addresses the fundamental challenge of achieving precision positioning with motors that have coupled torque-speed characteristics. Standard PID control fails because reducing PWM to slow down also reduces torque, causing stalls before reaching target positions.

#### The Core Problem

DC motors exhibit a torque-speed relationship where:
- High PWM duty → High torque but high speed (overshoots target)
- Low PWM duty → Low speed but insufficient torque (stalls before target)

With 3D-printed gears and inconsistent hand-assembled parts, the minimum PWM duty required to overcome static friction varied between builds. This created a narrow operating window where the motor could both move and stop accurately.

#### Three-Phase State Machine

The solution uses a three-phase approach that separates the motion into distinct controllable stages:

```c
typedef enum {
    FULL_SPEED,    // 100% power to build momentum
    COAST,         // Let physics decelerate naturally
    BRAKE,         // Active braking at target
} motor_state_t;
```

**Phase 1: FULL_SPEED**

The motor runs at maximum PWM duty (100%) to ensure sufficient torque to overcome gear friction. During this phase, encoder speed is sampled at 25% of total travel distance to measure actual system response.

```c
case FULL_SPEED:
    HAL_GPIO_WritePin(DC->dir ? DC->OUT_PORT : DC->IN_PORT,
        DC->dir ? DC->OUT_PIN : DC->IN_PIN, GPIO_PIN_SET);

    // Sample speed at 25% of travel for coast point calculation
    if (DC->counts_going >= 50 && DC->countE >= DC->counts_going / 4 && !DC->sum) {
        DC->sum = DC->speedE;
    }

    // Calculate coast point based on measured speed
    if ((float)(DC->counts_going < 1000 ? DC->counts_going : 1000)
            * (0.0002f * (float)DC->sum) * ((float)DC->sum / 1500.0f) >= (float)DC->error) {
        DC->ms = COAST;
        DC->brakepoint = (uint16_t)(0.12f * (float)DC->sum);
    }
    break;
```

The coast point calculation uses the measured speed to predict stopping distance. The formula accounts for:
- Remaining encoder counts (`DC->error`)
- Measured speed at 25% travel (`DC->sum`)
- Empirically determined constants (0.0002f, 1500.0f)

**Phase 2: COAST**

This is the critical innovation. By cutting power and allowing the motor to coast, braking behavior becomes predictable—it's determined purely by mechanical friction and inertia, not by the unpredictable interaction between drive torque and load.

```c
case COAST:
    if (DC->error <= 15) DC->brakepoint = 700;
    
    // Speed too slow - risk of stalling before target
    if (DC->speedE <= DC->brakepoint && DC->error >= 0) {
        // Brief regenerative braking pulse
        HAL_GPIO_WritePin(DC->dir ? DC->IN_PORT : DC->OUT_PORT,
                DC->dir ? DC->IN_PIN : DC->OUT_PIN, GPIO_PIN_SET);
    }
    // Speed too high - risk of overshooting
    else if (DC->speedE > 1100) {
        // Reapply power briefly
        HAL_GPIO_WritePin(DC->dir ? DC->OUT_PORT : DC->IN_PORT,
                DC->dir ? DC->OUT_PIN : DC->IN_PIN, GPIO_PIN_SET);
    }
    // Speed in acceptable range - coast freely
    else {
        HAL_GPIO_WritePin(DC->OUT_PORT, DC->OUT_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DC->IN_PORT, DC->IN_PIN, GPIO_PIN_RESET);
    }

    // Transition to brake when close enough or stalled
    if (DC->error <= 8 || speedTim(DC->id) > 5000) {
        DC->ms = BRAKE;
    }
    break;
```

The coast phase maintains speed within a controllable window:
- If speed drops below `brakepoint`, a brief reverse pulse adds energy
- If speed exceeds 1100 counts/sample, power is reapplied
- Otherwise, the motor coasts with both outputs LOW

**Phase 3: BRAKE**

Active braking is applied by setting both H-bridge outputs HIGH, short-circuiting the motor windings for maximum electromagnetic braking.

```c
case BRAKE:
    DC_Stop(DC->id);  // Both outputs HIGH
    break;
```

#### Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Speed sample point | 25% of travel | Measure actual system response |
| Coast calculation factor | 0.0002 × speed × (speed/1500) | Predict stopping distance |
| Brakepoint | 12% of measured speed | Minimum coast velocity |
| Upper speed limit | 1100 counts/sample | Maximum coast velocity |
| Minimum travel | 12 counts | Below this, direct positioning fails |
| Target proximity | 8 counts | Transition to brake |
| Stall timeout | 5000 timer ticks | Detect mechanical blockage |

#### Results

This algorithm achieves:
- Positioning accuracy: ±12 encoder counts typical
- Orientation error: <1° at sensor (within 10° directivity spec)
- Minimum reliable movement: 12 counts
- Consistent performance across different assembly tolerances

### Homing Sequence

Three-phase DC motor homing via `dc_home()`:

```c
void dc_home() {
    if (phase == HOMING_DONE) {
        phase = STEP1;
        DC_Reset(0);  // Reset left motor (finds limit via timeout)
        DC_Reset(1);  // Reset right motor
    } else if (phase == STEP1) {
        phase = STEP2;
        DC_Move(0, 200, 1);  // Extend left rod
    } else if (phase == STEP2) {
        phase = STEP3;
        DC_Move(1, 200, 1);  // Extend right rod
    } else {
        phase = HOMING_DONE;
        // Notify master
        packet[0] = 0x31;
        CAN_Tx(packet, 8, DEVICE_MASTER);
    }
}
```

`DC_Reset()` drives motor at low duty (35%) until timeout (no encoder pulses for 3500 timer ticks), indicating mechanical limit reached.

### Orientation Angle Calculation

Sensor orientation controlled via rod length differential:

```c
void dc_orient(float ori) {
    if (ori > M_PI / 3) ori = M_PI / 3;    // Clamp to ±60°
    if (ori < -M_PI / 3) ori = -M_PI / 3;
    
    // Convert angle to encoder counts: arctan(l/8) relationship
    // l = 8 * tan(θ), scaled to encoder counts
    int16_t target = abs((int16_t)roundf(tan(ori) * 8 / 20.082 * 1800));
    
    uint8_t id = ori < 0 ? 1 : 0;  // Select motor based on direction
    
    if (side == (id ? -1 : 1) || !side) {
        // Same side or neutral - direct move
        DC_Move(id, abs(target - (id ? DCR.countE : DCL.countE)),
                (target - (id ? DCR.countE : DCL.countE)) > 0 ? 1 : 0);
    } else if (!ori || swap == SWAP) {
        // Return to center first
        DC_Move(id, abs(-(id ? DCR.countE : DCL.countE)),
                -(id ? DCR.countE : DCL.countE) > 0 ? 1 : 0);
    } else {
        // Need to swap sides - queue the swap
        swap = SWAP;
        dc_orient(ori);
    }
}
```

Geometry constants:
- Rod spacing: 8mm
- Gear ratio to rod: 20.082mm per 1800 counts
- Maximum orientation: ±60° (±π/3 radians)
- Theoretical maximum: ±68.28°

### Module Position Calculation

Stepper-based module positioning along shell rail:

```c
void module_move(float pos) {
    // 360:6 gear ratio (pinion to ring), 32× microstepping, 200 steps/rev
    int32_t target = (int32_t)roundf((360.0f / 6.0f) *
            (32 * 200) * pos / 360.0f);
    on_program = 1;
    Stepper_Move(target - stepper.location_steps);
}
```

- Gear ratio: 360:6 (60:1)
- Microstepping: 32×
- Steps per revolution: 200
- Full module travel: 32 × 200 × 60 = 384,000 microsteps per 360°

### TMC2208 UART Protocol

Single-wire UART communication using USART3 at 115200 baud. Write datagrams are 8 bytes:

```c
union WriteReadReplyDatagram {
    struct {
        uint64_t sync : 4;            // 0b0101
        uint64_t reserved : 4;
        uint64_t serial_address : 8;  // 0x00
        uint64_t register_address : 7;
        uint64_t rw : 1;              // 1 = write
        uint64_t data : 32;           // LSB first after reversal
        uint64_t crc : 8;
    };
    uint64_t bytes;
};
```

### Completion Notifications

Module reports status to master controller (0x700):

```c
// Module positioning complete
void module_placed() {
    packet[0] = 0x30;
    CAN_Tx(packet, 8, DEVICE_MASTER);
}

// DC motor homing complete
void dc_home() {
    // ... after STEP3 ...
    packet[0] = 0x31;
    CAN_Tx(packet, 8, DEVICE_MASTER);
}

// Orientation control complete
void dc_placed(uint8_t id) {
    packet[0] = id ? 0x33 : 0x32;
    send_int16(id ? DCR.countE : DCL.countE, packet + 2);
    CAN_Tx(packet, 8, DEVICE_MASTER);
}
```
