# **3DC: Modular Ultrasonic 3D Scanner**

*CT-Inspired Feedback System for 3D Printing*

# **What It Is**

A modular scanning system that creates 3D thermal distribution maps and performs geometry scanning using ultrasonic measurement principles adapted from computed tomography (CT). The scanner consists of a ring-shaped motion shell that enables continuous rotation, compact carriage modules that position sensors anywhere along the ring, and high-frequency ultrasonic transceivers (300kHz) that perform precise point-to-point measurements. All components communicate via CAN bus through custom-designed slip rings, allowing infinite rotation while maintaining reliable data and power connections.

# **The Vision: Why Full Feedback Matters**

Consider how CAD software works: you design, inspect, modify, and iterate—all within a seamless workflow. You can undo operations, edit specific features, and refine your model continuously. This cycle of creation and feedback is what makes digital design powerful. I believe 3D printing should evolve the same way. Even as printing technology advances toward the dream of "arranging materials at wanted locations," the fundamental workflow remains broken without comprehensive feedback. Current printers operate essentially blind—they execute instructions without truly perceiving what they've created or how the environment affects the process.

For 3D printing to become the versatile manufacturing technology we envision, it needs the same make-check-change cycle that CAD provides digitally. This project is my attempt to build that feedback infrastructure versatile enough to capture multiple volumetric environmental data with a single scanner—in hopes of someday enabling not just error detection, but eventually the kind of iterative physical editing that would transform how we approach fabrication.

# **How It Works**

\[Sensor A\] \----sound wave---- \[Sensor B\]			  
     \\                          	           /				  
      \\   measured: time-of-flight     /				  
       \\  known: distance       	        /				  
        \\  → calculated: sound speed				  
         \\                                      /				  
          \----multiple angles----				  
                   ↓						  
    \[Sinogram / Raw Data\]					  
                   ↓						  
    \[Inverse Radon Transform\]				  
                   ↓						  
    \[2D Sound Velocity Map \= Temperature Map\]		  
                   ↓						  
    \[Use as calibration for proximity ToF\]			  
                   ↓						  
    \[Gather proximity ToF to form point clouds and 3D scans\]	

## **Measurement Geometry**

The mechanical structure enables independent modules to carry sensors to any position along the circular rail and adjust their orientations. This means any measurement between two points on the circle—any chord, any diameter—is achievable, limited only by the carriage module's angular range. The system can replicate the beam geometries used in CT scanning: parallel beams, fan beams, and more. While ultrasonic sensing is inherently point-to-point (especially with 300kHz high-directivity transceivers), the positioning freedom means the system can collect measurements between any two points on the rail.

## **From Sound Speed to Temperature Maps**

Sound travels through air at a speed determined primarily by temperature—approximately 0.6 m/s faster per degree Celsius. By measuring time-of-flight between known positions, the system calculates the average sound velocity along each path. Collecting these measurements across multiple angles and positions creates a sinogram, which inverse Radon transformation reconstructs into a tomographic image of sound velocity distribution. Since sound velocity maps directly to temperature, this yields a 2D thermal slice. Stack multiple slices, and you have a 3D thermal map of the printing space.

## **Calibrated Proximity Measurement**

Ultrasonic proximity sensing is typically considered a cheaper but less precise alternative to optical methods like LiDAR. Temperature variations degrade accuracy because they affect sound speed unpredictably. However, with the thermal map already computed, the system can compensate for these variations. Combined with advanced signal processing for the high-frequency transceivers, ultrasonic ToF becomes a viable high-precision proximity sensor at far lower cost than optical alternatives. This approach also sidesteps a fundamental limitation of optical sensors: transparent and reflective objects that confuse laser-based systems are detected normally by sound.

## **The Core Insight**

What began as an attempt to avoid the failure modes of optical sensors—transparent objects, reflective surfaces—evolved into something more: a single sensor type that provides both environmental awareness (thermal mapping) and geometric measurement (calibrated proximity). The combination of established concepts (CT reconstruction, ultrasonic ToF) with a novel application context created capabilities that neither approach offers alone.

# **What I Built & What’s still in progress**

I developed the complete mechanical architecture to prototype and demonstrate this vision. The motion shell provides the scanning geometry; the carriage modules achieve precise positioning in an extremely compact form factor; and the sensor boards handle high-frequency ultrasonic generation and reception with the signal integrity required for accurate timing measurements. Every component—mechanical design, electronics, firmware—was designed from scratch to meet the specific requirements of this system.

What’s incomplete yet is the accurate sensor measurement and therefore full validation of its scanning capabilities.

Please join the following link for the demo video: [https://youtu.be/h6HlKs1gbwU](https://youtu.be/h6HlKs1gbwU)

# **Key Achievements**

## **The Motion Shell**

![][image1]![][image2]

# **![][image3]**

## The motion shell is a ring-shaped mechanical framework (approximately 444mm outer diameter) that serves as the primary scanning structure. It features fully connected carriage rails with integrated gear tracks, custom ENIG-plated pcb based slip rings for continuous power and CAN bus communication during infinite rotation, and stepper-driven gearboxes for precise positioning. The shell can rotate continuously in the horizontal plane and tilt up to 90° via external motion axes, enabling full hemispherical scanning coverage. All structural components were designed for segmented FDM printing and assembled to maintain circularity and gear mesh accuracy.

## 

## **The Carriage Module**

![][image4]**![][image5]**

# A compact mechatronic unit measuring 46×30×77mm that carries sensors along the shell's rail. Each module contains: a NEMA8 stepper motor for linear positioning along the rail, two N20 geared DC motors with magnetic encoders for dual-axis sensor orientation control, and a custom controller board (STM32F103-based) managing all motor control and CAN communication. The module interfaces with the shell's slip ring contacts for tool-free installation and removal. Achieving precise DC motor positioning in this form factor required developing a custom control algorithm, as conventional approaches like PID, those with speed based controls, failed due to torque-speed coupling inherent to DC motors.

# **The Sensor Module**

![][image6]![][image7]A high-frequency ultrasonic transceiver system designed for 300kHz operation—a frequency range typically found in research-grade measurement systems rather than consumer sensors. The board uses DDS for precise waveform generation, a BJT push-pull buffer for high-voltage transducer drive, and 4-channel TX/RX switching for multi-transducer operation. The reception path features a two-stage amplifier with Bessel filtering to preserve waveform shape for phase-based ToF measurement, rather than simple threshold detection used in typical ultrasonic sensors. Individual TX and RX functions have been validated; integration debugging is in progress.

# **An Assembled Device**

![][image8]

![][image9]  
Assembled device means the motion shell filled with carriage modules. The module can be easily added and removed when the shell is open. The carriage modules also can travel freely inside the shell’s rail when closed. When tilted, carriage modules at the tiltable side of the shell will be moved simultaneously, allowing for positioning other than just around the circle.

# **What It’s Capable of**

- Continuous 360° rotation of the motion shell with precise stepper-driven positioning  
- Independent carriage module travel along the full circular rail  
- Dual-axis sensor orientation control via DC motors with custom positioning algorithm  
- Real-time CAN bus communication across slip ring contacts during rotation  
- Tool-free module installation/removal when shell is open  
- 90° tilt capability for hemispherical scanning coverage  
- High-frequency ultrasonic signal generation at 300kHz  
- Waveform-preserving signal reception and ADC capture (validated individually)  
- Coordinated multi-axis motion via CAN network commands

# **Technical Challenges**

The core challenge was translating conceptual design into physical reality at every level:

* **Mechanical:** Beyond fitting three motors, encoders, and control electronics into a small carriage module envelope, the deeper challenge was designing unconventional mechanical structures at this scale—then making them actually work. The motion shell required maintaining circularity and folding its half for future use of measurement across different viewports as well as convenient module replacements. The carriage module's dual-axis orientation controller demanded tight tolerances between gear trains, motor mounts, and the sensor attachment point. CAD designs that looked feasible often revealed clearance issues, assembly impossibilities, or structural weaknesses only after printing and testing.

* **Electronics:** This was my first custom sensor circuit, requiring multiple design iterations to understand what actually works for high-frequency ultrasonic measurement. Getting the DDS-to-opamp signal chain to drive transducers with sufficient current capacity was the first hurdle. Far more difficult was capturing clean received signals—early attempts picked up noise from the TX path, power rails, and switching transients. Multiple filter topologies failed before the Bessel configuration worked. Fitting the signal into the STM32's integrated opamp and ADC voltage range added another constraint layer, requiring careful biasing and gain staging.

* **Communication:** Creating custom slip rings from ENIG-plated PCBs with spring-loaded ball pin contacts was experimental and arguably risky—but orders of magnitude cheaper than commercial alternatives. The challenge was finding the right pin distribution and contact pressure: too light and connections drop out; too heavy and the gold plating scratches through. Additional complexity came from joining separate PCB arc segments into a continuous ring while avoiding dead zones at the junction points where contacts could lose connection during rotation.

* **Control:** Precision DC motor positioning proved unexpectedly difficult. Standard PID control assumes you can modulate speed smoothly, but DC motor torque varies with speed—reducing PWM to slow down also reduces torque, causing stalls or inconsistent stopping positions. After failed conventional approaches, a phase-based algorithm that maintains near-full output while using encoder feedback to time the deceleration achieved the required positioning accuracy.

# **Current Status**

**Completed:** Motion shell assembly, carriage modules with full motion control, CAN network integration, sensor board hardware.

**In Progress:** Sensor interface debugging, ToF measurement validation.

**Next:** Complete sensor integration, implement reconstruction algorithms, integrate with printer control.

# **Future Plans**

![][image10]

The immediate priority is completing sensor integration—validating ToF measurements and debugging the TX/RX switching circuit. Once point-to-point measurements work reliably, the next phase is implementing the tomographic reconstruction pipeline: collecting multi-angle measurements, building sinograms, and applying inverse Radon transformation to generate thermal maps.

With thermal mapping validated, I can test the core hypothesis: whether computed thermal distribution actually improves proximity measurement accuracy through sound speed compensation. This determines whether the dual-function concept—thermal mapping and calibrated geometry scanning from one sensor type—works in practice.

Longer term, the system needs integration with actual printer control. The current framework operates as a standalone scanner. Connecting it to printer firmware would enable real-time feedback during printing—the original motivation for this project.

The furthest goal remains unchanged: not just error detection, but enabling iterative editing of physical objects. Make-check-change instead of make-check-throw away-remake. The mechanical infrastructure now exists. What remains is proving the sensing concept works, then building the software layer that would let a printer act on what it perceives.

**Additional Resources**

This document provides an overview of the project's core concepts and current progress. For complete technical details and specifications including circuit schematics, mechanical CAD files, firmware source code, and detailed development documentation, please visit the GitHub repository: [github.com/lhtlove/SonicFlow-Scanner](https://github.com/lhtlove/SonicFlow-Scanner)