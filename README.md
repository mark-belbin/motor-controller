# FOC Motor Controller
This project was a multi-year effort to develop a ground-up, brushless motor (BLDC) controller for use in underwater robotics. 

The final controller is designed for T200 or T100 thrusters from BlueRobotics, boasting sensorless FOC and a miniature form-factor that is built right into the thruster.

## Key Features
- Sensorless FOC / sinusoidal commutation
- Input voltage up to 14V
- 250W Continuous Power
- CAN Bus telemetry and control
- Built into the T200 thruster, water-cooled
- Low underwater acoustic noise
- Up to 10-20% better efficiency than standard ESCs
- Great low-RPM operation and control
- *Ludicrous* mode, able to overmodulate and reach higher RPMs


![PCB Front](https://github.com/mark-belbin/motor-controller/blob/master/docs/images/pcb_front.png)

> FOC Thruster Controller V2 Top View

![PCB Back](https://github.com/mark-belbin/motor-controller/blob/master/docs/images/pcb_back.png)

> FOC Thruster Controller V2 Bottom View

# Table of Contents




# Introduction
Brushless motors, referred to as BLDC or PMSM motors, have no physical link between the stator and rotor such as a standard DC motor. Therefore, these motors must be commutated (spun properly) using an external controller. The key is knowing where the motor rotor is relative to the stator, such that the magnetic field of the stator can be generated with accuracy in order to succesfully push along the rotor and get the motor to spin. 

For most hobby BLDC controllers, commonly referred to as electronic speed controllers (ESCs), motor commutation is accomplished through sensorless trapezoidal 6-step control. This commutation method steps through 6 distinct steps in order to keep the motor spinning, based only on the feedback voltage of the three motor phases. With only 6 available control positions, this method of commutation lacks maximum possible efficiency, but its a simple, reliable, and cheap control method. 

This motor controller design uses Field-oriented control (FOC), also known as sinusoidal control or sine wave control. The FOC algorithm used is provided by Texas Instrument's InstaSPIN product line. This enables FOC using sensorless feedback, meaning no extra encoders or sensors are attached to the motor. The TI sensorless system measures only the 3 phase voltages and 3 phase currents of the motor in order to estimate the current rotor position with high accuracy. Also important is the intrinsic properties of the motor, such as the phase resistance, inductance, and the motor flux linkage. InstaSPIN requires a motor calibration procedure be run on each unique motor to automatically determine these intrinsics, if not the FOC algorithm will not run.

This custom FOC controller design will evaluated by comparing against the BlueRobotics Basic ESC R3, a standard hobby controller. Both will run the same BlueRobotics T200 thruster, which is the first intended use of the custom developed controller. 

![T200 and Basic ESC](https://github.com/mark-belbin/motor-controller/blob/master/docs/images/T200_BR_ESC.PNG)

> BlueRobotics T200 Thruster and Basic ESC R3.


# FOC Benefits
FOC has numerous benefits compared to traditional trapezoidal control. The most striking are the increase in efficiecny and reduction in motor noise. This is possible since FOC control applys the most optimal stator magnetic field to push the rotor forward, and as a result generates clean and noise-free sine-wave current wave forms.

A comparison of current waveforms is shown below. Both the custom FOC controller and the Blue Robotics Basic ESC R3 are controlling a submerged T200 thruster at 2000 RPM. The FOC phase currents are much cleaner and closer to sinewaves, while the Basic ESC waveforms are more choppy and trapezoidal.

![FOC Currents](https://github.com/mark-belbin/motor-controller/blob/master/docs/images/phase_currents.PNG)

> FOC phase currents vs the BR Basic ESC R3 at 2000 RPM.

The 


![Efficiency Graph](https://github.com/mark-belbin/motor-controller/blob/master/docs/images/eff_graph.PNG)

> FOC Controller vs the BR Basic ESC R3.


## Motor Controller PCB

## Controller Firmware

## T200 Thruster Mount

# AUV Integration


(Insert pic of capstone AUV)

# Repository Content
The repository is organized into 3 categories; hardware, firmware, and docs. Included in the hardware folder is the latest FOC controller V2, as well as an older version V1, and many test boards built to test and evaluate the project along the way. The firmware folder includes the latest V2 firmware, as well as the V1 firmware and sample test projects that were provided by TI and are useful for debugging. 

