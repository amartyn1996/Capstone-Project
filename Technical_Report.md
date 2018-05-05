# Arduino Controlled Quadcopter
Austin Martyn
5/6/2018

# <a name="Abstract"></a>Abstract

Quadcopters are helicopters which use four rotors for propulsion and stabilization. Precise control hardware and software is required for stabilization, as the quadrotor design is inherently unstable. Many hobbyists and remote control enthusiasts build their own quadcopters from kits ordered online. Only a few write their own control software though, as inexpensive flight controllers are readily available. Those that do however, often find it to be a challenging and rewarding experience. 

This report covers an attempt to write the software for a functional quadcopter. It covers both the general theory and application details embodied in the software; as well as the process by which the software was made and the obstacles faced during the experience. The result only partially completed its objective. However, there is still much that could be learned from the process.

Keywords:

Arduino, Quadcopter, Proportion Integral Derivative Controller, Inertial Measurement Unit, Electronic Speed Controller, Pulse Width Modulation, Capstone, PID Tuning

# <a name="Table-of-Contents"></a>Table of Contents

1. [Abstract](#Abstract)
2. [Table of Contents](#Table-of-Contents)
3. [Introduction and Overview](#Introduction-and-Overview)
4. [Design, Development, and Testing](#Design,-Development,-and-Testing)
5. [Results](#Results)
6. [Conclusions and Future Work](#Conclusions-and-Future-Work)
7. [References](#References).

# <a name="Introduction-and-Overview"></a>Introduction and Overview

## Problem and Objectives

This project's goal is to write the software for a functional quadcopter. This is a difficult task, as the software must seamlessly interact with all of the hardware components and perform the actions required to correct for any deviation from the desired orientation in real time. If it does not, the quadcopter will either crash, or not get off of the ground in the first place. Fortunately, this problem, and many others like it, have been solved for a while now. There are a variety of methods to communicate between hardware devices like I2C and Pulse Width Modulation (PWM), and there is an entire branch of Systems Engineering called Control theory; from which concepts like the Proportion Integral Derivative (PID) Controller could correct for orientation error in flight. Software design principles, like encapsulation, are also used. Good software design minimizes the need for code refactoring and reduces coupling.

## Features

The following are the key features which were planned for the project:

- The software should be able to recognize and control all attached devices.
- The quadcopter's actions should be directed via Radio Control.
- The software should know the quadcopter's current orientation.
- The control software should attempt to maintain the quadcopter's desired orientation.
- The quadcopter should be capable of level flight.

With the exception of level flight, all of the features listed above were implemented in full. While the software can make the quadcopter fly, the flight is not stable. 

## Hardware Overview and Definitions

A functional quadcopter requires at least these eight separate hardware components: A frame, battery, Inertial Measurement Unit (IMU), four electric motors, four propellers, four Electronic Speed Controllers (ESCs), radio control (RC) receiver, and a flight controller.

- Frame:
The frame holds all of the other components together. A frame's shape and size varies depending on the model and manufacturer. This project uses an F450 size frame with an integrated power distribution board. 

- Battery:
The battery powers the electronic components. The one being used is a 30C, three cell, 11.1 Volt Lithium Polymer Battery. 

- Inertial Measurement Unit (IMU):
An IMU is a device with an integrated gyroscope and accelerometer. The IMU used in this project is the GY-521 MPU6050 [1,2].

- Electric Motors:
Electric motors, along with the propellers, provide the thrust required for a quadcopter to stay aloft. The quadcopter uses four A2212 1000kv brushless motors.

- Propellers:
Propellers provide lift when spun. They come in two orientations: clockwise and counter-clockwise. The orientation denotes what direction to spin to achieve downward thrust. This project uses four 10x4.5 propellers, two clockwise and two counter clockwise.

- Electronic Speed Controllers (ESCs):
ESCs control the electric motors. They can safely handle the motor's inductive load. The speed at which the ESCs drive to motors is dependent upon a Pulse Width Modulated signal. The ones used in this paper are generic 30A brushless ESCs.

- Radio Control (RC) receiver:
The RC receiver listens for and amplifies radio signals. Hobby RC receivers often listen for signals from radio remote controllers. A 7 channel, 2.4GHz RC receiver was used.

- Flight Controller:
A flight controller manages the other components of the quadcopter. It receives data from the IMU and RC Receiver, runs that data through a control algorithm, then sends a signal to the ESCs so that the quadcopter stays in its desired orientation. The flight controller for this project is an Arduino UNO.

- Arduino UNO:
The Arduino UNO is a board with an embedded programmable microcontroller, and it is where our program resides. 


The components were assembled in the same manner as stated in Brokking.net's "Project YMFC-AL" tutorial [3].


The following are additional definitions:

- Pulse Width Modulation (PWM):
PWM is a method of encoding a message in pulsing signal. In our case, the pulse is in the form of a square wave. With the time between the rising edges of the square wave being between four and twenty milliseconds, and the length of the pulse between one and two milliseconds.

- Inter-Integrated Circuit (I2C):
A method of passing data between multiple hardware devices.

- Proportion Integral Derivative (PID) Controller:
PID Controllers are feedback loops which attempt to reduce the error of a system from a set point. The control response comes from three terms: Proportion ( current error ), Integral ( sum of previous errors ), and Derivative ( change in error ). Before control is applied, the resulting values from each term is multiplied by some gain. The gain is effectively the controller's sensitivity to each term.

- Interrupt
An interrupt is a function that is called as soon as some condition is met. They can happen at almost any time during code execution. Thus, interrupt functions are often kept short in the case they are called in time sensitive code.

- Pin Change Interrupt
Pin Change Interrupts are interrupts that occur whenever a pin's voltage passes some set value.

# <a name="Design,-Development,-and-Testing"></a>Design, Development, and Testing

## Design

The following is a high level overview on how the quadcopter software operates. First, the IMU updates its sensor data, and then passes it on to the Orientation Handler. The Orientation Handler interprets that data and returns the current orientation. The desired orientation and throttle are read in via Radio Control. Then, both the current orientation and the desired orientation are sent to the PID controller. The PID Controller gives us the required response to correct for the error between the two. Finally, the ESCs carry out the response, and the quadcopter moves to its desired orientation. This high level flow of data can be seen implemented in the Capstone class, and is visualized in Figure 1. The details of the process is encapsulated into five distinct classes: IMU, OrientationHandler, RCReceiver, PIDController, and ESC.

Figure 2 is a useful visual reference for the following section.

- IMU: 
The IMU and MPU6050 classes are an implementation of Strategy pattern. IMU consists of virtual functions which are implemented in the MPU6050 class. Thus, the following implementation details will refer to it. The MPU6050 hardware holds the change in its gyroscope angle and acceleration from its accelerometer per update in a pair of registers. External components can request the values from those registers via I2C. The software uses the built in Arduino Wire library to do this [4]. After reading the raw acceleration and gyroscope values from the hardware, the values are converted to a form more useful. The accelerometer data is converted to g's (Approximately 9.8m/s^2), and the gyroscope data is converted to degrees per second. The converted accelerometer data and gyroscope data can be accessed externally from the getSensorData() method.

- OrientationHandler:
The OrientationHandler retrieves the acceleration and gyroscope data from the IMU. Both are needed to determine the current orientation. While it is possible to do so with only one or the other, there are drawbacks to that approach. Using only the accelerometer is less than ideal for two reasons: One, quadcopters vibrate a lot. That vibration creates a lot of noise in the measured acceleration. That noise can be averaged out over multiple updates though. However, averaging out the noise makes changes in the acceleration value less responsive. Two, the quadcopter's acceleration direction might not be the same as gravity's acceleration direction. If the quadcopter suddenly accelerates, then it will think that the acceleration vector is up; the control software will attempt to maintain the quadcopter's orientation to 'up', and will eventually crash. Using only the accelerometer creates a dangerous positive feedback loop. On the other hand, using only the gyroscope is not ideal either. The gyroscope values 'drift' over time. This 'drift' is from both how often the gyroscope values are sampled and the inherent drift of the gyroscope itself. This 'drift' eventually causes the quadcopter's 'up' to no longer be the same as actual up. Thus, the only option left is to use both together. The orientation is primarily computed from the gyroscope data and the accelerometer data corrects for 'drift.' The orientation stays accurate so long as the quadcopter does not sustain long periods of acceleration. 

- RCReceiver:
RCReceiver listens on four pins for a PWM signal from the RC Receiver hardware. The length of the pulse on each pin determines the amount of activation. For example, a 1ms pulse is 0% activation, 1.5ms pulse is 50% activation, and 2ms pulse is 100% activation. The four signals correspond to roll, throttle, pitch, and yaw. An example activation for a desired orientation of up and a throttle of maximum would be: pitch 50%, roll 50%, yaw 50%, throttle 100%. The pulse timings are determined by pin change interrupt. When a pin goes high, the current time is saved. When a pin goes low, the pulse length is the difference between now and the saved time.

- PIDController:
When the PIDController receives the current orientation and desired orientation, it computes the difference between the two. This difference is the error. The proportional term is simply that error. The integral term is the sum of all of the previous errors. The derivative is a rolling average of the change in error over some set number of cycles. The rolling average is required, as the noise present in the derivative measurement is very high. Unfortunately, the more samples in the rolling average, the less responsive the derivative term is. A high number of samples effectively negates the purpose behind the derivative term. The response is the sum of each term multiplied by its respective gain.

- ESC
ESC receives the response and throttle, then assigns an appropriate throttle setting for each motor. For example, if large pitch up force is demanded, then the front two motors would be assigned a high throttle value and the back two a low value. These throttle values are then converted to pulse lengths. A pulse is then sent to each motor's respective ESC. The manner by which this happens is very similar to how RCReceiver receives PWM signals, except in reverse. A pulse stays high until the difference between the current time and the start time is greater than the desired pulse length.


![Figure 1](https://i.imgur.com/bDffPlC.png)
Figure 1

![Figure 2](https://i.imgur.com/xUXGB8o.png)
Figure 2

## Development

An iterative approach was taken towards implementing new features. First, initial research was performed. Second, basic functionality was implemented and tested. Third, the remaining functionality was implemented and tested. If some new functionality was required, the process will begin again but for only that new feature. Upon receiving the assignment, work began right away. After a week of planning and research, the first thing implemented was IMU functionality. It was finished rather quickly, but the next one, OrientationHandler, took a little longer. Converting rotations from the gyroscope to pitch, roll, and yaw was a little bit more difficult than anticipated. An issue which exemplifies this is as follows: 

Imagine a level quadcopter. Have it pitch up 90 degrees, then roll left 90 degrees, then pitch down 90 degrees. The resulting orientation is a 90 degree yaw left from its original orientation. The yaw changed without any rotations along the yaw axis.

That issue, in one form or another, existed until it was finally fixed much later in development. The next feature to be implemented was ESC control. Initially control was 'hard coded.' Think, slowly ramp up power over a few seconds, hold max power for a second, then turn off. Next, ESCs were controlled directly by orientation with some set throttle amount. This was effectively proportional only control. The next thing to be implemented was RCReceiver. Initial implementation did not take very long, but later safety features were added. For example, if it has not received a signal in a while or if a pulse was too long, then the throttle is set to zero. Lastly, the PID controller was implemented, and its output was 'piped into' ESC.

After some initial tuning, something went wrong. The ESCs were no longer listening to the Arduino and were randomly changing throttle. Eventually, the problem was discovered with some outside help and an oscilloscope. The problem turned out to be two separate issues regarding power. The first issue was that the battery being used was over charged. The second issue was that the Arduino received about half the voltage it should have. Whenever there was high power draw, the voltage would drop and shut down the Arduino. Eventually the Arduino would turn back on and continue where it left off. Finding and resolving these issues took a while. Finally, PID tuning resumed and was then promptly halted when a leg on the quadcopter's frame broke off. This happened twice to the same leg. The first time that happened it was repaired with epoxy, and the second time it was repaired by first gluing the two pieces, then bolting them together with a piece of metal. The before and after photographs of this repair are Figures 3 and 4 respectively. PID tuning continued until the end of the semester. 

![Figure 3](https://i.imgur.com/IjSvznA.jpg)
Figure 3

![Figure 4](https://i.imgur.com/bZDy4ll.jpg)
Figure 4

## Testing

The testing methodology of features for this project was a little bit different from how software is usually tested. Instead of unit testing, each feature's actions can be visualized over serial monitor [5]. This is done by setting the preprocessor macro definition 'VISUALIZE' in Capstone to 1, and the desired feature to be visualized macro definition also to 1. This was done because, in practice, testing specific features this way is very simple, fast, and accurate given the person doing the testing knows what they are doing. The development time saved by not writing tests was spent implementing features.

# <a name="Results"></a>Results

## Features

- The software should be able to recognize and control all attached devices.

A solid success. The Inertial Measurement Unit, Radio Control Receiver, and Electronic Speed Controllers are all being used to their full potential. Gyroscope and accelerometer data is retrieved from the IMU. Desired control settings are gathered from the RC Receiver, and the throttle of each ESC can be precisely controlled. 

- The quadcopter's actions should be directed via Radio Control.

Radio Control is fully functional. Throttle, roll, pitch, and yaw commands can influence how the quadcopter behaves.

- The software should know the quadcopter's current orientation.

Another success. The OrientationHandler is able to maintain an accurate approximation of the quadcopter's current orientation even when undergoing rapid maneuvers and vibration. However, it cannot handle prolonged high acceleration in a single direction. The quadcopter eventually will think that 'up' is in the acceleration's direction.

- The control software should attempt to maintain the quadcopter's desired orientation.

The PID controller accomplished this task with elegance. PID controllers are simple to implement, and provide robust tunable control. The decision to use a PID controller in this project was a wise one.

- The quadcopter should be capable of level flight.

Unfortunately, this could not be finished in time. The only flight achievable is short lived and relatively unstable. It is believed that better PID controller tuning is required to accomplish this task.

## Performance

The following is a typical flight experience: First the battery is plugged in. The ESCs perform their calibration. Throttle is applied and the quadcopter takes off. The quadcopter continues to move in the initial horizontal direction. While it is doing this, it continuously oscillates back and forth from the vertical position. After a few seconds, the quadcopter will either lose control and crash, or be far enough away from the operator that it is required to land. 

# <a name="Conclusions-and-Future-Work"></a>Conclusions and Future Work

## Summary

Writing the software for a quadcopter's flight controller was an interesting experience. The software had to maintain the quadcopters's orientation in flight despite the instabilities inherent to the design. Here is the theory behind how the task was accomplished: First, acceleration and gyroscope data is read from the Inertial Measurement Unit. Second, that data is used to find the quadcopter's current orientation. Third, the desired orientation and throttle is retrieved via Radio Control. Fourth, a PID controller uses the current and desired orientation to find the appropriate control response to reduce the error between the two. Fifth, the response and throttle are interpreted and a signal is sent to the Electronic Speed Controllers. Sixth, the quadcopter moves, and the cycle starts over again. The resulting program, with the appropriate PID controller tune, should be able to make the quadcopter fly stably.However, at this moment the tune is not ideal. In testing, the quadcopter is only able to stay aloft for several seconds until it either crashes or must land. 

## Some Advice

The largest problems that occurred while developing the software were not related to the software itself. Solving electrical and hardware issues consumed a significant portion of the time that would have been better allocated to implementing features. Some words of advice on how to avoid the types of problems encountered in this project: Regularly check the status of a charging battery even if the charger is 'designed to prevent overcharging.' Make sure the device is receiving enough voltage when the power source is under heavy load. Buy an oscilloscope, or find a friend who owns one and use theirs. When it comes to hardware projects, breaking parts is inevitable. Make sure the components are available and have a plan for how they will be fixed when they do. Another piece of advice, this time regarding PID tuning: Streamline the tuning process. Do this by minimizing the time between testing each tune. Have multiple batteries so that when one runs out, testing can continue on the other one while the old one charges. Alter the PID Controller tune with a laptop so that changes can be made immediately after a flight ends. Keep spare propellers and an Allen wrench on hand so that when a propeller breaks it can be fixed immediately. 

## Suggestions for Future Work

To those who wish to continue this line of work, here are a few ideas one might be interested in exploring: One, use higher quality hardware and mount it sturdily to the frame. Two, have the quadcopter actively attempt to maintain its current position. Currently it only attempts to maintain its orientation. Three, add a Global Positioning System Receiver to the quadcopter. Implementing this and suggestion two would allow for autonomous flying. Four, log the flight data. It could come in handy when determining if a PID controller tune is more stable than another.

# <a name="References"></a>References

[1] InvenSense Inc. 2013. MPU-6000 and MPU-6050 Product Specification Revision 3.4.

[2] InvenSense Inc. 2013. MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2.

[3] J.M. Brokking. 2018. Project YMFC-AL - The Arduino auto-level quadcopter. Retrieved from http://www.brokking.net/ymfc-al_main.html

[4] Arduino Wire Library. https://playground.arduino.cc/Main/WireLibraryDetailedReference#wire

[5] Arduino Serial Library. https://www.arduino.cc/reference/en/language/functions/communication/serial/
