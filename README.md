# Arduino Controlled Quadcopter

## Project Description

This is my Senior year Capstone Project. I partially completed my goal of making a functional Arduino Controlled Quadcopter, and in doing so I learned a lot about the trials present in hardware projects.

In flight, the quadcopter will attempt to keep its orientation despite the inherent instability of the quadrotor design. I say attempt because my code is not perfect in the slightest regard. Despite this, I would say that the project was a success. All of the components interact with each other in a manner which is expected for a project like this, and in doing so accomplishes most of the basic tasks which were originally assigned. 


## Installation and User Instructions
If you want to try this code out on your own quadcopter follow these steps:
1. Build your quadcopter using this guide: http://www.brokking.net/ymfc-al_main.html
2. Download and compile the source, then upload it to the Arduino (I used the Arduino IDE, but if you know some other way then go right on ahead and do it that way)

My remote controller is broken. So, I had to remap some of the controls in the quadcopter's software. If you want to have full remote control, then you will need to change a tiny bit of the source.


Change this line in Capstone.ino:
```
pid->PIDControl(pitch, roll, yaw, RCRoll, RCYaw, 0, demandPitch, demandRoll, demandYaw);
```
to this:
```
pid->PIDControl(pitch, roll, yaw, RCPitch, RCRoll, RCYaw, demandPitch, demandRoll, demandYaw);
```

Also, it is more than likely that the PID tune for your quadcopter will be no good.

You can tune the PID by changing these lines in PIDController.h:
```
#define PID_P_GAIN_P 0.02
#define PID_I_GAIN_P 0.25
#define PID_D_GAIN_P 100.0
#define PID_P_GAIN_R 0.02
#define PID_I_GAIN_R 0.25
#define PID_D_GAIN_R 100.0
#define PID_P_GAIN_Y 0.02
#define PID_I_GAIN_Y 0.25
#define PID_D_GAIN_Y 100.00
#define NUM_ELEMENTS_TO_AVERAGE 9
#define MAX_SUM_ERROR 0.04
```
If you know anything about PIDs, then most of those options should make sense to you. 

NUM_ELEMENTS_TO_AVERAGE is the number of samples in the derivative's rolling average. More samples means less noise, but it is also not as quick to react to change.

MAX_SUM_ERROR is the maximum allowed accumulated error. A larger value here leads to a more potent integral.


3. Fly it outside!
    - Make sure the battery is not plugged in
    - Place the quadcopter down (preferably on a level surface)
    - Turn on your remote control and place the throttle into the maximum position
    - Plug in the battery
    - You will hear three ascending notes from the ESCs
    - As soon as you hear two beeps, bring the throttle to the minimum position
    - You will hear three beeps, then one long beep
    - Try to fly it!
    - When you crash, UNPLUG THE BATTERY AND TURN OFF THE REMOTE! I don't want you to lose your fingers when you accidentally bump the throttle on the remote while you are holding your quadcopter!


## Developer Instructions

If you would like you modify your own copy of this code, then go right on ahead. The setup process is identical to steps 1 and 2 in the Installation and User Instructions section.

There are no 'real' tests. However, I did build in a way to visualize what each component is doing. 

In Capstone.ino there are several different visualization modes. You can enable a visualization mode by setting both VISUALIZE and whatever visualization mode you desire to 1. 
```
#define VISUALIZE 0
#define VIS_IMU 0
#define VIS_RC 0
#define VIS_ORIENTATION 0
#define VIS_PID 0
#define VIS_VIBRATION 0
```
All of the visualization modes print to serial. None of the visualization modes require battery power with the exception of VIS_VIBRATION. If you are attempting to visualize vibration, then it might be a good idea to strap down your quadcopter.








# Project Proposal

## Project Overview
The end goal of this project is to make a functional, remote controlled auto-leveling quadcopter.

The following are some goals that should be achieved (from most-least important):
- Arduino should be able to recognise and control attached devices. This is important to demonstrate that all of the hardware is in working order.
- The quadcopter should hover in place. Once this is complete, most of the fundamental control software should be written.
- The quadcopter should be able to move vertically, and stop or chage direction. This *should* be simple, just change the power to the motors.
- The quadcopter should be able to move horizontally, and stop or change direction. This one has the potential of being quick to implement. Changing the desired orientation of the quadcopter in the control code from vertical to some other angle should produce the desired result. 
- The quadcopter's actions should be directed by remote control. This could be super simple or difficult as I (Austin) don't know too much about RC.

Who would be interested in this?
- The DIY Community: The DIY community enjoys it when people demonstrate their knowledge by making something that could have been bought commercially.
- RC Hobbyist: While the cost of RC quadcopters has dropped significantly over the past decade, larger quadcopters are still quite an investment. Making their own from inexpensive components could be a fun and cost effective way to continue their hobby.

## Similar Existing Work
This project is quite similar to many others out there. However, this project was primarily inspired by Brokking(dot)net's [YMFC-AL project](http://www.brokking.net/ymfc-al_main.html). In fact, almost all of the hardware used in their project will be used in this one. Although, none of the code would be shared between the two.

## Previous Experience
The following are some things which I think qualify me (Austin) for this task:
- About a semester's worth of "playing around" with Arduino.
- PID tuning (long story, don't ask).
- Approximately four years of programming for school.
- Countless hours of reading/watching online tutorials.

## Technology
Here are some of the key components that will be used:
- Hardware:
    - Arduino
    - MPU-6050 gyro / accelerometer
    - LiPo battery
    - Electric motors and Props
    - The frame
- Software:
    - Arduino IDE
    - Git
    - A web browser (stackoverflow :wink:)

## Risk Areas
The following are some admissions of (Austin's) ignorance:
- While I have managed devices with Arduino before, I have never done so many at one time.
- Most of my previous Arduino projects were far simpler than this. They generally only did a few things each.
- While I understand the underlying concept of a PID controller (and have tuned them), I have never implemented one before.
- Testing will be difficult. The code must work intimately with real, physical hardware. I do not have much experience dealing with that.

## Product Backlog
What needs to be done:
- ~~Ensure all components are functional~~
- ~~Arduino must be able to control / read data from all devices individually~~
- ~~Arduino must be able to control / read data from all devices simultaneously~~
- ~~PID controller must respond to changes in quadcopter orientation~~
- ~~PID controller should keep the quadcopter in the desired orientation along a single axis~~
- ~~PID controller should keep the quadcopter in the desired orientation along two axises independently~~
- PID controller should keep the quadcopter in the desired orientation along two axises simultaneously
- The quadcopter should be able to hover in place
- The quadcopter should be able to change hover height
- ~~The quadcopter should be able to move horizontally~~
- ~~The quadcopter should be Remote Controlled~~







# Acknowledgements

I would like to thank professor Barry for his help on this project. Without it, I doubt I would have made nearly as much progress.
