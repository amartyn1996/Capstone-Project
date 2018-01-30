# Arduino Controlled Auto-Leveling Quadcopter
Developers: Austin Martyn

## Project Overview
The end goal of this project is to make a functional, remote controlled auto-leveling quadcopter.

The following are some goals that should be achieved (from most-least important):
- Arduino should be able to recognise and control attached devices. This is important to demonstrate that all of the hardware is in working order.
- The quadcopter should hover in place. Once this is complete, most of the fundamental control software should be written.
- The quadcopter should be able to move vertically, and stop or chage direction. This *should* be simple, just change the power to the motors.
- The quadcopter should be able to move horizontally, and stop or change direction. This one has the potential of being quick to implement. Changing the desired orientation of the quadcopter in the control code from vertical to some other angle should produce the desired result. 
- The quadcopter's actions should be directed by remote control. This could be super simple or difficult as I (Austin) don't know too much about RC.

Who would be interested in this?
- The DIY Community: The DIY community enjoys it when people demonstrate their knowlege by making something that could have been bought commercially.
- RC Hobbyist: While the cost of RC quadcopters has dropped significantly over the past decade, larger quadcopters are still quite an investment. Making their own from inexpensive components could be a fun and cost effective way to continue their hobby.

## Similar Existing Work
This project is quite similar to many others out there. However, this project was primarily inspired by Brokking(dot)net's [YMFC-AL project](http://www.brokking.net/ymfc-al_main.html). In fact, almost all of the hardware used in their project will be used in this one. Although, none of the code would be shared between the two.

## Previous Experience
The following are some things which I think qualify me (Austin) for this task:
- About a semester's worth of "playing around" with Arduino.
- PID tuning (long story, don't ask).
- Aproximately four years of programming for school.
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
- Ensure all components are functional
- Arduino must be able to control / read data from all devices individually
- Arduino must be able to control / read data from all devices simultaneously
- PID controller must respond to changes in quadcopter orientation
- PID controller should keep the quadcopter in the desired orientation along a single axis
- PID controller should keep the quadcopter in the desired orientation along two axises independently
- PID controller should keep the quadcopter in the desired orientation along two axises simultaneously
- The quadcopter should be able to hover in place
- The quadcopter should be able to change hover height
- The quadcopter should be able to move horizontally
- The quadcopter should be Remote Controlled

