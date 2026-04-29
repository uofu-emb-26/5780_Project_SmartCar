# ECE 5780 Group Project (Spring 2026)
**Project:** An adaptive cruise control smart vehicle using the STM32 microcontroller

## Team Members
- Kyuwon Hwang
- Yoonhyeok Oh
- Ethan Palisoc

## Project Overview
Our system is a battery-powered smart vehicle built on previously designed PCB and motor-driver hardware. Rather than simply driving forward until an obstacle is dected, the vehicle will operate as a closed-loop system with distance regulation. Using ultrasonic sensing, the microcontroller will continuously monitor the distance between the vehicle and surrounding objects and will adjust the vehicle's control parameters accordingly. Additionally, the vehicle is intended to maintain a predefined safe distance relative to the obstacles in front of or behind it. Thus, this project demonstrates, on a smaller scale, the expected functionality of autonomous embedded systems such as self-driving cars as they must continuously sense their environment, interpret changing conditions in real time, and adjust their motion accordingly.

### Implementation Details
An STM32F072B-DISCO will serve as the center of our implemention of a closed-loop motor control that uses PWM output and a PID-based control algorithm. Through this, the vehicle will be capable of smoother motion, reduced oscillation, and more stable behavior when objects move dynamically. Furthermore, because ultrasonic measurements are often noisy, the project will also incorporate signal filtering techniques such as moving-average or low-pass filtering to improve measurement stability before the control logic is applied.

### System Behavior
The following is the expected behavior as was mentioned in the project proposal:
- Bidirectional autonomous distance regulation
  - Front obstacle is too close -> Vehicle will stop and move backward to restore target distance
  - Back obstacle is too close -> Vehicle will stop and move forward to restore target distance
- PID controller will compute distance error relative to target spacing and adjust speed accordingly

At the time of this commit, this is its current behavior:
- The vehicle moves forward by default
- It stops when the front ultrasonic sensor detects an obstacle within 10cm
- Hardware issues currently hindering functionality:
  - Back sensor has not been properly integrated
  - Vehicle does not turn, only moving back-and-forth

## Project Setup
Here are the components used for this project:
- STM32F072B-DISCO Microcontroller
- 2WD Smart Car Chassis w/ DC Gear Motors
- Custom-designed PCB w/ L298N Motor Driver
- 2x HC-SR04 Ultrasonic Sensors

### Component Wiring
- STM32 to Motor Driver
  - PB4 &rarr; ENABLE
  - PA6 &rarr; INPUT1
  - PA5 &rarr; INPUT2
  - GND &rarr; Motor Driver GND
- STM32 to Ultrasonic Sensors
  - PA9 &rarr; TRIG
  - PA1 &rarr; ECHO
  - +5V &rarr; VCC
  - GND &rarr; GND
- Chassis Connections 
  - Cut & solder the loose ends of both **red** motor wires together &rarr; **MTR_RED**
  - Cut & solder the loose ends of both **black** motor wires together &rarr; **MTR_BLK**
  - External Power Supply (+) &rarr; Motor driver VCC
  - External Power Supply (-) &rarr; Motor driver GND
- Connect STM32 GND, Sensor GND, Motor driver GND, and External Power Supply (-) together
