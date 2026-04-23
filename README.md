# ECE 5780 Group Project (Spring 2026)

## Team Members
- Kyuwon Hwang
- Yoonhyeok Oh
- Ethan Palisoc

## Project Overview
### An Adaptive Cruise Control (ACC) Smart Vehicle
Our system is a battery-powered smart vehicle built on previously designed PCB and motor-driver hardware. Rather than simply driving forward until an obstacle is dected, the vehicle will operate as a closed-loop system with distance regulation. Using ultrasonic sensing, the microcontroller will continuously monitor the distance between the vehicle and surrounding objects and will adjust the vehicle's control parameters accordingly. Additionally, the vehicle is intended to maintain a predefined safe distance relative to the obstacles in front of or behind it.

### Implementation Details
An STM32 will serve as the center of our implemention of a closed-loop motor control that uses PWM output and a PID-based control algorithm. Through this, the vehicle will be capable of smoother motion, reduced oscillation, and more stable behavior when objects move dynamically. Furthermore, because ultrasonic measurements are often noisy, the project will also incorporate signal filtering techniques such as moving-average or low-pass filtering to improve measurement stability before the control logic is applied.

### Expected Behavior
- Bidirectional autonomous distance regulation
  - Front obstacle is too close -> Vehicle will stop and move backward to restore target distance
  - Back obstacle is too close -> Vehicle will stop and move forward to restore target distance
- PID controller will compute distance error relative to target spacing and adjust speed accordingly
