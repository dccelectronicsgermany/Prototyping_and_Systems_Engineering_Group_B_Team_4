## Autonomous Lane-Following and Obstacle-Avoiding Vehicle

 # Project Overview
This project involves the design and implementation of an autonomous vehicle capable of:
- Following a lane based on a color using infrared sensors.
- Detecting obstacles on its path using ultrasonic sensors.
- Changing its route dynamically to avoid obstacles while continuing to follow the lane.
The vehicle is built using an Arduino Uno microcontroller and standard robotic hardware components.
 #  Hardware Components
- Chassis (Robot car frame)
- 2 Motor controllers (for driving the wheels)
- 2 Tires
- Breadboard (for circuit prototyping)
- Arduino Uno (central control unit)
- 2 Ultrasonic sensors (for obstacle detection)
- 2 Infrared sensors (for lane detection based on color contrast)
- LiPo Battery (for powering the system)
- ON/OFF Switch (for power management)
 # Functionality Description
- Lane Following:
The vehicle uses two infrared sensors placed underneath to detect lane markings. It adjusts the motor speed and direction to stay within the lane based on color contrast (typically dark lines on a lighter surface).

- Obstacle Detection and Avoidance:
Two ultrasonic sensors monitor the surroundings. When an obstacle is detected within a predefined distance:
The vehicle slows down.
Makes a directional decision (left or right) based on available space.
Reorients itself and continues following the lane.

- Power Management:
The vehicle is powered by a rechargeable LiPo battery, and the system can be turned ON or OFF using a physical switch.
