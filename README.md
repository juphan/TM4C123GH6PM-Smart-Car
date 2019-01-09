# TM4C123GH6PM-Smart-Car
Obstacle avoidance car that uses the low-power TI TM4C123GH6PM Tiva Launchpad as the main microcontroller. The device was made to 
replicate the functions of typical Arduino obstacle avoidance vehicles kits, but instead use C programming to program the board. 
The code is relatively simple and structured to allow changes to the code. The project was programmed using Keil uVision 5.

Parts List:
1) TM4C123GH6PM Launchpad
2) Mini-breadboard (x1)
3) Assortment of Male/Male, Male/Female, and Female/Female wires
4) 1k Resistors (x2)
5) 2k Resistors (x2)
6) Fuel Tank Boosterpack
7) HC-SR04 Ultrasonic Sensor (x2)
8) KOOKYE 2PCS Mini Servo Motor 360 Degree Continuous Rotation (x2)
9) Hextronik HXT900 9GR Micro Servo (x1)
10) 6V Battery Pack (4 AA Batteries)
11) Hammer Caster Wheel (x1)
12) Mounts and Wheels (3D Printed)

Pin connections is summarized in the C code file named "main.c" within the code folder. Make sure that the "Echo" pin on the HC-SR04 
sensor does not connect directly to the TM4C123GH6PM Launchpad. This setup prevents damage to the board. Use a 2/3 voltage divider 
setup on a breadboard to connect the "Echo" pin to the appropriate pin on the microcontroller.
