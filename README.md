# Lab8 - Final Project (Self-Leveling Robot)
Michael Sell (Lab partner: Ben Schaser)

## Table of Contents:

1. [`Resources`](#resources)
2. [`Materials`](#materials)
3. [`Objective`](#objective)
4. [`Robot Design`](#robot-design)
5. [`Robot Assembly`](#robot-assembly)
6. [`Code`](#code)
    * [PID Control](#pid-control)
7. [`Troubleshooting`](#troubleshooting)
8. [`Performance`](#performance)


## Resources
#
* [Self-Levling Bot Instructional](https://www.instructables.com/Arduino-Self-Balancing-Robot-1/)
* [Self-Leveling Bot w/ Video](https://electricdiylab.com/diy-self-balancing-robot/)
* [MotorShield Instruction](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-stepper-motors)
* [Potential Wheel Option](https://www.amazon.com/90-10mm-Black-Robot-Wheels/dp/B00T3MQG7M/ref=sr_1_33?crid=D4G5H5ZWXL8G&keywords=Plastic+Robotic+Wheel+Rubber+Tire+Wheel+100m&qid=1667595648&qu=eyJxc2MiOiIwLjk3IiwicXNhIjoiMC4wMCIsInFzcCI6IjAuMDAifQ%3D%3D&sprefix=plastic+robotic+wheel+rubber+tire+wheel+100m%2Caps%2C231&sr=8-33)


## Materials
#
* `Arduino UNO`

* `2 Stepper Motors (42mm High Torque Hybrid Stepping Motor)`
    * [LINK TO WEBPAGE](https://www.adafruit.com/product/324)
    * [DataSheet][def]

[def]: motordatasheet.jpeg

* `Adafruit Motorshield Kit`
    * [LINK TO WEBPAGE](https://www.adafruit.com/product/1438)
    * <a href="https://cdn-shop.adafruit.com/datasheets/TB6612FNG_datasheet_en_20121101.pdf">View DataSheet</a> 
    
* `Gyroscope Module`
    * [LINK TO WEBPAGE](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B00LP25V1A/ref=sr_1_1?crid=32FG5TYBEF8PB&keywords=MPU-6050%2BModule%2B3%2BAxis%2BGyroscope%2B%2B%2B3%2BAxis%2BAccelerometer%2FG-Sensor%2C%2BI2%2B%C2%B0C%2C%2Be.g.%2Bgenuino%2Bfor%2BArduino%2C%2BRaspberry%2BPi&qid=1664679029&qu=eyJxc2MiOiIwLjcyIiwicXNhIjoiMC4wMCIsInFzcCI6IjAuMDAifQ%3D%3D&sprefix=mpu-6050%2Bmodule%2B3%2Baxis%2Bgyroscope%2B%2B%2B3%2Baxis%2Baccelerometer%2Fg-sensor%2C%2Bi2%2Bc%2C%2Be.g.%2Bgenuino%2Bfor%2Barduino%2C%2Braspberry%2Bpi%2Caps%2C68&sr=8-1&th=1)
    * Main Chip: MPU-6050
    * Power supply: 3~5V
    * Communication mode: standard IIC communication protocol
    * Chip built-in 16bit AD converter, 16bit data output
    * Gyroscopes range: +/- 250 500 1000 2000 degree/sec
    * Acceleration range: ±2 ±4 ±8 ±16g 

* `Batteries`
    * In our case: 8 AA 1.5V rechargeable batteries (TENERGY Premium 2500mAh Ni-MH Rechargeable)

* `Breadboard/wires`

* `Wheels/Tires`
    * Anything around 10cm diameter wheels should suffice. Preferably a good amount of grip for the the bot to maintain traction with various surfaces. In our case, we will be 3D Printing the wheel rims and using rubber tape to wrap around the rims for our "tires".

* `Hardware`
    * Various screws, nuts, etc. that we have yet to determine
    * Brackets for stepper motor installation
    * 4 support beams for structural integrity

* `Structure`
    * Platforms for UNO, battery, and anything else to rest on while offering the bot support. (3 in total, in our case, we will be cutting them out of a 1/4" piece of wood with a laser cutter)


## Objective
#
Create a robot using an Arduino Uno and the above listed materials that is able to autonomously level itself without any human interaction after the bot is turned on. 

## Robot Design
#
Platform Dimensions: 3.5 x 7 x 1/4" (x3)

## Robot Assembly
#

## Code
#
<font size = "1">

```C++
// Code goes here obvi
```
</font>

### PID Control
pain

## Troubleshooting
#

## Performance
#