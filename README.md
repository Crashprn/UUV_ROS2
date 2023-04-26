# Overview

This repository contains the code that controls the UUV. The code can be found in 3 sections:

## Arduino Code:

This code is how the Arduino communicates with the computer to decode information like motor PWM values and send back messages such as the UUV orientation. The Arduino IDE code that is on the current Uno is in the folder Control_Motor_Serial.

### Message format

For the message format, the Arduino assumes that every byte of information it receives is in a specific order. This order is a sign to indicate forward or reverse spin and a PWM value or the amount of spin for the motor. An example of this message is shown below:

32 92 32 92 45 92 32 92

This then decodes to:

-92 Right Motor, -92 Left Motor, +92 Back Motor, -92 Front Motor

As can be seen, the Arduino understands 32 as negative spin and 45 as positive spin. After understanding this, what the Arduino is doing can be understood much better. 

### Setup

The first thing that needs to be done is defining the Arduino pins that will be used to control the motors. As can be seen they are grouped in sets of 3, PWM pin, IN1 pin, IN2 pin. Where the IN pins control the logic of the motor controller (set the polarity or spin direction of the motor). Next we make available memory to read in our message, convert our message to signed integers, and store our previous values.

We also configure the BNO-055 IMU and make sure it can be detected by the Arduino. After this we see a call to confirmCycle, which essentially acts as a stop gate for the program until the ROS program is ready to progress (Will go over this in the confirm cycle section). We then calibrate the IMU and then we are ready to receive messages

### Loop

In the main loop, the bulk of the time is spent waiting on messages to come in. As a result, things are only executed if the Arduino has 8 bytes of information waiting to be read (Motor message is 8 bytes!). Once there is a message waiting to be read, the Arduino reads the message and decodes the message (Message section). Once this is done, the Motor values (in newPinStates) are then used to change the pin values and spin the motors.

Finally, the orientation, and acceleration of the of the UUV is read from the IMU and sent back to the ROS program for further processing.

### Receive Serial



## ROS2 Code:

This code is contained within the ros_ws folder, and if you haven't already do the ROS2 tutorials found here: [Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

Once you complete the tutorials, a lot of the process for how the code works will be a lot more clear. And because the tutorials are available online ROS2 specific things will not be explained in this documentation.

## Hardware Implementation:

This will go over how to set everything wiring wise because the code assumes some things are static (ie specific arduino pins are bound to a specific motor).
As a result this section will have a lot of pictures to show you enough to understand how the wiring is working.

Bay Station IP: 129.123.120.58

Onboard IP: 129.123.121.96


Helpful DHCP Server links

Setting Static Ip address:
https://linuxconfig.org/change-ip-address-on-ubuntu-server

Configuring DHCP server:
https://www.youtube.com/watch?v=1csFmQeXHlg&t=1s
