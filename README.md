# The Documentation for Future Engineer 2023
### [Zenith] 


![](https://lh3.googleusercontent.com/u/0/drive-viewer/AJc5JmS-gvzix8rqHiP9ptq7tHeZygsObiNOmIOgPZ77TDPFEsBTKvNW-LatH-ngLn_0nhACZ-FHElf1pMwIivX24kCyNMjfTw=w1920-h929)


<p align="center">
  <img src="https://ybrobot.club/image/YB%20Robot%20logo.png" width="200"/>
</p>
<p align="center">
<b>By Yothinburana School Robot Club</b>
</p>

![Team](https://github.com/OlyPongpapat/Future-Engineer--BOO/assets/122149490/4e4d9215-4092-4590-ab22-bc3f343689a1)
<br>
## About team 
We are Zenith team from Thailand. We Zenith are a team of three people studying at the same school.The first person is me, Chaiwat Chinsupawat who stands in the middle of the picture My role is to design the document. I do this role because I’m the only person on the team who knows about the program and the design of the robot. The second person who stands at the left is Pongpapat Putongkam. His role is to design the robot. We vote for him to be the robot designer because he has short meditation so he will always have a new idea. The last person who stands at the right of the picture is Pawit Nateenantasawasd. His personality is introverted so everyone on the team votes for him to be a programmer because he can concentrate on the program. We name Zenith because Zenith means the best or most successful point or time. We were able to form a team because we attend the same school and share an interest in robotics, which led us to decide to join the school's robotics club.                                         

<br>
Team members and roles:<br>                                                                                                                                          
1.Chaiwat Chaiwat-Document Designer<br>                                                                                                                                                     
2.Pongpapat Putongkam-Mechanic Designer <br>                                                                                                                                            
3.Pawit Nateenantasawasd-Programmer                                                                                      


<br><br><br><br>


## Robot Running (Youtube Links):
What you will see in the first clip is a robot walking, and when it encounters a black line, it will turn right. Then, the ultrasonic sensor's servo will turn left. However, if it encounters a blue line, it will turn left, and the ultrasonic sensor's servo will turn right. We use the ultrasonic sensor to determine the distance between the robot and the wall<br>
Round 1 - [https://youtu.be/AAdyjSv6BT0](https://youtu.be/hPf2NdIELDc?si=pPlFXDwzhR4QGv74)
<br>

In the second clip, you will see that it will be similar to the first clip, but with the addition of obstacle management. In the obstacle management, we will use the Pixy camera for detection. If the camera detects a green-colored block, it will turn left to avoid the block. However, if it sees a red-colored block, it will turn right to avoid the block.<br>
Last Round - https://youtu.be/ON5lbqp0pIs?si=JSA1nNsiVxj0adTu

<br>


### This file contains:

- Designing process
- Programing explain

<br><br><br><br>

# **Part 1: Robot Designing** ✨

<br>

### **The Robot** 🤖

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/blob/main/v-photos/FULL.jpg"  width="400"/>

<br>

### Design & Concept
We use a Customize board name POP-32i that using a 32-bit microcontroller, the STM32F103CBT6, with 128KB of flash memory, so you can program it up to 10,000 times. It also has 20KB of data RAM and a 20MHz clock signal from a ceramic resonator. In the national round, we use Arduino Uno because the rule has limitations but in the international round it is more open so POP-32i is a customized board that is the most suitable because it fast processing and easy to use.
This robot is made almost entirely out of Lego. So, our design process is quite hard.

<br>

## **Choosing parts of our robot:**

<br>

- **Driving Motor** ( Power Functions L-Motor )

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/090f0240-b039-4335-a869-1769695a7ace"  width="400"/>  <img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/b6fe03de-5aed-4876-8065-6b62df5169cd" width="250"/><br>
<br>
| Specification        | Description                  |
|----------------------|------------------------|
| Category             | LEGO Parts             |
| Subcategory          | Electrical              |
| Theme                | LEGO Power Functions  |
| Brand                | LEGO                   |
| Color                | Light Bluish Gray      |
| Released in          | 2012                   |
| Weight               | 45 grams               |
| Dimensions (LxWxH)  | 7 x 4 x 3 studs        |



This motor is small yet powerful. and it is the perfect size for our robot.
The motor comes with a dedicated port for Lego. So, we modified it to make it able to connect to the board.

<br>

- **Ultrasonic Sensor** ( Gravity: URM09 Ultrasonic Distance ) to measure the distance between the robot and the walls

<img src="https://camo.githubusercontent.com/16c1386fb545f593a4291ff12f99c2c1107c30e7dbdb3e14bd4e977f374d7ae9/68747470733a2f2f6466696d672e6466726f626f742e636f6d2f73746f72652f646174612f53454e303330372f53454e303330372e6a70673f696d61676556696577322f312f772f3536342f682f333736" width="300"/>

| Specification          | Description            |
|------------------------|------------------------|
| Supply voltage         | 3.3 V to 5.5 V         |
| Power consumption      | 20 mA                  |
| Measurement range     | from 2 cm to 500 cm    |
| Resolution             | 1 cm                   |
| Accuracy               | 1%                     |
| Frequency              | max. 30 Hz             |
| Operating temperature  | -10°C to 70°C          |
| Dimensions             | 47 x 22 mm             |
| Distance calculation   | Distance = Vout (mV) x 520 / Vin (mV) |


DFRobot URM09 is an ultrasonic sensor designed explicitly for fast-ranging and obstacle-avoidance applications.
This Ultrasonic can measure the distance between 2-500 centimeters precisely which is very important in this mission.

<br>

- **Servo** ( GEEKSERVO-360 ) one for **steering** our robot and another for rotating **ultrasonic sensor**

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/13a08c56-4dc1-482b-bff6-812d6c08bf86" width="300"/>

| Specification          | Description            |
|------------------------|------------------------|
| Temperature Range      | -20 ℃ to 60 ℃         |
| Operating Temperature  | -10 ℃ to 50 ℃         |
| Electrical Voltage     | 4.8V ~ 6V (Servo) / 3V (Motor) |


We use Geekservo 2kg 360 Degrees for steering the robot and employ an Ultrasonic Sensor for rotation. This servo is compatible with LEGO, making it easy and convenient to build the robot.<br>
<br>





<br>

- **Color Sensor** - We use **Virus-III** Designed by Sopon and **ZX-03** Designed by INEX to detect the color of the field.We use the Red ZX-03 Reflector to check the blue line and blue Virus-III to check red to detect color values on the field.

<img src="https://media.discordapp.net/attachments/998531635219796011/1143546792269586555/IMG_4607.jpg?width=286&height=405" width="300"/>

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/059d9221-6a78-4121-a1ab-30730911ec04" width="300"/><br>

<br>

- **Gyro** - We use **GY-25** gyro to control the direction of our robot.

<img src="https://inwfile.com/s-o/q6moq1.jpg" width="300"/><br>

| Specification            | Description                        |
|--------------------------|------------------------------------|
| Chip                     | MPU-6050                            |
| Electrical Voltage       | 3 - 5V                             |
| Communication Modes      | Serial Communication (9600, 115200 Baud), I2C Communication  |
| Dimensions               | 15.5mm x 11.5mm                   |
| Pin Spacing              | 2.54mm                             |
| Direct Data Output       | Raw Data                           |
| Pitch Angle (Yaw)       | ± 180°                            |
| Roll Angle (Roll)       | ± 180°                            |
| Yaw Angle (Pitch)       | ± 180°                            |
| Angular Resolution      | 0.01°                             |


Dataset: http://mkpochtoi.ru/GY25_MANUAL_EN.pdf<br>
Library: https://github.com/ElectronicCats/mpu6050

<br>

- **Camera** - We use **Pixy 2.1** for our robot because it's very easy to use and has a free library and application to configure it.
<img src="https://www.zagrosrobotics.com/images/pixy2_3.jpg"  width="300"/>

| Specification                       | Description                                                                                                       |
|-------------------------------------|-------------------------------------------------------------------------------------------------------------------|
| Field of View (Horizontal)          | Wider field-of-view: 80 degrees (compared to the previous version's 60 degrees)                                |
| Advantages of Wider FOV             | - Ability to see more of the environment per image                                                              |
|                                     | - Enhanced ability to capture a broader area                                                                   |
| Lens Characteristics                 | - The new lens may introduce some spherical distortion due to the wider field-of-view                           |


<br>

- **Board** -  We use **POP-32i** that is coustomize board <br>

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/8bdcfd01-aafb-47bf-b655-e20e0f350442"  width="300"/><br>

| Specification              | Description                                                      |
|----------------------------|------------------------------------------------------------------|
| Microcontroller            | STM32F103CBT6 (32-bit)                                           |
| Flash Memory Capacity      | 128KB                                                            |
| Flash Memory Programming    | Capable of programming up to 10,000


<br>
<br>

- **Microcontroller** - **STM32F103CBT6**, the Processor unit for our robot
  
<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/92f60e15-7905-49d8-8c64-3f621613e2cd" width="300"/>  <img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/04714e86-b5a6-488f-b91f-4743bc127dfb" width="300"/> 

| Specification               | Description                         |
|-----------------------------|-------------------------------------|
| Manufacturer                | STMicroelectronics                  |
| Product Category            | ARM Microcontrollers - MCU          |
| Series                      | STM32F103CB                         |
| Mounting Style              | SMD/SMT                             |
| Package / Case              | LQFP-48                             |
| Core                        | ARM Cortex M3                       |
| Program Memory Size         | 128 kB                              |
| Data Bus Width              | 32 bit                              |
| ADC Resolution              | 12 bit                              |
| Maximum Clock Frequency     | 72 MHz                              |
| Number of I/Os              | 37 I/O                              |
| Data RAM Size               | 20 kB                               |
| Supply Voltage - Min        | 2 V                                |
| Supply Voltage - Max        | 3.6 V                              |
| Minimum Operating Temperature| -40°C                               |
| Maximum Operating Temperature| +85°C                               |
| Packaging                   | Tray                                |
| Brand                       | STMicroelectronics                  |
| Data RAM Type               | SRAM                                |
| Height                      | 1.4 mm                             |
| Interface Type              | CAN, I2C, SPI, USART, USB           |
| Length                      | 7 mm                                |
| Number of ADC Channels      | 10 Channel                          |
| Number of Timers/Counters   | 4 Timer                            |
| Processor Series            | ARM Cortex M                        |
| Product                     | MCUs                                |
| Product Type                | ARM Microcontrollers - MCU          |
| Program Memory Type         | Flash                               |
| Subcategory                 | Microcontrollers - MCU               |
| Tradename                   | STM32                               |
| Width                       | 7 mm                                |
| Unit Weight                 | 0.006409 oz                         |

<br>

- **ZX-Switch01** - We use **ZX-Switch01** to make it easy to start the program.

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/3e788057-3528-42fd-94f2-0f2d0362f62c" width="300"/><br>

Additional Details:<br>
When the switch is pressed, the DATA pin will be in logic "1" from R2, which is connected to a pull-up. When the switch is pressed, the DATA pin will be "0" because the switch creates a short circuit to ground. Current flows through the LED and R1, causing the LED to light up brightly. Additionally, the DATA pin can also serve as an input, allowing you to control the LED's on and off state as needed.

<br>

- **Step Down** - We use **Step Down** to display the battery voltage and to make it deliver the power at the set level.

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/945e54df-e223-417e-bf35-1c05d9597dd6" width="300"/><br>

| Specification              | Description              |
|----------------------------|--------------------------|
| Input Voltage              | 4-38 VDC                 |
| Output Voltage             | 1.25-36 VDC              |
| Output Current Maximum     | 5A                       |
| Chipset                    | XL4015E1                 |
| Dimension (ขนาด)         | 39mm x 66mm x 18mm (WxLxH) |


<br>

- **Helicox 1100mAh LiPo battery (7.4V)** - We use a **Helicox 1100mAh LiPo battery** for the power.

<img src="https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/21fde8ad-0665-471b-94e2-3828cc323b69" width="300"/><br>

| Specification           | Description                      |
|-------------------------|----------------------------------|
| Voltage                 | 7.4V                             |
| Capacity                | 1100mAh 30C                      |
| Charge Current          | Up to 5 times the capacity (5C)  |
| Connector Type          | JST (easily detachable)          |
| Dimensions              | 32x70x12 mm                      |
| Weight                  | 60 grams                         |



<br>
<br>


**Wire connections for our robot**
```c
//  Gyro
//  Compass Variables
//  Connect to YX RX
float pvYaw, pvRoll, pvPitch;
uint8_t rxCnt = 0, rxBuf[8];

//  Motor connect to port 4
//  motor(4, speed);

//  Servos
int const STEER_SRV = 5;
int const ULTRA_SRV = 6;

//  Ultrasonic Sensor
int const ULTRA_PIN = 2;

//  Light Sensors
int const RED_SEN = 0;
int const BLUE_SEN = 1;

//  Button
int const BUTTON = 3;

// Pixy2.1 camera
// Connect to I2C
Pixy2I2C pixy;
```

![](https://lh3.googleusercontent.com/fife/AAbDypBMpb49LbKQ5uKGLfBDmphj8Q8arFVbUlOLNF8VVuXZUzkWvl3eadt5nIXetApAGM7xuwYlJYdm-KfruJEWDW08ieDruFHPaADXrP7M5XFcIfTFgCtY1fjK248FAwB_7itTqeUqzz8pINb41MQSfPzSzkifoEEMkDSyi1I-xJGw0YoBDbfG9zFeddrwtirY0IhSuuieEyrTlboi465iIsCqC__VBJN0-RV1PZkTUowbddHRlyaFzzxbg7O-Jov3FQW7jEgIFs0O2N1yQPx4CO8iDaT0TOcZ1_F7_eemipeXO4S4E6m0oJfD1PPAM21phtAkt1eKt8yExGOCzG3ldnAfriJ87OFrO_T0MO6mjr4OHHut8QX6QlA9YFfd1lUWe9POO6wRtRw-sgNifQiTRhEBlmrgXWLpPbZ0hRhcZYgA7tPZLdf0NT4UTS3-fbQSNGOIUq_ymm8Gx5ijanCEv4of1LiZajwnGiM-FbypDGq1SEkXtfCRteScp8EkKMZRXxhc3aLF2WPfy5UFPwGRDMAdBydtPxS6vD59GdavwcT2wM0_KaMOVXi_bt4DuZG5uf1BWUjaZsJMuJi8ep2sVsNUydBfVH7GA_FGagEzsXjmVWXOPzN4mLbx8haLA8274SpNiXhLMwLUam78VUcSefPEwddz6qgcSfW9c38OUQM03DozOGG3lE8nlZVFuu34VbXgZ8JWC1oPFVYxpYdMX1uQuENEolRZLeBf9sLs4i6UjiRAc-Btg6yNIvgf6CE30h-MIxXZP_CzLSxrODLrGQ3JfhYhH6h9DbrDEqD1pzlYd6HwN8QLn2BRTaJ3SNkk-Tw5X4YlSjZ-BdXg5h5t8j5lQq5Bku3wTZZ7ko15zrIhn_KO2Khng5n6a3ZsXnzQ6rLfF4_KyWNc7id1gsK8KbAWzRpw2AFXCg7J0mUvOmT2wbMUgJLRstg56BeaUlR1xZEQj-dMW8avdAiY0MJnnNpAz7QgXOLF-1UgCnmjKFx5rBme2_4KOm02dMK3lPYj1iV_REC5tAvsLiW4PKRkK6pgRKA_UWmDZiGZTorhgJcVrHkbaeyVv7oI31QaOPY4ULHsYju2wQRvFyWVeWqs5aMI0TRtCV72LUWD1kFNwv89BkGQWG-RRzy5oaxuqUjjrEll-fXJ9713vTQudQbFeSJgInNSrL6URnIIbswHI6UHWtb6ngPBqpmc_I5vtKsSB0Nz8LublM7z5L1st0ipVBGiEdzOLmtNY2P5TjpzVuQp1BeVMTFz9hoh4Mw7k2N9WQ6MN5EnKXEihFJDayxoqXcQYlAfnT-nycc0X8_VMKgcp_BbtZDKjNSZLS-3P9kW6iJQZ-svLY-Kgk1fj6WakgY1gD_dJBEikCEsC6Xg3xRZ0m75TbM-XgiBCYJun125R1O8SiooYNDEDzBQkEclHLUtWslDrk6fyHvGTah7XUi6VBy-ZvxaCCGKSGZ5y7_hiDPAFMNbYElfjDyPHmzphBHIXzpD17WWDNppPShQ0lq32baZH7wJpz7RYXy8GUUJY3-pWqFe1nv2C6-V-6pPQYzPuNusvADoTesBf1jDZtpq0cXsnKuOVTBUcc7a3pai9Sk5pePf14ab34IXVvzUipNU9P8xd653gCYYeh9qnW9DJgX8KiSAKvAuFn17jcAXdeDG=w1920-h929)  


# **Part 2: Programming**
 
Please note that this program is specially written for INEX's POP32 board using the Arduino IDE program to command it. If you want to recreate it. You may need to change the method of writing the program by yourself.

<br>

### **Required Programs**

- **[Arduino IDE 2.1.1]** - For Programming your robot
- **[Pixymon V2]** - For displaying and setting up your Pixy

<br>

### **Required Libraries**

To be able to use the sensors, many libraries need to be downloaded and included in this project. The libraries that will need to be downloaded will depend on the sensors you use. Most of these libraries are important for completing 
both rounds of the competition.

All the libraries used can be included in the program using the **Library Manager** in **Arduino IDE.** except for the Pixy library which you need to download separately from [this website.](https://pixycam.com/downloads-pixy2/)

```
#include "Mapf.h"

#include <Servo.h>

#include <PID_v2.h>

#include <POP32.h>

#include <Pixy2I2C.h>
```

## Setting up() 

Finally, we got to the exciting part 
Please, download our Arduino program, and use it as a reference, so you can further understand our method.

<br>

### Setup()

For the ```setup()```, we have to include these libraries and set the variable of the sensors in order for the robot to work as wanted.

```c
#include "Mapf.h"
#include <Servo.h>
#include <PID_v2.h>
#include <POP32.h>
#include <Pixy2I2C.h>

Pixy2I2C pixy;

long pixy_timer;

Servo servo1;
Servo servo2;

//  Compass Variables
float pvYaw, pvRoll, pvPitch;
uint8_t rxCnt = 0, rxBuf[8];

//  Motor connect to port 4
//  motor(4, speed);

//  Servos
int const STEER_SRV = 5;
int const ULTRA_SRV = 6;

//  Ultrasonic Sensor
int const ULTRA_PIN = 2;

//  Light Sensors
int const RED_SEN = 0;
int const BLUE_SEN = 1;

//  Button
int const BUTTON = 3;

//  Field Config
char TURN = 'U';
int compass_offset = 0;
long halt_detect_line_timer;
bool found_block = false;
int Servo_Value;
int SteerServo_Value;
int motor_steer;
int count = 0;

//  Field Config
char Blocks_TURN = 'U';
float avoidance_degree = 0;
long timer_block_decay;
float found_block_factor;
float bug_degree = 0;
char last_block = 'U';
char before_last_block = 'U';
bool checked_before_last_block = false;
bool checked_last_block = false;
float LastBlock = 20;

// Specify the links and initial tuning parameters
PID_v2 compassPID(0.575, 0, 0.027, PID::Direct); //0.315

int x = 1;
float Y = 35;
int z = 1;
int f;

void setup() {
  compassPID.Start(0, 0, 0);
  compassPID.SetOutputLimits(-180, 180);
  compassPID.SetSampleTime(10);
  pinMode(STEER_SRV, OUTPUT);
  pinMode(ULTRA_SRV, OUTPUT);
  pinMode(ULTRA_PIN, INPUT);
  pinMode(RED_SEN, INPUT);
  pinMode(BLUE_SEN, INPUT);
  pinMode(BUTTON, INPUT);
  Serial.begin(115200);
  pixy.init();
  steering_servo(0);
  ultra_servo(0, 'L');
  // check_leds();
  while (analogRead(3) > 500)
    ;
  zeroYaw();
  while (analogRead(3) <= 500)
    ;
}

```


<br><br>

## First Round Program explanation

The program can be simplified to the following parts:

<br>

###  Steering of the Robot - Turns the servo used for controlling the robot's driving direction to a specified degree.

Variables that affect the steering include:

- The input degree [ The direction we want to robot to steer ]
- The direction of the robot measured by **compass sensor** [ The direction that the robot is now facing ]
- The distance between the wall and the robot measured by **the ultrasonic sensor** [ Preventing the robot from hitting the wall ]

```c
void loop() {
  //(❁´◡`❁);
  motor(4,50);
  delay(400);
  while (analogRead(BUTTON) > 500) {
    getTaco();
    line_detection();
    int wall_distance = getDistance();
    motor_and_steer((1 * x) * compassPID.Run((x * pvYaw) + ((wall_distance - Y)) * ((float(TURN == 'TURN') - 0.5) * 2)));
    ultra_servo(-pvYaw,TURN);
    if (count >= 12) {
      long timer01 = millis();
      while (millis() - timer01 < 650) {
        getTaco();
        line_detection();
        motor_and_steer((1 * x) * compassPID.Run((x * pvYaw) + ((wall_distance - Y)) * ((float(TURN == 'TURN') - 0.5) * 2)));
        ultra_servo(-pvYaw,TURN);
      }
      motor(4,0);
      while (true) {
      }
    }
  }
    motor(4, 0);
  while (analogRead(BUTTON) <= 500)
    ;
  while (analogRead(BUTTON) > 500)
    ;
  while (analogRead(BUTTON) <= 500)
    ;
}
```

<br>

###  The Speed of the Driving Motor

Variables that affect the speed of the motor include:

- The direction of the robot is measured by **compass sensor** [ If the robot is facing in the wrong direction, decrease the speed (to give it enough time to process) ]

```c
void motor_and_steer(int degree) {
  degree = max(min(degree, 45), -45);
  steering_servo(degree);
  motor(4, (map(abs(degree), 0, 45, 50, 45)));
}
```

<br>

###  🚗 Steer the robot to the left or right when a line is detected - There are 2 lines on each corner of the mission field(one red, one blue). This program detects those lines and tells the robot to steer in the correct direction. 

Variables that affect the turning of the robot include:
- The Reflected light values measured by the **VirusIII and ZX-03**

```c
void line_detection() {
  found_block_factor = min(max(mapf(millis() - timer_block_decay, 0, 1000 , 1, 0), 0), 1);
  int blue_value = analogRead(BLUE_SEN);
  if (TURN == 'U') {
    int red_value = analogRead(RED_SEN);
    if (blue_value < 1600 || red_value < 1200) {
      int lowest_red_sen = red_value;
      long timer_line = millis();
      while (millis() - timer_line < 100) {
        int red_value = analogRead(RED_SEN);
        if (red_value < lowest_red_sen) {
          lowest_red_sen = red_value;
        }
      }
      if (lowest_red_sen > 1200) {
        // Red
        TURN = 'L';
        Blocks_TURN = 'L';
        compass_offset += 90;
        x = 1;
        Y = 40;
      } else {
        // Blue
        TURN = 'R';
        Blocks_TURN = 'R';
        compass_offset -= 90;
        x = -1;
        Y = 35;
      }
      halt_detect_line_timer = millis();
      count++;
    }
  } else {
    if (f == 1 && count == 10) {
      z = -1;
    }
    if (millis() - halt_detect_line_timer > 1525) {
      if (blue_value < 1600) {
        if (TURN == 'R') {
          Blocks_TURN = 'R';
          compass_offset -= 90;
        } else {
          Blocks_TURN = 'L';
          compass_offset += 90;
        }
        halt_detect_line_timer = millis();
        count++;
      }
    }
  }
}
```
<br>

**This program ends after turning at the corners of the field 12 times (3 rounds).**

**In case that the "LAST BLOCK" of the second round is red block make a U-turn.**

<br><br>

## last Round Program explaination

The program can be simplified to the following parts:

<br>

### Steering of the Robot - Turns the geekservo used for steering the robot to a specified degree.

Variables that affect the steering incudes:

- The color and position of the color blocks detected by **Pixy 2.1** [ Adjusts the vector of the robot to avoid hitting the detected obstacle on the field ]
- The input degree [ The direction we want to robot to steer to ]
- The direction of the robot measured by **compass sensor** [ The direction that the robot is now facing ]
- The distance between the wall and the robot measured by **the gravity ultrasonic sensor** [ Preventing the robot from hitting the wall]


```c
void loop() {
  //(❁´◡`❁);
  long countdown_stop = millis();
  while (analogRead(BUTTON) > 500) {

    getTaco();
    line_detection();
    ultra_servo(-pvYaw, Blocks_TURN);
    float distance_wall = getDistance();
    float steering_degree = ((1 * x * z) * compassPID.Run((x * z * pvYaw) + ((distance_wall - Y)) * ((float(Blocks_TURN == 'TURN') - 0.5) * 2)));
    if (millis() - pixy_timer > 50) {
      avoidance_degree = calculate_avoidance();
      pixy_timer = millis();
    }
    int final_degree = map(max(found_block, found_block_factor), 1, 0, mapf(min(max(distance_wall, 5), 30), 5, 30, steering_degree, -1.62 * avoidance_degree), steering_degree);
    while (count == 1) {
      if ((millis() - halt_detect_line_timer > 10 && millis() - halt_detect_line_timer < 300) && count == 1 && !checked_before_last_block) {
        if (before_last_block == 'R') {
          LastBlock = LastBlock * 1;
        } else if (before_last_block == 'L') {
          LastBlock = LastBlock * 1;
        } else {
          LastBlock = LastBlock * 1;
        }
        checked_before_last_block = true;
      }
      if ((millis() - halt_detect_line_timer > 1500 && millis() - halt_detect_line_timer < 1700) && count == 1 && !checked_last_block) {
        if (last_block == 'R') {
          f = 1;
          steering_servo(avoidance_degree * LastBlock);
          motor(4, 50);
          delay(600);
          if (TURN == 'L') {
            TURN = 'R';
            compass_offset -= 180;
          } else {
            TURN = 'L';
            compass_offset += 180;
          }

          checked_last_block = true;
          count += 2;
          break;
        }
      } else {
        break;
      }
    }  
    // Stops everything
    if (millis() - countdown_stop > 1600) {
      motor(4, 0);
      while (true)
        ;
    }
    if (count < 12) {
      countdown_stop = millis();
    }
    motor_and_steer(final_degree);
  }
  motor(4, 0);
  while (analogRead(BUTTON) <= 500)
    ;
  while (analogRead(BUTTON) > 500)
    ;
  while (analogRead(BUTTON) <= 500)
    ;
}
```
<br>

The calculating of degree in which the robot needs to turn to avoid the color blocks, the program can be simplified as follows:

- Calculate the distance between the camera and the block using this following formula:
> distance = size_obj * focal_length / size_obj_on_sensor

<br>
After calculating the distace, we need to find the position of the block using **Trigonometry**.

```c
pixy.ccc.getBlocks();
...

float(atan2(1000 * 2 / pixy.ccc.blocks[nearest_index].m_height * tan(float(map(pixy.ccc.blocks[nearest_index].m_x, 0, 316, -30, 30) + Wrap(bearingPID - initial_deg, -180, 179)) / 180 * PI) - 20, 1000 * 2 / pixy.ccc.blocks[nearest_index].m_height - 10)) * 180 / PI
```

> atan2!? 
> It is the 2-argument arctangent. By definition, degree (radians) = atan2(y,x)

> Find out more about atan2: [atan2 - Wikipedia]

As shown in the program above, we can calculate the driving direction degree using the **atan2 function**. Setting the **y coordinate** by measuring the position of the color blocks in 3D space using **the Pixy camera** then **subtract or add 20 cm** to **offset the y coordinate**, so that the robot will not hit the detected obstacle. The same for the **x coordinate**, and then we get the final steering degree! 

<br>

###  Changing the Speed of the Driving Motor - transmission of data with L298n to drive the motor at a specified speed with a specified acceleration.

Variables that affect the speed of the motor includes:

- The direction of the robot measured by **compass sensor** [ If the robot is facing in the wrong direction, decelerate the speed ]

**Uses the same code previously mentioned**

<br>
### 🚗 Steer the robot to left or right when a line is detected - There are 2 lines on each corner of the mission field(one red, one blue). This program detects those lines and tell the robot to steer to the correct direction.  

Variables that affect the turning of the robot includes:
- The number of color blocks detected by **the pixy camera** [ To make sure the robot turns at the corner of the field after avoiding the obstacles on the field ]
- The Reflected light values measured by the **Virus IIISensors**


**Uses mostly the same code previously mentioned**

<br>



**This program ends after turning at the corners of the mission field 12 times (3 rotation).**

<br>


# **THE END**
For further questions please email pawit.book@gmail.com

   [Yothinburana School Robot Club]: <https://ybrobot.club/>
   [Original Flipped Digital Lab]: <https://ofdl.tw/en/>
   [EV3FW-V1.21C-OFDL.bin]: <https://github.com/a10036gt/EV3-BatteryInfo-Block/releases/download/v1.2/EV3FW-V1.21C-OFDL.bin>
   [OFDL-EV3_Blocks-Collections]: <https://github.com/a10036gt/OFDL-EV3_Blocks-Collections/releases/tag/2020.09.12>
   [Pixy Blocks and Examples]: <https://github.com/charmedlabs/pixy/raw/master/releases/lego/lego_pixy-1.1.4.zip>
   [EV3 Education Lab]: <https://e498eb58-16e9-491c-8ce4-828510ab7c41.filesusr.com/archives/1f66bb_4708cf7510f64585bd447c26a4110fc5.zip?dn=LME-EV3_Full-setup_1.4.5_en-US_WIN32.zip>
   [Pixymon V2]: <https://github.com/charmedlabs/pixy/raw/master/releases/pixymon_windows/pixymon_windows-2.0.9.exe>
   [Wikipedia - Bezier Curve]: <https://en.wikipedia.org/wiki/B%C3%A9zier_curve>
   [Arduino-pico Github]: <https://github.com/earlephilhower/arduino-pico>
   [Arduino IDE 2.1.1]: <https://www.arduino.cc/en/software>
   [atan2 - Wikipedia]: <https://en.wikipedia.org/wiki/Atan2>
