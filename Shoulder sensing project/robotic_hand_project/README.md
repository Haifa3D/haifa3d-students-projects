# Sensing System For Robotic Hand Control

## Overview

In this Undergraduaduate student project we developed a  
system to control the operation of a robotic hand using shoulder movements.  
Presented By:  [Nitay Ozer](mailto:nitay.ozer@campus.technion.ac.il?subject=Robotic\%20Hand\%20Control\%20via\%20Shoulder\%20Movement), [Itay Mal](mailto:itay.mal@campus.technion.ac.il?subject=Robotic\%20Hand\%20Control\%20via\%20Shoulder\%20Movement)  
Supervised By:  [Kobi Kohay](mailto:kohai@ee.technion.ac.il?subject=Robotic\%20Hand\%20Control\%20via\%20Shoulder\%20Movement)

## Pre-requisits

### Hardware

* ESP32 - we used [Sparkfun ESP32 Thing Plus](https://www.sparkfun.com/products/17381)
* Inertial Measurement Unit (IMU) - we used [GY-521](http://hiletgo.com/ProductDetail/2157948.html)
* Piezzo electric buzzer
* 3.7V Lithium-Polymer (LiPo) Battery
* [Prototype board](https://www.sparkfun.com/products/12070)
* 3D printed case [see stl files](/box_3d/)

### Software

* Arduino IDE with ESP32 Support - we used [VSCode](https://code.visualstudio.com/) with [Arduino Extention](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)
* Arduino Libraries:
  * Native Arduino libraries
  * Adafruit_MPU6050
  * Adafruit_Sensor
* Python3 and relevant packeges

## ESP32 Circuit Connection

![connection scheme](/readme_src/esp_imu_connection.PNG)

## Code Parts

### [Trainig Data Collection](/code/on_cpu/)

### [Classifier Training](/code/ML/)

### [Inference On uController](/code/on_esp32_ble/)

## Supporters 🙏

Many thanks to the supporters and advisors of this project!  
[![Haifa 3D](/readme_src/Haifa_3D_logo.png)](https://github.com/Haifa3D)   [![Technion](/readme_src/TechnionLogo.png)](https://www.technion.ac.il/en/home-2/)    [![CRML Lab](/readme_src/CRML-logo.jpg)](https://crml.eelabs.technion.ac.il/)
