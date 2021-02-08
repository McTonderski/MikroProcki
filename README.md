## Metronome  

Due to damage to one of the parts required to create the project we needed to modify our topic.

Metronome is a device for accurately conveying the tempo of a piece of music.

----
### **Team members** 
- Maciej Tonderski
- Magdalena Nowosiad


----
### **Purpose** 

The purpose of this project is to produce the device that can be used during musical exercises and composing.



 ----
 ### **Success criteria** 

Writing a program which will by recognizing the position of the device, distinguishing between single and double tapping, tapping the uploaded rhythm.


 ----
 ### **Desired Result** 

Desired outcome of this project would be a device which lets user set the pace of the tapping to help keeping the tempo while playing the instruments or singing.
Additionally playing programmed rhythm e.g. from Rubik's repertoire while turning the board over which will also reset previously set pace.


 ----
 ### **Technical specification** 
 * Code is present at master branch
 * C in the STM32Cube env 
 * MAKEFILE toolchain
 * Board:
   * STM32F411RE
 * Components:
   * LCD1602 
   * ADXL345 -> 3 axis accelerometer 
 * Libs used:
  * HAL lib
  
 ---
 ### **Functional description** 
 
Acceleration sensor turned upside down will cause board to play preprogrammed "Rubik's" rythm. 
When turned back up, board is armed, that means it can record taps, and play them back.
Board after detecting time delay between taps, act as metronome with recorded tempo.

In order to restart board and record different tempo user needs to turn board back wait for Rubik's rythm to play and turn board back up.



