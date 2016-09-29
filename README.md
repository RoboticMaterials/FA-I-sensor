# FA-I / Proximity Sensor
Sketch for interfacing Robotic Materials' FA-I tactile sensor and sensing arrays. You will need a copy of the Arduino IDE and
any compatible Arduino device that supports I2C, 3.3V and 5V. If you have ordered the evaluation kit with a Teensy LC, you will need to install Teensyduino from https://www.pjrc.com/teensy/teensyduino.html.

Connect the Arduino/Teensy to your serial port and launch the serial plotter from the Arduino IDE. You should see something like this:

<img src="https://github.com/RoboticMaterials/FA-I-sensor/blob/master/screenshot_serialplotter.png" width=500>

The blue signal is the raw proximity/force reading. After touch, it roughly corresponds to the human SA-I signal, that is slow adapting pressure. You would not notice anything if it slowly changes, but only if the value exceeds a certain threshold. The yellow signal roughly corresponds to the human FA-I signal, that is fast adapting pressure. It helps you notice changes such as the lightest contact, but keeps quiet for constant pressure. Thinking about wearing clothes is a good analogy: your FA-I sensors alert you when you wear them and your SA-I sensors ignore them, but notice of you larger changes. The third signal is the result of a touch detector that analyses the FA-I signal and its derivative. The picture shows two touch and release events that correspond to minima and maxima in the FA-I signal, respectively. These minima and maxima can be detected in the code, which outputs a "T" for a touch signal and a "R" for a release event.

Switch to "Serial Monitor Mode". You will see three comma-separated values like those

38859,-29261,0<br>
40856,-22480,T<br>
41211,-16091,0<br>
41479,-11532,0<br>
41454,-8048,0<br>
41195,-5375,0<br>
39526,-2094,0<br>
23951,14109,0<br>
12157,21670,0<br>
9185,18141,R<br>
8438,13445,0<br>
8213,9636,0<br>
8121,6837,0<br>
8081,4825,0<br>
8037,3421,0<br>

The first value is the raw sensor reading that corresponds to proximity or force before or after contact, respectively. The second value is the FA-II value, and the third column indicates a detected touch ("T") or release ("R") event. 

You can enter the following commands into the serial terminal:

h: A list of possible commands<br>
c: Toggle continuous mode<br>
s: Single-shot measurement<br>
t: Toggle touch/release analysis<br>





