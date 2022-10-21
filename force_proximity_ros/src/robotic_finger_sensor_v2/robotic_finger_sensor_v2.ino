/*
  This is a sample publishing sensor data of Robotic Finger Sensor v2 to ROS topic
  Robotic Finger Sensor v2: https://www.sparkfun.com/products/14687
  This sample is based on https://github.com/sparkfunX/Robotic_Finger_Sensor/blob/df733d36cc5239a0b1cce9eb1aaf39e2d0913bdc/Firmware/Example1_BasicReadings/Example1_BasicReadings.ino
  and https://github.com/RoboticMaterials/FA-I-sensor/blob/ca3e80b00429e3f17e96f564599a26a89222fe3a/force_proximity_ros/src/proximity_sensor_vcnl4040/proximity_sensor_vcnl4040.ino
*/

#include <Wire.h>
#include <ros.h>
#include <force_proximity_ros/ProximityStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include "SparkFun_VCNL4040_Arduino_Library.h"  // Search and install SparkFun_VCNL4040 on library manager of Arduino IDE
// https://github.com/sparkfun/SparkFun_VCNL4040_Arduino_Library
#include "SparkFun_LPS25HB_Arduino_Library.h"  // Search and install SparkFun_LPS25HB on library manager of Arduino IDE
// https://github.com/sparkfun/SparkFun_LPS25HB_Arduino_Library

/***** ROS *****/
ros::NodeHandle nh;
force_proximity_ros::ProximityStamped prox_msg;
sensor_msgs::FluidPressure press_msg;
ros::Publisher prox_pub("proximity_sensor", &prox_msg);
ros::Publisher press_pub("pressure_sensor", &press_msg);

/***** USER PARAMETERS *****/
unsigned long time;

/***** GLOBAL CONSTANTS *****/
#define LOOP_TIME 20  // loop duration in ms
// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
VCNL4040 prox_sensor;
LPS25HB press_sensor;
unsigned int proximity_value; // current proximity reading
unsigned int average_value;   // low-pass filtered proximity reading
signed int fa2;               // FA-II value;
signed int fa2derivative;     // Derivative of the FA-II value;
signed int fa2deriv_last;     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(prox_pub);
  nh.advertise(press_pub);
  while(!nh.connected())
  {
    nh.spinOnce();
  }

  Wire.begin();
  Wire.setClock(400000); // Increase I2C bus speed to 400kHz

  // Start proximity sensor
  if (prox_sensor.begin() == false)
  {
    while (1); // Freeze!
  }
  // Make config the same as proximity_sensor_vcnl4040.ino
  prox_sensor.setLEDCurrent(75);  // Set IR LED current to 75mA (set to 200mA in begin())
  prox_sensor.disableSmartPersistance();  // Disable smart persistence (enabled in begin())
  // Such config methods may be too slow to be put in fast loop
  // because they first read current data in register before writing new data.
  // To avoid this, you should fix library in some way or use low-level methods instead of such methods
  delay(10);
  // Initialize variables
  proximity_value = prox_sensor.getProximity();
  average_value = proximity_value;
  fa2 = 0;

  // Start pressure sensor
  press_sensor.begin();
  if(press_sensor.isConnected() == false)  // The library supports some different error codes such as "DISCONNECTED"
  {
    while (1); // Freeze!
  }
}

void loop()
{
  time = millis();

  // Read and publish proximity sensor value
  proximity_value = prox_sensor.getProximity();
  fa2deriv_last = fa2derivative;
  fa2derivative = (signed int) average_value - proximity_value - fa2;
  fa2 = (signed int) average_value - proximity_value;
  prox_msg.proximity.proximity = proximity_value;
  prox_msg.proximity.average = average_value;
  prox_msg.proximity.fa2 = fa2;
  prox_msg.proximity.fa2derivative = fa2derivative;
  if (fa2 < -sensitivity) prox_msg.proximity.mode = "T";
  else if (fa2 > sensitivity) prox_msg.proximity.mode = "R";
  else prox_msg.proximity.mode = "0";
  prox_pub.publish(&prox_msg);
  average_value = EA * proximity_value + (1 - EA) * average_value;  // Do this after proximity message creation

  // Read and publish pressure sensor value
  press_msg.fluid_pressure = press_sensor.getPressure_hPa() * 100.0;
  press_msg.variance = 0;  // variance is unknown
  press_pub.publish(&press_msg);

  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();
}
