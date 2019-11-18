 /*
   This code demonstrates the force response of the RM Pressure Sensor eval kit.

   Press c to turn off continuous mode.
   Press h for help.

   This code requires Teensy Arduino to be installed: https://www.pjrc.com/teensy/td_download.html
   Select Teensy LC from the Board Menu. Load the code onto the Teensy.

   The output will be distance reading and a touch or release character.

   Brought to you by SparkFun (orignial code), the Correll Lab at the University
   of Colorado, Boulder and Robotic Materials Inc.

   This software is open source and can be used for any purpose.
*/

/***** Library parameters ****/
#define WIRE Wire1


#include <i2c_t3.h>     // Use <i2c_t3.h> for Teensy and <Wire.h> for Arduino
#include <math.h>
#include <ros.h>
#include <force_proximity_ros/ProximityStamped.h>


/***** ROS *****/
ros::NodeHandle  nh;
force_proximity_ros::ProximityStamped prx_msg;
ros::Publisher prx_pub("proximity_sensor", &prx_msg);

/***** USER PARAMETERS *****/
int ir_current_ = 8;                     // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 0; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz
unsigned long time;

/***** GLOBAL CONSTANTS *****/
#define VCNL4010_ADDRESS 0x13
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define PROXIMITY_MOD 0x8F  // proximity modulator timing

#define LOOP_TIME 10  // loop duration in ms

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
unsigned int proximity_value; // current proximity reading
unsigned int average_value;   // low-pass filtered proximity reading
signed int fa2;              // FA-II value;
signed int fa2derivative;     // Derivative of the FA-II value;
signed int fa2deriv_last;     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

unsigned long start_time;
int continuous_mode = 1; //Default on
int single_shot = 0;
int touch_analysis = 1; //Default on

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(prx_pub);

  WIRE.begin();
  delay(1000);

  writeByte(AMBIENT_PARAMETER, 0x7F);
  writeByte(IR_CURRENT, ir_current_);
  writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay

  delay(50);

  delay(100);
  proximity_value = readProximity();
  average_value = proximity_value;
  fa2 = 0;
}

void loop()
{
  time = millis();

  // Read sensor values
  proximity_value = readProximity();
  fa2deriv_last = fa2derivative;
  fa2derivative = (signed int) average_value - proximity_value - fa2;
  fa2 = (signed int) average_value - proximity_value;

  prx_msg.proximity.proximity = proximity_value;
  prx_msg.proximity.average = average_value;
  prx_msg.proximity.fa2 = fa2;
  prx_msg.proximity.fa2derivative = fa2derivative;
  if (fa2 < -sensitivity) prx_msg.proximity.mode = "T";
  else if (fa2 > sensitivity) prx_msg.proximity.mode = "R";
  else prx_msg.proximity.mode = "0";

  prx_pub.publish(&prx_msg);

  //Serial.print(proximity_value);
  //Serial.print(",");
  //Serial.print(fa2);
  //Serial.print(",");
  //Serial.print(",");
  //Serial.print(fa2derivative);
  //Serial.print("T");
  //Serial.print("R");

  // Do this last
  average_value = EA * proximity_value + (1 - EA) * average_value;
  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();

}


unsigned int readProximity() {
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure

  while (!(readByte(0x80) & 0x20)); // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

unsigned int readAmbient() {
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while (!(readByte(0x80) & 0x40)); // wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
}

byte writeByte(byte address, byte data)
{
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);
  WIRE.write(data);
  return debug_endTransmission(WIRE.endTransmission());
}


byte readByte(byte address) {
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);

  debug_endTransmission(WIRE.endTransmission());
  WIRE.requestFrom(VCNL4010_ADDRESS, 1);
  while (!WIRE.available());
  byte data = WIRE.read();
  return data;
}

byte debug_endTransmission(int errcode)
{
  if (false)
  {
    switch (errcode)
    {
      // https://www.arduino.cc/en/Reference/WireEndTransmission
      case 0:
        //Serial.println("CAVOK");
        break;
      case 1:
        //Serial.println("data too long to fit in transmit buffer ");
        break;
      case 2:
        //Serial.println("received NACK on transmit of address ");
        break;
      case 3:
        //Serial.println("received NACK on transmit of data");
        break;
      case 4:
        //Serial.println("other error");
        break;
    }
  }
  return errcode;
}

