 /*
   This code requires Teensy Arduino to be installed: https://www.pjrc.com/teensy/td_download.html
   Select Teensy LC from the Board Menu. Load the code onto the Teensy.

   The output will be distance reading and a touch or release character.

   Brought to you by SparkFun (orignial code), the Correll Lab at the University
   of Colorado, Boulder and Robotic Materials Inc.

   This software is open source and can be used for any purpose.
*/

/***** Library parameters ****/

#define NGRIPPERS 2
#define NSENSORS 8

#include <i2c_t3.h>     // Use <i2c_t3.h> for Teensy and <Wire.h> for Arduino
#include <math.h>
#include <ros.h>
#include <force_proximity_ros/ProximityArray.h>

/********* I2C multiplexer ********

  A2 A1 A0
  L L L 112 (decimal), 70 (hexadecimal)
  L L H 113 (decimal), 71 (hexadecimal)
  L H L 114 (decimal), 72 (hexadecimal)
  L H H 115 (decimal), 73 (hexadecimal)
  H L L 116 (decimal), 74 (hexadecimal)
  H L H 117 (decimal), 75 (hexadecimal)
  H H L 118 (decimal), 76 (hexadecimal)
  H H H 119 (decimal), 77 (hexadecimal)
*/
int i2c_multi_id = 112;


/***** ROS *****/
ros::NodeHandle nh;
force_proximity_ros::Proximity proximities[NGRIPPERS*NSENSORS];
force_proximity_ros::ProximityArray arr_msg;
ros::Publisher arr_pub("proximity_grippers", &arr_msg);

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
i2c_t3* i2c_chans[NGRIPPERS];
unsigned int proximity_value; // current proximity reading
unsigned int average_value[NGRIPPERS * NSENSORS]; // low-pass filtered proximity reading
signed int  fa2[NGRIPPERS * NSENSORS];            // FA-II value;
signed int fa2derivative[NGRIPPERS * NSENSORS];   // Derivative of the FA-II value;
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

unsigned long start_time;
int continuous_mode = 1; //Default on
int single_shot = 0;
int touch_analysis = 1; //Default on

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(arr_pub);
  Wire.begin();
  Wire1.begin();
  delay(1000);

  if (NGRIPPERS) i2c_chans[0] = &Wire1;
  if (NGRIPPERS > 1) i2c_chans[1] = &Wire;
  for (int i = 0; i < NGRIPPERS; i++) {
    i2c_chans[i]->begin();
    //i2c_chans[i]->setDefaultTimeout(1000);
    delay(100);
    for (int j = 0; j < NSENSORS; j++) {
      i2c_chans[i]->beginTransmission(i2c_multi_id);
      i2c_chans[i]->write(1 << j);
      debug_endTransmission(i2c_chans[i]->endTransmission());

      writeByte(i, AMBIENT_PARAMETER, 0x7F);
      writeByte(i, IR_CURRENT, ir_current_);
      writeByte(i, PROXIMITY_MOD, 1); // 1 recommended by Vishay
      delay(50);
      byte temp = readByte(i, PRODUCT_ID);
      //byte proximityregister = readByte(IR_CURRENT);
      if (temp != 0x21) { // Product ID Should be 0x21 for the 4010 sensor
        Serial.print("IR sensor failed to initialize: id = ");
        Serial.print(". Should have returned 0x21 but returned ");
        Serial.println(temp, HEX);
      }
   }
  }
  delay(100);
  for (int i = 0; i < NGRIPPERS; i++) {
    for (int j = 0; j < NSENSORS; j++) {
      //  Watchdog.reset();
      i2c_chans[i]->beginTransmission(i2c_multi_id);
      i2c_chans[i]->write(1 << j);
      debug_endTransmission(i2c_chans[i]->endTransmission());
      average_value[i*NSENSORS+j] = readProximity(i);
      fa2[i*NSENSORS+j] = 0;
    }
  }
}

void loop()
{
  time = millis();
  // Read sensor values
  for (int i = 0; i < NGRIPPERS; i++) {
    for (int j = 0; j < NSENSORS; j++) {

      i2c_chans[i]->beginTransmission(i2c_multi_id);
      i2c_chans[i]->write(1 << j);
      i2c_chans[i]->endTransmission();

      proximity_value = readProximity(i);
      fa2derivative[i*NSENSORS+j] = (signed int) average_value[i*NSENSORS+j] - proximity_value - fa2[i*NSENSORS+j];
      fa2[i*NSENSORS+j] = (signed int) average_value[i*NSENSORS+j] - proximity_value;

      proximities[i*NSENSORS+j].proximity = proximity_value;
      proximities[i*NSENSORS+j].average = average_value[i*NSENSORS+j];
      proximities[i*NSENSORS+j].fa2 = fa2[i*NSENSORS+j];
      proximities[i*NSENSORS+j].fa2derivative = fa2derivative[i*NSENSORS+j];
      if (fa2[i*NSENSORS+j] < -sensitivity) proximities[i*NSENSORS+j].mode = "T";
      else if (fa2[i*NSENSORS+j] > sensitivity) proximities[i*NSENSORS+j].mode = "R";
      else proximities[i*NSENSORS+j].mode = "0";
      average_value[i*NSENSORS+j] = EA * proximity_value + (1 - EA) * average_value[i*NSENSORS+j];
    }
  }

  arr_msg.header.stamp = nh.now();
  arr_msg.proximities = proximities;
  arr_msg.proximities_length = NSENSORS*NGRIPPERS;
  arr_pub.publish(&arr_msg);

  while (millis() < time + LOOP_TIME); // enforce constant loop time
  nh.spinOnce();

}

unsigned int readProximity(int i) {
  byte temp = readByte(i, 0x80);
  writeByte(i, 0x80, temp | 0x08); // command the sensor to perform a proximity measure

  while (!(readByte(i, 0x80) & 0x20)); // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(i, 0x87) << 8;
  data |= readByte(i, 0x88);

  return data;
}

unsigned int readAmbient(int i) {
  byte temp = readByte(i, 0x80);
  writeByte(i, 0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while (!(readByte(i, 0x80) & 0x40)); // wait for the proximity data ready bit to be set
  unsigned int data = readByte(i, 0x85) << 8;
  data |= readByte(i, 0x86);

  return data;
}

byte writeByte(int i, byte address, byte data)
{
  i2c_chans[i]->beginTransmission(VCNL4010_ADDRESS);
  i2c_chans[i]->write(address);
  i2c_chans[i]->write(data);
  return debug_endTransmission(i2c_chans[i]->endTransmission());
}

byte readByte(int i, byte address) {
  i2c_chans[i]->beginTransmission(VCNL4010_ADDRESS);
  i2c_chans[i]->write(address);

  debug_endTransmission(i2c_chans[i]->endTransmission());
  i2c_chans[i]->requestFrom(VCNL4010_ADDRESS, 1);
  while (!i2c_chans[i]->available());
  byte data = i2c_chans[i]->read();
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
        /* Serial.println("CAVOK"); */
        break;
      case 1:
        /* Serial.println("data too long to fit in transmit buffer "); */
        break;
      case 2:
        /* Serial.println("received NACK on transmit of address "); */
        break;
      case 3:
        /* Serial.println("received NACK on transmit of data"); */
        break;
      case 4:
        /* Serial.println("other error"); */
        break;
    }
  }
  return errcode;
}


