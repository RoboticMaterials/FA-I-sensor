/*
 * Brought to you by Sparkfun (orignial code), the Correll Lab at the University of Colorado, Boulder
 * and Robotic Materials Inc.
 * 
 * This software is open source and can be used for any purpose. 
 */

#include <Wire.h>

/***** USER PARAMETERS *****/
int ir_current_ = 4; // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
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
#define EA 0.3  // exponential average (cut-off frequency for high-pass filter)
#define IGNORE_TIME 250 // time to ignore events after first touch/release events (in ms)

/***** GLOBAL VARIABLES *****/
unsigned int average_value;
unsigned int proximity_value_;
unsigned long start_time;
unsigned long ignore_time;
char buf[20];
char cmd;

int  continuous_mode=1;
int  single_shot=0;
int  touch_analysis=1;

struct TouchEvent {
  unsigned int ttouch;
  unsigned int trelease;
  unsigned int color;
} touchevent;


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(1000);


  writeByte(AMBIENT_PARAMETER, 0x7F);
  writeByte(IR_CURRENT, ir_current_);
  writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay
    
  delay(50); 

  byte temp = readByte(PRODUCT_ID);
  byte proximityregister = readByte(IR_CURRENT);
  if (temp != 0x21){  // Product ID Should be 0x21 for the 4010 sensor
      Serial.print("IR sensor failed to initialize: id = ");
         Serial.print(". Should have returned 0x21 but returned ");
      Serial.println(temp, HEX);
  }
    
  delay(100);
}

void loop()
{
    time=millis();
    if (Serial.available() > 0) {
                // read the incoming byte:
                cmd = Serial.read();
                switch(cmd){
                 case 's' : single_shot = 1; break;
                 case 'a' : if(touch_analysis==0) touch_analysis=1; else touch_analysis=0; break;
                 case 'c' : if(continuous_mode==0) continuous_mode=1; else continuous_mode=0; break;
                 case 'h' : Serial.println("c: Toggle continuous mode");
                            Serial.println("s: Single-shot measurement"); 
                            break;               
                }

        }
    
   // Read sensor values
    proximity_value_ = readProximity();
    if(continuous_mode || single_shot){
     //sprintf(buf, "%6u,%6d", proximity_value_,average_value-proximity_value_);
     sprintf(buf,"%6u",proximity_value_);
     //sprintf(buf, "%6d",average_value-proximity_value_);
     Serial.print(buf);
     Serial.println();
     single_shot=0;
    }
    if(millis()>ignore_time){
    if(((signed int) (average_value-proximity_value_)) < -1000){ 
      touchevent.ttouch=millis();
      touchevent.color=proximity_value_;
      if(touch_analysis){
       sprintf(buf,"T %d",touchevent.color);
       Serial.println(buf);
      }
      ignore_time=touchevent.ttouch+IGNORE_TIME;
      
    }
      else 
    if(((signed int) (average_value-proximity_value_)) > 1000){ 
      touchevent.trelease=millis();
      if(touch_analysis){
       Serial.println("R");
      }
      ignore_time=touchevent.trelease+IGNORE_TIME;
    }
    }  
    
    average_value=EA*proximity_value_+(1-EA)*average_value;
    while(millis()<time+LOOP_TIME); // enforce constant loop time
}


unsigned int readProximity(){   
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure

  while(!(readByte(0x80) & 0x20));  // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

unsigned int readAmbient(){   
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while(!(readByte(0x80) & 0x40));  // wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
}

byte writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  return debug_endTransmission(Wire.endTransmission());
}

byte readByte(byte address){
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  
  debug_endTransmission(Wire.endTransmission());
  Wire.requestFrom(VCNL4010_ADDRESS, 1);
  while(!Wire.available());
  byte data = Wire.read();
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
      Serial.println("CAVOK");
      break;
    case 1:
      Serial.println("data too long to fit in transmit buffer ");
      break;
    case 2:
      Serial.println("received NACK on transmit of address ");
      break;
    case 3:
      Serial.println("received NACK on transmit of data");
      break;
    case 4:
      Serial.println("other error");
      break;
  }
  }
  return errcode;
}

