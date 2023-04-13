# ADXL355
Arduino Library for Analog Devices - ADXL355

This library has been tested on:
* RaspberryPi Pico
* ESP32-S3 Module

Currently available bus:
* SPI 

Feel free to colaborate!!

```
#include "ADXL355_.h"

#define _spi SPI
#define spi_speed     8000000
#define ADXL_CS_PIN   4              // custom CS pin
#define ADXL_INT1_PIN 14             // custom INT1 pin
#define ADXL_INT2_PIN 10             // custom INT2 pin
#define ADXL_DRDY_PIN 8              // custom DRDY pin
#define ADXL_SYNC_PIN ADXL_DRDY_PIN  // custom SYNC pin

ADXL355 adxl355(&_spi, ADXL_CS_PIN, spi_speed);

void adxl_setup(){
  adxl355.begin();
 
  adxl355.setRange(RANGE_2G);         // set measuring range: RANGE_2G, RANGE_4G or RANGE_8G
  adxl355.setINT1_pin(ADXL_INT1_PIN);
  //adxl355.setDRDY_pin(ADXL_DRDY_PIN);
  adxl355.setFilter(NOT_ENABLED, ODR_62_5); // set High Pass Filter and Low Pass Filter
  adxl355.setInterruptPol(HIGH);      // set interrupt polarity HIGH 
  
  adxl355.DRDY_Off();
  adxl355.setOffset_mg(0,0,-1000);    // Set Z axis offset to 1g
  adxl355.setThresh_mg(300);          // Activity detection threshold
  adxl355.setThreshCount(10);         // Activity counter: if more than 10 consecutive measures are over 300mg, activity is detected.
  adxl355.setActivity(0,0,0);         // Enable activity detection on X, Y and/or Z axis.
  adxl355.setInterrupt(0b00000001);   // 0b00000001 RDY_EN1: if data ready to read INT1 pin is HIGH
  //adxl355.setInterrupt(0b00001000); // 0b00001000 ACT_EN1: if activity detected INT1 pin is HIGH 
  
  */  EXTERNAL SYNCRONIZATION   /*
  //adxl355.setSYNC_pin(ADXL_DRDY_PIN);
  adxl355.setExtCLK(EXT_CLK_OFF);             // EXT_CLK_OFF, EXT_CLK_ON
  adxl355.setExtSync(INT_SYNC);   // INT_SYNC, EXT_SYNC_INTERP_OFF, EXT_SYNC_INTERP_ON

  adxl355.printDetails();
}

void setup() {
  Serial.begin(115200);
  adxl_setup();
}

void loop() {
  if ((adxl355.isINT1())){
      adxl355.get3VRaw(data);      
      ADXL355_print3V(data);
    }
}

void ADXL355_print3V(int32_t *data){
  Serial.print(data[0]); Serial.print("\t");
  Serial.print(data[1]); Serial.print("\t");
  Serial.print(data[2]); Serial.print("\n");
}


```
