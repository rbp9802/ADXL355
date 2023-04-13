#include <Arduino.h>
#include <SPI.h>

#include "ADXL355.h"

ADXL355::ADXL355(uint8_t cs_pin): _cs(cs_pin)
{
  #ifdef ESP32
    SPIClass dev_spi(SPI);
  #elif defined ARDUINO_ARCH_RP2040
    SPIClassRP2040 dev_spi(SPI);
  #endif
  //dev_spi = SPI;
  spi_speed = 5000000;
  //dev_i2c = NULL;
}

ADXL355::ADXL355(SPIClass *spi,uint8_t cs_pin): dev_spi(spi), _cs(cs_pin)
{   
  spi_speed = 5000000;
  //dev_i2c = NULL;
}

ADXL355::ADXL355(SPIClass *spi,uint8_t cs_pin, uint32_t spi_speed): dev_spi(spi), _cs(cs_pin), spi_speed(spi_speed)
{ 
  //dev_i2c = NULL;
}

//ADXL355::ADXL355(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
//{ 
//  dev_spi = NULL;
//}

void ADXL355::setSPI_pins(uint8_t sck, uint8_t miso, uint8_t mosi){
  _sck = sck;
  _miso = miso;
  _mosi = mosi;
}

void ADXL355::begin(void){
  if (dev_spi){
    #ifdef ESP32
      dev_spi->begin(_sck, _miso, _mosi);
    #elif defined ARDUINO_ARCH_RP2040
      dev_spi->begin();
    #else
      dev_spi->begin();
    #endif
    pinMode(_cs, OUTPUT);
    cs(HIGH);
  }
  
  setRange(RANGE_2G);
  DRDY_Off();
  Temp_Off();
  setFilter(HPF_CORNER::NOT_ENABLED, ODR_LPF::ODR_4000);
}

void ADXL355::end(void){
  if (dev_spi){
    dev_spi->end();
    pinMode(_cs, INPUT);
  }
}

void ADXL355::setDRDY_pin(uint8_t DRDY_PIN){
  _drdy = DRDY_PIN;
  DRDY_On();
  pinMode(_drdy, INPUT);
}

void ADXL355::setSYNC_pin(uint8_t SYNC_PIN){
  _drdy = SYNC_PIN;
  DRDY_Off();
  pinMode(_drdy, OUTPUT);
  digitalWrite(_drdy, LOW);
}

void ADXL355::syncPulse(){
  digitalWrite(_drdy, HIGH);
  delayMicroseconds(5);
  digitalWrite(_drdy, LOW);
}

void ADXL355::setINT1_pin(uint8_t INT1_PIN){
  _int1 = INT1_PIN;
  pinMode(_int1, INPUT);
}

void ADXL355::setINT2_pin(uint8_t INT2_PIN){
  _int2 = INT2_PIN;
  pinMode(_int2, INPUT);
}

bool ADXL355::isDRDY(void){
  return !(digitalRead(_drdy));
}

bool ADXL355::isINT1(void){
  if (digitalRead(_int1) == ADXL355_POL){
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ADXL355::getStatus(void){
  uint8_t stat = read_register(STATUS__);
  return (STATUS_VALUES)stat;
}

void ADXL355::DRDY_On(void){
  stop();
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111011));
  start();
}

void ADXL355::DRDY_Off(void){
  stop();
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111011) | 0b0100);
  start();
}

void ADXL355::Temp_On(void){
  stop();
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111101));
  start();
}

void ADXL355::Temp_Off(void){
  stop();
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111101) | 0b010);
  start();
}


void ADXL355::start(void){
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111110) | 0b0);
  } // Enable measure mode

void ADXL355::stop(void){
  uint8_t tmp = read_register(POWER_CTL);
  write_register(POWER_CTL, (tmp & 0b11111110) | 0b1);
  delay(ADXL355_setDelay);} // Disnable measure mode

int ADXL355::whoami(){
  int t = read_register(PARTID);
  return t;
}

RANGE_VALUES ADXL355::getRange(){
  uint8_t tmp = read_register(Range);
  tmp = tmp & 0b00000011;
  return  (RANGE_VALUES)tmp;
}

void ADXL355::setRange(RANGE_VALUES range){
  stop();
  int temp = read_register(Range);
  write_register(Range, (temp & 0b11111100) | range);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setFilter(HPF_CORNER hpf, ODR_LPF lpf){
  stop();
  write_register(Filter, (hpf << 4) | lpf);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setFilter(uint8_t filter_){
  stop();
  write_register(Filter, filter_);
  start();
  delay(ADXL355_setDelay);
}

ODR_LPF ADXL355::FilterToODR_LPF(uint8_t Filter_){
  uint8_t odr_lpf__ = (Filter_ & 0b00001111);
  return (ODR_LPF)odr_lpf__;
}

HPF_CORNER ADXL355::FilterToHPF_CORNER(uint8_t Filter_){
  uint8_t hpf_corner__ = ((Filter_ & 0b01110000) >> 4);
  return (HPF_CORNER)hpf_corner__;
}

uint32_t ADXL355::getODR_us(){
  int ODR = getODR();
  return 1000000ul/ODR;
}

float ADXL355::getODR(){
  int val = read_register(Filter);
  val = (val & 0b00001111);
  switch (val){
    case ODR_4000: return 4000.; break;
    case ODR_2000: return 2000.; break;
    case ODR_1000: return 1000.; break;
    case ODR_500:  return  500.; break;
    case ODR_250:  return  250.; break;
    case ODR_125:  return  125.; break;
    case ODR_62_5: return 62.500; break;
    case ODR_32:   return 31.250; break;
    case ODR_16:   return 15.625; break;
    case ODR_8:    return  7.813; break;
    case ODR_4:    return  3.906; break;
    default:       return    0;   break;
  }  
}

float ADXL355::getLPF(){
  float _odr = getODR();
  return _odr/4;
}
float ADXL355::getHPF(){
  int _odr = getODR();
  float hpf;
  uint8_t _hpf = (read_register(Filter) & 0b01110000) >> 4;
  switch (_hpf){
    case 0: hpf = 0.0; break;
    case 1: hpf = 24.7*_odr; break;
    case 2: hpf = 6.2084*_odr; break;
    case 3: hpf = 1.5545*_odr; break;
    case 4: hpf = 0.3865*_odr; break;
    case 5: hpf = 0.0954*_odr; break;
    case 6: hpf = 0.0238*_odr; break;
  }
  return hpf/10000;
}

float ADXL355::getGroupDelayCycles(){
  int ODR = read_register(Filter);
  ODR = (ODR & 0b00001111);
  int interp = getExtSync();
  switch (interp){
    case EXT_SYNC_INTERP_ON:
      switch (ODR){
        case ODR_4000: return 3.52f; break;
        case ODR_2000: return 3.01f; break;
        case ODR_1000: return 2.75f; break;
        case ODR_500:  return 2.63f; break;
        case ODR_250:  return 2.58f; break;
        case ODR_125:  return 2.55f; break;
        case ODR_62_5: return 2.53f; break;
        case ODR_32:   return 2.52f; break;
        case ODR_16:   return 2.52f; break;
        case ODR_8:    return 2.52f; break;
        case ODR_4:    return 2.52f; break;
        default:       return 0.00f; break;
      }
    default:
      switch (ODR){
        case ODR_4000: return 2.52f; break;
        case ODR_2000: return 2.00f; break;
        case ODR_1000: return 1.78f; break;
        case ODR_500:  return 1.63f; break;
        case ODR_250:  return 1.57f; break;
        case ODR_125:  return 1.54f; break;
        case ODR_62_5: return 1.51f; break;
        case ODR_32:   return 1.49f; break;
        case ODR_16:   return 1.50f; break;
        case ODR_8:    return 1.50f; break;
        case ODR_4:    return 1.50f; break;
        default:       return 0.00f; break;
      }
  }
}

uint32_t ADXL355::getGroupDelay_us(){
  float GD = getGroupDelayCycles();
  uint32_t ODR_us = getODR_us();
  uint32_t GD_us = ODR_us*GD;//1000ul
  return GD_us;
}

uint16_t ADXL355::getTempRaw(){
  uint8_t tempMeasures[] = {0, 0};
  read_register(TEMP2, tempMeasures, 2);
  uint16_t temp = (tempMeasures[0] << 8) + (tempMeasures[1]);
  return temp;
}

float ADXL355::getTemp(float bias = 1852.0, float slope = -9.05){
  uint16_t temp = getTempRaw();
  float res = ((temp - bias) / slope) + 25;
  return res;
}

int32_t ADXL355::getXRaw(){
  uint8_t axisMeasures[] = {0, 0, 0};
  read_register(XDATA3, axisMeasures, 3);  // Read accelerometer data
  int32_t xdata = (axisMeasures[2] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[0] << 12); // Split data
  xdata = ((xdata<<12)>>12);  // extend the sign bit from bit 19 to bit 31
  delayMicroseconds(100);
  getStatus();
  return xdata;
}

int32_t ADXL355::getYRaw(){
  uint8_t axisMeasures[] = {0, 0, 0};
  read_register(YDATA3, axisMeasures, 3); // Read accelerometer data
  int32_t ydata = (axisMeasures[2] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[0] << 12); // Split data
  ydata = ((ydata<<12)>>12);  // extend the sign bit from bit 19 to bit 31
  delayMicroseconds(100);
  getStatus();
  return ydata;
}

int32_t ADXL355::getZRaw(){
  uint8_t axisMeasures[] = {0, 0, 0};
  read_register(ZDATA3, axisMeasures, 3); // Read accelerometer data
  int32_t zdata = (axisMeasures[2] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[0] << 12); // Split data
  zdata = ((zdata<<12)>>12);  // extend the sign bit from bit 19 to bit 31
  delayMicroseconds(100);
  getStatus();
  return zdata;
}

void ADXL355::get3VRaw(int32_t *data){
  uint8_t axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  read_register(XDATA3, axisMeasures, 9); // Read accelerometer data
  data[0] = twocomp((axisMeasures[2] >> 4) | (axisMeasures[1] << 4) | (axisMeasures[0] << 12));
  data[1] = twocomp((axisMeasures[5] >> 4) | (axisMeasures[4] << 4) | (axisMeasures[3] << 12));
  data[2] = twocomp((axisMeasures[8] >> 4) | (axisMeasures[7] << 4) | (axisMeasures[6] << 12));
  delayMicroseconds(100);
  getStatus();
}

void ADXL355::get3VRaw(int16_t *data){
  uint8_t axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  read_register(XDATA3, axisMeasures, 9); // Read accelerometer data
  data[0] = twocomp((axisMeasures[1]) | (axisMeasures[0] << 8));
  data[1] = twocomp((axisMeasures[4]) | (axisMeasures[3] << 8));
  data[2] = twocomp((axisMeasures[7]) | (axisMeasures[6] << 8));
  delayMicroseconds(100);
  getStatus();
}

void ADXL355::getFIFO3VRaw(int32_t *data){
  uint8_t axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  read_register(FIFO_DATA, axisMeasures, 9); // Read accelerometer data
  data[0] = twocomp((axisMeasures[2] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[0] << 12));
  data[1] = twocomp((axisMeasures[5] >> 4) + (axisMeasures[4] << 4) + (axisMeasures[3] << 12));
  data[2] = twocomp((axisMeasures[8] >> 4) + (axisMeasures[7] << 4) + (axisMeasures[6] << 12));
  delayMicroseconds(100);
  getStatus();
}

uint16_t ADXL355::full_scale(){
  return (2000*(1 << (uint8_t)getRange()));
}

float ADXL355::full_scale_factor(){
  return ((1 << 20)/full_scale());
}

void ADXL355::getOffset(int32_t x, int32_t y, int32_t z){
  uint8_t offsetMeasures[] = {0, 0, 0, 0, 0, 0};
  read_register(OFFSET_X_H, offsetMeasures, 6);

  x = twocomp((offsetMeasures[0] << 12) | (offsetMeasures[1] << 4));
  y = twocomp((offsetMeasures[2] << 12) | (offsetMeasures[3] << 4));
  z = twocomp((offsetMeasures[4] << 12) | (offsetMeasures[5] << 4));
}

void ADXL355::getOffset_mg(int16_t &x, int16_t &y, int16_t &z){
  uint8_t offsetMeasures[] = {0, 0, 0, 0, 0, 0};
  read_register(OFFSET_X_H, offsetMeasures, 6);

  x = twocomp((offsetMeasures[0] << 12) | (offsetMeasures[1] << 4))/full_scale_factor();
  y = twocomp((offsetMeasures[2] << 12) | (offsetMeasures[3] << 4))/full_scale_factor();
  z = twocomp((offsetMeasures[4] << 12) | (offsetMeasures[5] << 4))/full_scale_factor();
}

void ADXL355::setOffset(int32_t x, int32_t y, int32_t z){
  stop();
  int16_t workx = (x >> 4); uint8_t hix = (workx & 0xff00) >> 8; uint8_t lox = workx & 0x00ff;
  int16_t worky = (y >> 4); uint8_t hiy = (worky & 0xff00) >> 8; uint8_t loy = worky & 0x00ff;
  int16_t workz = (z >> 4); uint8_t hiz = (workz & 0xff00) >> 8; uint8_t loz = workz & 0x00ff;
  uint8_t buf[] = {hix, lox, hiy, loy, hiz, loz};
  write_register(OFFSET_X_H, buf, 6);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setOffset_mg(int16_t x, int16_t y, int16_t z){
  int16_t workx = ((int32_t)(x*full_scale_factor()) >> 4); uint8_t hix = (workx & 0xff00) >> 8; uint8_t lox = workx & 0x00ff;
  int16_t worky = ((int32_t)(y*full_scale_factor()) >> 4); uint8_t hiy = (worky & 0xff00) >> 8; uint8_t loy = worky & 0x00ff;
  int16_t workz = ((int32_t)(z*full_scale_factor()) >> 4); uint8_t hiz = (workz & 0xff00) >> 8; uint8_t loz = workz & 0x00ff;
  uint8_t buf[] = {hix, lox, hiy, loy, hiz, loz};
  stop();
  write_register(OFFSET_X_H, buf, 6);
  start();
  delay(ADXL355_setDelay);
}

uint32_t ADXL355::getThresh(){
  uint8_t treshMeasures[] = {0, 0};
  read_register(ACT_TRESH_H, treshMeasures, 2);
  uint32_t tresh = (treshMeasures[0] << 12) | (treshMeasures[1] << 4);
  return tresh;
}

uint16_t ADXL355::getThresh_mg(){
  uint8_t treshMeasures[] = {0, 0};
  read_register(ACT_TRESH_H, treshMeasures, 2);
  uint16_t tresh = ((treshMeasures[0] << 12) | (treshMeasures[1] << 4))/full_scale_factor();
  return tresh;
}

void ADXL355::setThresh(uint32_t thresh){
  stop();
  int16_t workt = (thresh >> 4); 
  uint8_t hi = (workt & 0xff00) >> 8; 
  uint8_t lo = workt & 0x00ff;
  uint8_t buf[] = {hi, lo};
  write_register(ACT_TRESH_H, buf, 2); 
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setThresh_mg(uint16_t thresh_mg){
  stop();
  int16_t workt = ((int32_t)(thresh_mg*full_scale_factor()) >> 4); 
  uint8_t hi = (workt & 0xff00) >> 8; 
  uint8_t lo = workt & 0x00ff;
  uint8_t buf[] = {hi, lo};
  write_register(ACT_TRESH_H, buf, 2); 
  start();
  delay(ADXL355_setDelay);
}

uint8_t ADXL355::getThreshCount(){
  uint8_t count = read_register(ACT_COUNT);
  return count;
}

void ADXL355::setThreshCount(uint8_t count){
  stop();
  write_register(ACT_COUNT, count);
  start();
  delay(ADXL355_setDelay);
}

uint8_t ADXL355::getActivity(){
  uint8_t act = read_register(ACT_EN);
  return act;
}

void ADXL355::setActivity(bool x, bool y, bool z){
  stop();
  uint8_t act = (z << 2) | (y << 1) | x;
  write_register(ACT_EN, act);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setInterrupt(uint8_t value){
  stop();
  write_register(INT_MAP, value);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setInterruptPol(bool POL){
  stop();
  ADXL355_POL = POL;
  int tmp = read_register(Range);
  write_register(Range, (tmp & 0b10111111) | (POL << 6));         // Full Scale +/- 2G
  start();
  delay(ADXL355_setDelay);
}

bool ADXL355::getInterruptPol(){
  bool pol = ((read_register(Range) >> 6) & 0b01);
  return pol;
}

void ADXL355::setExtCLK(EXT_CLK clk){
  uint8_t tmp = read_register(SYNC);
  tmp = (tmp & 0b11111011) | (clk << 2);
  stop();
  write_register(SYNC, tmp);
  start();
  delay(ADXL355_setDelay);
}

void ADXL355::setExtSync(EXT_SYNC sync){
  uint8_t tmp = read_register(SYNC);
  tmp = (tmp & 0b11111100) | sync;
  stop();
  write_register(SYNC, tmp);
  start();
  delay(ADXL355_setDelay);
}

uint8_t ADXL355::getExtCLK(){
  uint8_t count = read_register(SYNC);
  count = (count & 0b00000100) >> 2;
  return count;
}

uint8_t ADXL355::getExtSync(){
  uint8_t count = read_register(SYNC);
  count = (count & 0b00000011);
  return count;
}

/* 
 * Print ADXL355 details
 */
void ADXL355::printDetails(){
  uint8_t _status   = getStatus();
  uint8_t _power    = read_register(POWER_CTL);
  uint8_t _int      = read_register(INT_MAP);
  
  int16_t _offset_x, _offset_y, _offset_z;
  getOffset_mg(_offset_x, _offset_y, _offset_z);

  uint8_t _act_en   = getActivity();

  uint8_t _range;
  switch (getRange()){
    case 1: _range = 2; break;
    case 2: _range = 4; break;
    case 3: _range = 8; break;
  }

  Serial.println("---------- ADXL355 Details ----------");
  Serial.print("IDs::  AD ID: 0x");Serial.print(read_register(DEVID_AD),HEX);
  Serial.print("  Mems ID: 0x");Serial.print(read_register(DEVID_MST),HEX);
  Serial.print("  Dev ID: 0x");Serial.print(whoami(),HEX);
  Serial.print("  Part ID: 0x");Serial.println(read_register(REVID),HEX);
  
  Serial.print("Status::  NVM_BUSY: ");Serial.print((_status>>4)&0b1);
  Serial.print("  Activity: ");Serial.print((_status>>3)&0b1);
  Serial.print("  FIFO_OVR: ");Serial.print((_status>>2)&0b1);
  Serial.print("  FIFO_FULL: ");Serial.print((_status>>1)&0b1);
  Serial.print("  DATA_RDY: ");Serial.println((_status>>0)&0b1);
  
  Serial.print("Power Mode: ");Serial.print(_power & 0b1);
  Serial.print("  DRDY_OFF: ");Serial.print(_power>>2);
  Serial.print("  TEMP_OFF: ");Serial.println((_power>>1) & 0b1);
   
  Serial.print("Filter::   ODR: ");Serial.print(getODR(),2);
  Serial.print("  LPF: ");Serial.print(getLPF(),2);
  Serial.print("  HPF: ");Serial.println(getHPF(),2);
  
  Serial.print("Sync::     Clock: ");Serial.print(getExtCLK());
  Serial.print("  Interp: ");Serial.println(getExtSync());
  
  Serial.print("Range:      +/-");Serial.print(_range);Serial.println("g");
  Serial.print("Offset::    x: ");Serial.print(_offset_x);Serial.print("mg");
  Serial.print("  y: ");Serial.print(_offset_y);Serial.print("mg");
  Serial.print("  z: ");Serial.print(_offset_z);Serial.println("mg");
  
  Serial.print("Activity::  x: ");Serial.print(_act_en >> 2);
  Serial.print("  y: ");Serial.print((_act_en >> 1) & 0b1);
  Serial.print("  z: ");Serial.println(_act_en & 0b1);
  Serial.print("Act thresh (mg): ");Serial.print(getThresh_mg());
  Serial.print("  count: ");Serial.println(getThreshCount());

  Serial.print("Interrupts::  Pol: ");Serial.println(getInterruptPol());
  Serial.print("Int1::  ACT: ");Serial.print((_int >> 3) & 0b1);
  Serial.print("  OVR: ");Serial.print((_int >> 2) & 0b1);
  Serial.print("  FULL: ");Serial.print((_int >> 1) & 0b1);
  Serial.print("  DRDY: ");Serial.println((_int >> 0) & 0b1);
  Serial.print("Int2::  ACT: ");Serial.print((_int >> 7) & 0b1);
  Serial.print("  OVR: ");Serial.print((_int >> 6) & 0b1);
  Serial.print("  FULL: ");Serial.print((_int >> 5) & 0b1);
  Serial.print("  DRDY: ");Serial.println((_int >> 4) & 0b1);

  Serial.print("FIFO::  Entries: ");Serial.print(read_register(FIFO_ENTRIES));
  Serial.print("  Samples: ");Serial.println(read_register(FIFO_SAMPLES));

  Serial.print("Full scale (mg):            ");Serial.println(full_scale());
  Serial.print("Full scale factor (LSB/mg): ");Serial.println(full_scale_factor());
}

/* 
 * Write registry in specific device address
 */
void ADXL355::write_register(uint8_t reg, uint8_t val) {
  beginTransaction();
  dev_spi->transfer((reg << 1) | WRITE_BYTE);
  dev_spi->transfer(val);
  endTransaction();
}

/* 
 * Write multiple registries
 */
void ADXL355::write_register(uint8_t reg, uint8_t *buf, uint8_t len){
  beginTransaction();
  dev_spi->transfer((reg << 1) | WRITE_BYTE);
  while(len--){dev_spi->transfer(*buf++);}
  endTransaction();
}

/* 
 * Read registry in specific device address
 */
uint8_t ADXL355::read_register(uint8_t reg) {
  uint8_t result = 0;
  beginTransaction();
  dev_spi->transfer((reg << 1) | READ_BYTE);
  result = dev_spi->transfer(0x00);
  endTransaction();
  return result;
}

/* 
 * Read multiple registries
 */
void ADXL355::read_register(uint8_t reg, uint8_t *buf, uint8_t len) {
  beginTransaction();
  dev_spi->transfer((reg << 1) | READ_BYTE);
  while (len--){*buf++ = dev_spi->transfer(0x00);}
  endTransaction();
}

/* 
 * Control CS level
 */
void ADXL355::cs(bool level){
  digitalWrite(_cs, level);
}

/* 
 * Begin SPI Transaction
 */
void ADXL355::beginTransaction(){
  cs(LOW);
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
}

/* 
 * End SPI transaction
 */
void ADXL355::endTransaction(){
  dev_spi->endTransaction();
  cs(HIGH);
}

/* 
 * Twos Complement
 */
int32_t ADXL355::twocomp(uint32_t val){
  if (val & (1 << 20 - 1)){
    val = val - (1 << 20);
  }
  return val;
}
