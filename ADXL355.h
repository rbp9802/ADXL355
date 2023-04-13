#ifndef _ADXL355_H
#define _ADXL355_H

#include <stdint.h>

/* Memory Map */
#define DEVID_AD     0x00  // R
#define DEVID_MST    0x01  // R
#define PARTID       0x02  // R
#define REVID        0x03  // R
#define STATUS__     0x04  // R
#define FIFO_ENTRIES 0x05  // R
#define TEMP2        0x06  // R
#define TEMP1        0x07  // R
#define XDATA3       0x08  // R
#define XDATA2       0x09  // R
#define XDATA1       0x0A  // R
#define YDATA3       0x0B  // R
#define YDATA2       0x0C  // R
#define YDATA1       0x0D  // R
#define ZDATA3       0x0E  // R
#define ZDATA2       0x0F  // R
#define ZDATA1       0x10  // R
#define FIFO_DATA    0x11  // R
#define OFFSET_X_H   0x1E  // R/W
#define OFFSET_X_L   0x1F  // R/W
#define OFFSET_Y_H   0x20  // R/W
#define OFFSET_Y_L   0x21  // R/W
#define OFFSET_Z_H   0x22  // R/W
#define OFFSET_Z_L   0x23  // R/W
#define ACT_EN       0x24  // R/W
#define ACT_TRESH_H  0x25  // R/W
#define ACT_TRESH_L  0x26  // R/W
#define ACT_COUNT    0x27  // R/W
#define Filter       0x28  // R/W
#define FIFO_SAMPLES 0x29  // R/W
#define INT_MAP      0x2A  // R/W
#define SYNC         0x2B  // R/W
#define Range        0x2C  // R/W
#define POWER_CTL    0x2D  // R/W
#define SELF_TEST    0x2E  // R/W
#define Reset        0x2F  // W

#define READ_BYTE    0x01
#define WRITE_BYTE   0x00

typedef enum  {
  STANDBY_MODE  = 0x01,
  MEASURE_MODE  = 0x00,
  TEMP_OFF      = 0x02,
  TEMP_ON       = ~TEMP_OFF
}POWER_CTL_VALUES;

typedef enum {
  RANGE_2G   = 0x01,
  RANGE_4G   = 0x02,
  RANGE_8G   = 0x03,
  RANGE_MASK = 0x03
}RANGE_VALUES;

typedef enum {
  NOT_ENABLED  = 0x00,
  ODR_X_2_47   = 0x01,
  ODR_X_62_084 = 0x02,
  ODR_X_15_545 = 0x03,
  ODR_X_3_862  = 0x04,
  ODR_X_0_954  = 0x05,
  ODR_X_0_238  = 0x06,
  HPF_CORNER_MASK = 0x70
}HPF_CORNER;

typedef enum {
  ODR_4000 = 0x00,   // LPF = 1000Hz
  ODR_2000 = 0x01,   // LPF =  500Hz
  ODR_1000 = 0x02,   // LPF =  250Hz
  ODR_500  = 0x03,   // LPF =  125Hz
  ODR_250  = 0x04,   // LPF = 62.500Hz
  ODR_125  = 0x05,   // LPF = 31.250Hz
  ODR_62_5 = 0x06,   // LPF = 15.625Hz
  ODR_32   = 0x07,   // ODR = 31.25Hz LPF =  7.813Hz
  ODR_16   = 0x08,   // ODR = 15.675Hz LPF =  3.906Hz
  ODR_8    = 0x09,   // ODR = 7.813Hz LPF =  1.953Hz
  ODR_4    = 0x0A,   // ODR = 3.906Hz LPF =  0.977Hz
  ODR_LPF_MASK = 0x0F
} ODR_LPF;

typedef enum {
  NVM_BUSY_  = 0x10,
  ACTIVITY_  = 0x08,
  FIFO_OVR_  = 0x04,
  FIFO_FULL_ = 0x02,
  DATA_RDY_  = 0x01
} STATUS_VALUES;

typedef enum {
  I2C_SPEED_FAST = 0x80,
  I2C_SPEED_SLOW = 0x00
} I2C_SPEED_VALUES;

typedef enum {
  EXT_CLK_ON  = 0b1,            //  --> Supply a 1.024 MHz Clock signal on the INT2 pin and a SYNC pulse on the DRDY pin at the desired ODR. See datasheet Table 14.
  EXT_CLK_OFF = 0b0             
} EXT_CLK;

typedef enum {
  INT_SYNC = 0b00,              //  --> INT1, INT2 and DRDY pins available
  EXT_SYNC_INTERP_OFF = 0b01,   //  --> SYNC pulse on the DRDY pin. Do not use if EXT_CLK_OFF
  EXT_SYNC_INTERP_ON  = 0b10    //  --> Make sure to send DRDY through INT1 (setInterrupt(0b00000001)) or INT2 (if EXT_CLK_OFF) (setInterrupt(0b00010000)).  
} EXT_SYNC;                      //      SYNC pulse on the DRDY pin at the desired ODR. See datasheet Table 14.

  
class ADXL355{
  public:
    ADXL355(uint8_t cs_pin);
    ADXL355(SPIClass *spi, uint8_t cs_pin);
    ADXL355(SPIClass *spi, uint8_t cs_pin, uint32_t spi_speed);
    void setSPI_pins(uint8_t sck, uint8_t miso, uint8_t mosi);
    //ADXL355(TwoWire *i2c, uint8_t address);
    uint8_t getStatus(void);
    void begin(void);
    void end(void);
    void start(void);
    void stop(void);
    int whoami();

    float    full_scale_factor();
    uint16_t full_scale();
    
    void setDRDY_pin(uint8_t DRDY_PIN);
    void setSYNC_pin(uint8_t SYNC_PIN);
    void syncPulse();
    void setINT1_pin(uint8_t INT1_PIN);
    void setINT2_pin(uint8_t INT2_PIN);
    bool isDRDY(void);
    bool isINT1(void);
    void DRDY_On(void);
    void DRDY_Off(void);
    
    void setRange(RANGE_VALUES range);
    RANGE_VALUES getRange();
    void setFilter(HPF_CORNER hpf, ODR_LPF lpf);
    void setFilter(uint8_t filter_);
    float getODR();
    float getLPF();
    float getHPF();
    ODR_LPF FilterToODR_LPF(uint8_t Filter_);
    HPF_CORNER FilterToHPF_CORNER(uint8_t Filter_);

    void Temp_On(void);
    void Temp_Off(void);
    uint16_t getTempRaw();
    float getTemp(float bias, float slope);
    
    int32_t getXRaw();
    int32_t getYRaw();
    int32_t getZRaw();
    void get3VRaw(int32_t *data);
    void get3VRaw(int16_t *data);
    void getFIFO3VRaw(int32_t *data);

    
    uint32_t getODR_us();
    float getGroupDelayCycles();
    uint32_t getGroupDelay_us();
    
    uint8_t getActivity();
    void setActivity(bool x, bool y, bool z);
    void getOffset(int32_t x, int32_t y, int32_t z); // in LSB
    void getOffset_mg(int16_t &x, int16_t &y, int16_t &z); // in mg
    void setOffset(int32_t x, int32_t y, int32_t z); // in LSB
    void setOffset_mg(int16_t x, int16_t y, int16_t z); // in mg
    uint32_t getThresh();
    uint16_t getThresh_mg();
    void     setThresh(uint32_t thresh);  // in LSB
    void     setThresh_mg(uint16_t thresh_mg);  // in mg
    uint8_t getThreshCount();
    void setThreshCount(uint8_t count);
    
    
    void setInterrupt(uint8_t value);
    void setInterruptPol(bool POL);
    bool getInterruptPol();
    
    void setExtCLK(EXT_CLK clk);
    void setExtSync(EXT_SYNC sync);
    uint8_t getExtCLK();
    uint8_t getExtSync();

    void printDetails();
    
  
  private:
    void write_register(uint8_t reg, uint8_t val);
    void write_register(uint8_t reg, uint8_t *buf, uint8_t len);
    void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
    uint8_t read_register(uint8_t reg);
    
    int32_t twocomp(uint32_t value);
    uint8_t _cs;
    uint8_t _miso = MISO;
    uint8_t _mosi = MOSI;
    uint8_t _sck  = SCK;
    uint8_t _int1; // ADXL355_INT1_PIN;
    uint8_t _int2; // ADXL355_INT2_PIN;
    uint8_t _drdy; // ADXL355_DRDY_PIN; 
    uint8_t ADXL355_setDelay = 10;
    bool    ADXL355_POL = 0;

    uint8_t  address;    //i2c
    uint32_t spi_speed; //spi
    uint8_t  _deviceId;
    
    //TwoWire *dev_i2c;
    SPIClass *dev_spi;
    void cs(bool level);
    void beginTransaction();
    void endTransaction();
    
};
#endif // ifndef _ADXL355_H
