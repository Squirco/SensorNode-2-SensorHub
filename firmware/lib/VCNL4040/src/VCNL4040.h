#ifndef VCNL4040_H
#define VCNL4040_H

#include <Arduino.h>

#define VCNL4040_I2C_ADDR       0x60
#define VCNL4040_ID_L           0x86
#define VCNL4040_ID_M           0x1

// Command Codes
#define VCNL4040_ALS_CONF         0x00
#define VCNL4040_ALS_THDH         0x01
#define VCNL4040_ALS_THDL         0x02
#define VCNL4040_PS_CONF_1_2      0x03
#define VCNL4040_PS_CONF_3_MS     0x04
#define VCNL4040_PS_CANC          0x05
#define VCNL4040_PS_THDL          0x06
#define VCNL4040_PS_THDH          0x07
#define VCNL4040_PS_DATA          0x08
#define VCNL4040_ALS_DATA         0x09
#define VCNL4040_WHITE_DATA       0x0A
#define VCNL4040_INT_FLAG         0x0B
#define VCNL4040_ID               0x0C

/* VCNL4040 Class */
class VCNL4040 {
public:

    //Initialization
    VCNL4040();
    ~VCNL4040();
    bool init();

    //Settings
    bool alsConf(uint8_t conf);
    bool alsSetIntThres(uint16_t high, uint16_t low);
    uint16_t lux();

    bool psConf(uint8_t conf1, uint8_t conf2, uint8_t conf3, uint8_t ms);
    bool psCalibrate(void);
    bool psSetCanc(uint16_t level);
    bool psSetIntThres(uint16_t high, uint16_t low);
    uint16_t ps();
    uint16_t white();

    uint16_t intFlag();
    uint8_t id();

    //I2C Commands
    bool wireWrite8(uint8_t reg, uint8_t val);
    bool wireWrite16(uint8_t reg, uint16_t val);
    bool wireReadStart(uint8_t val);
    bool wireRead8(uint8_t reg, uint8_t &val);
    bool wireRead16(uint8_t reg, uint16_t &val);
};

#endif
