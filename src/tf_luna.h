
#pragma once

#include "Particle.h"

namespace tf_luna {

#define CHECK_TF_LUNA(_expr) \
        ({ \
            const auto _ret = _expr; \
            if (_ret < 0) { \
                return _ret; \
            } \
            _ret; \
        })

#define CHECK_TE_LUNA_TRUE(_expr, _ret) \
        do { \
            const bool _ok = (bool)(_expr); \
            if (!_ok) { \
                return _ret; \
            } \
        } while (false)

typedef enum {
    TF_LUNNA_IDX0 = 0,
    TF_LUNNA_IDX1 = 1,
    TF_LUNNA_IDX2 = 2,
    TF_LUNNA_NUM = 3
} TfLunaIndex;

typedef enum {
    TF_LUNA_ERROR_NONE = 0,
    TF_LUNA_ERROR_I2C_WRITE = -1,
    TF_LUNA_ERROR_I2C_READ = -2,
    TF_LUNA_ERROR_NOT_FOUND = -3,
    TF_LUNA_ERROR_RESET = -4,
    TF_LUNA_ERROR_INVALID_ARG = -5,
    TF_LUNA_ERROR_INVALID_OBJ = -6,
    TF_LUNA_ERROR_INVALID_STATE = -7,
    TF_LUNA_ERROR_TIMEOUT = -8,
} TfLunaError;

typedef enum {
    TF_LUNA_TRIG_MODE_CONTINUOUS = 0,
    TF_LUNA_TRIG_MODE_SOFTWARE = 1,
} TfLunaTriggerMode;

/*
 * When there are more than two sensors are attached on the same I2C bus,
 * the powered-off sensors may cause the bus to be unstable. To avoid this,
 * we have to enable all of the sensors before we start the I2C communication.
 */
class TfLuna {
public:
    int init();
    /*
     * Configure the sensor with the allocated address. Since the I2C slave address
     * is saved to the NVM, if we moved the sensor to any of the other ports, we must
     * call this function to configure the sensor with the new address that is associated
     * with the new port.
     */
    int configure(bool verify = true);
    int setOutputFreq(uint8_t freqHz) const;
    int setTriggerMode(TfLunaTriggerMode mode) const;
    int triggerMeasurement() const;
    int readDistance(uint16_t* distance) const;
    int readAmp(uint16_t* amp) const;
    int readError(uint16_t* error) const;
    int readSignature(uint8_t* sig) const;
    void enable(bool en = true) const;
    const char* name() const;

    static TfLuna& instance(TfLunaIndex index);

private:
    TfLuna();
    TfLuna(TwoWire* i2c, uint8_t enPin, uint8_t slaveAddr, const char* name);
    ~TfLuna();

    int waitReady() const;
    int writeRegister(uint8_t regAddr, uint8_t value) const;
    int writeRegisters(uint8_t regAddr, uint8_t* value, uint8_t len) const;
    int readRegister(uint8_t regAddr, uint8_t* value) const;
    int readRegisterWord(uint8_t regAddr, uint16_t* value) const;
    int readRegisters(uint8_t regAddr, uint8_t* value, uint8_t len) const;

    static constexpr uint32_t BOOT_DELAY_MS = 50;
    static constexpr uint32_t TF_LUNA_TIMEOUT_MS = 1000;
    static constexpr uint8_t I2C_DEFAULT_SLAVE_ADDRESS = 0x10;
    static constexpr uint8_t I2C_SLAVE_ADDRESS_START = 0x08;
    static constexpr uint8_t I2C_SLAVE_ADDRESS_END = 0x77;
    static constexpr uint16_t SAMPLE_RATE_HZ = 500;
    static constexpr uint16_t OUTPUT_RATE_MAX_HZ = SAMPLE_RATE_HZ / 2;
    static constexpr uint16_t OUTPUT_RATE_MIN_HZ = SAMPLE_RATE_HZ / 500;
    static constexpr uint8_t TF_LUNA_SIG[4] = {'L', 'U', 'N', 'A'};

    // Register addresses
    static constexpr uint8_t REG_DISTANCE = 0x00;
    static constexpr uint8_t REG_AMP = 0x02;
    static constexpr uint8_t REG_ERROR = 0x08;
    static constexpr uint8_t REG_SAVE_TO_NVM = 0x20;
    static constexpr uint8_t REG_REBOOT = 0x21;
    static constexpr uint8_t REG_SLAVE_ADDR = 0x22;
    static constexpr uint8_t REG_TRIG_MODE = 0x23;
    static constexpr uint8_t REG_SW_TRIG = 0x24;
    static constexpr uint8_t REG_ENABLE = 0x25;
    static constexpr uint8_t REG_OUTPUT_FREQ = 0x26;
    static constexpr uint8_t REG_LOW_POWER_MODE = 0x28;
    static constexpr uint8_t REG_FAC_SETTINGS = 0x29;
    static constexpr uint8_t REG_SIG = 0x3C;

    TwoWire* i2c_;
    uint8_t enPin_;
    uint8_t slaveAddr_;
    bool configured_;
    const char* name_;
    bool valid_;
};

class TfLunaEnabledGuard {
public:
    TfLunaEnabledGuard(TfLuna* instance) {
        instance_ = instance;
        instance_->enable();
    }
    ~TfLunaEnabledGuard() {
        instance_->enable(false);
    }

private:
    TfLuna* instance_;
};

} // namespace tf_luna

#define TF_LUNA1                TfLuna::instance(TF_LUNNA_IDX0)
#define TF_LUNA2                TfLuna::instance(TF_LUNNA_IDX1)
#define TF_LUNA3                TfLuna::instance(TF_LUNNA_IDX2)
