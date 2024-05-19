/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "tf_luna.h"

using namespace tf_luna;

TfLuna& TfLuna::instance(TfLunaIndex index) {
    static TfLuna inst[TF_LUNNA_NUM] = {
        { &Wire, D8, 0x13, "Left TF-Luna" },
        { &Wire, D9, 0x12, "Middle TF-Luna" },
        { &Wire, A4, 0x11, "Right TF-Luna" },
    };
    if (index >= TF_LUNNA_NUM) {
        auto obj = new TfLuna();
        return *obj;
    }
    return inst[index];
}

void TfLuna::enable(bool en) const {
    pinMode(enPin_, OUTPUT);
    if (en) {
        digitalWrite(enPin_, HIGH); // Enable the sensor
        delay(BOOT_DELAY_MS);
    } else {
        digitalWrite(enPin_, LOW);
    }
}

int TfLuna::init() {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    enable();
    if (!i2c_->isEnabled()) {
        i2c_->begin();
    }
    Log.info("Wait %s to be ready...", name());
    CHECK_TF_LUNA(waitReady());  // or delay(BOOT_DELAY_MS);
    CHECK_TF_LUNA(setTriggerMode(TF_LUNA_TRIG_MODE_SOFTWARE));
    Log.info("%s initialized", name());
    configured_ = true;
    return TF_LUNA_ERROR_NONE;
}

// NOTE: Only up to two sensors can be attached at the same time when configuring the sensors.
// Otherwise, it may fail to configure the sensors.
int TfLuna::configure(bool verify) {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    enable();
    if (!i2c_->isEnabled()) {
        i2c_->begin();
    }

    TfLunaEnabledGuard guard(this);
    delay(2s);

    uint8_t sig[4];
    uint8_t expectedSlaveAddr = slaveAddr_;

    Log.info("Configuring %s...", name());

    // Try with the default address first
    Log.info("Try with the default address 0x%02X", I2C_DEFAULT_SLAVE_ADDRESS);
    slaveAddr_ = I2C_DEFAULT_SLAVE_ADDRESS;
    if (readRegisters(REG_SIG, sig, 4) != 0) {
        // Try with the allocated address
        Log.info("Try with the allocated address 0x%02X", expectedSlaveAddr);
        slaveAddr_ = expectedSlaveAddr;
        if (readRegisters(REG_SIG, sig, 4) != 0) {
            // Traverse all the other potential slave addresses
            Log.info("Try with other addresses");
            for (uint8_t addr = I2C_SLAVE_ADDRESS_START; addr <= I2C_SLAVE_ADDRESS_END + 1; addr++) {
                if (addr == I2C_DEFAULT_SLAVE_ADDRESS || addr == slaveAddr_) {
                    continue;
                }
                slaveAddr_ = addr;
                if (readRegisters(REG_SIG, sig, 4) == 0) {
                    break;
                }
            }
        } else {
            // The slave address is not changed, i.e. the non-volatile settings are still valid
            Log.info("Non-volatile settings are still valid");
            return TF_LUNA_ERROR_NONE;
        }
    }
    if (slaveAddr_ == (I2C_SLAVE_ADDRESS_END + 1)) {
        Log.error("The %s is probably not attached", name());
        return TF_LUNA_ERROR_NOT_FOUND;
    }
    Log.info("Deprecated address 0x%02X found, set to 0x%02X", slaveAddr_, expectedSlaveAddr);

    CHECK_TF_LUNA(writeRegister(REG_SLAVE_ADDR, expectedSlaveAddr)); // Set new slave address, applied after reboot
    slaveAddr_ = expectedSlaveAddr;
    Log.info("Waiting to be ready...");
    CHECK_TF_LUNA(waitReady());
    Log.info("Saving config to non-volatile memory...");
    CHECK_TF_LUNA(writeRegister(REG_SAVE_TO_NVM, 0x01)); // Save to non-volatile memory
    delay(50); // Make sure the settings are saved

    // Verify the new settings
    if (verify) {
        Log.info("Rebooting...");
        CHECK_TF_LUNA(writeRegister(REG_REBOOT, 0x02)); // Reboot
        Log.info("Waiting to be ready...");
        CHECK_TF_LUNA(waitReady());
        Log.info("Successfully reset the %s at 0x%02X", name(), slaveAddr_);
    }

    return TF_LUNA_ERROR_NONE;
}

int TfLuna::setOutputFreq(uint8_t freqHz) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    if (freqHz < OUTPUT_RATE_MIN_HZ) {
        freqHz = OUTPUT_RATE_MIN_HZ;
    } else if (freqHz > OUTPUT_RATE_MAX_HZ) {
        freqHz = OUTPUT_RATE_MAX_HZ;
    } else {
        uint8_t minDiff = OUTPUT_RATE_MAX_HZ;
        for (uint16_t n = 3; n <= SAMPLE_RATE_HZ; n++) {
            uint8_t diff = abs(freqHz - (SAMPLE_RATE_HZ / n));
            if (diff < minDiff) {
                minDiff = diff;
            } else {
                freqHz = SAMPLE_RATE_HZ / n;
                break;
            }
        }
    }
    CHECK_TF_LUNA(writeRegister(REG_OUTPUT_FREQ, freqHz)); // The HIGH byte is always 0x00
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::setTriggerMode(TfLunaTriggerMode mode) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    // Note: the sequence of the following settings is important
    // 1. Set the trigger mode
    // 2. Set the power mode
    // 3. Set the output frequency
    // Otherwise, the output frequency may not be set correctly
    CHECK_TF_LUNA(writeRegister(REG_TRIG_MODE, mode));
    CHECK_TF_LUNA(writeRegister(REG_LOW_POWER_MODE, 0)); // Normal mode
    if (mode == TF_LUNA_TRIG_MODE_SOFTWARE) {
        CHECK_TF_LUNA(writeRegister(REG_OUTPUT_FREQ, 0)); // Set the output frequency to 0 to make sure the software trigger mdoe works
    } else {
        CHECK_TF_LUNA(writeRegister(REG_OUTPUT_FREQ, 2/*Hz*/));
    }
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::triggerMeasurement() const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    CHECK_TE_LUNA_TRUE(configured_, TF_LUNA_ERROR_INVALID_STATE);
    CHECK_TF_LUNA(writeRegister(REG_SW_TRIG, 0x01));
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readDistance(uint16_t* distance) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    CHECK_TE_LUNA_TRUE(configured_, TF_LUNA_ERROR_INVALID_STATE);
    if (distance == nullptr) {
        return TF_LUNA_ERROR_NONE;
    }
    CHECK_TF_LUNA(readRegisterWord(REG_DISTANCE, distance));
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readAmp(uint16_t* amp) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    CHECK_TE_LUNA_TRUE(configured_, TF_LUNA_ERROR_INVALID_STATE);
    if (amp == nullptr) {
        return TF_LUNA_ERROR_NONE;
    }
    CHECK_TF_LUNA(readRegisterWord(REG_AMP, amp));
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readError(uint16_t* error) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    CHECK_TE_LUNA_TRUE(configured_, TF_LUNA_ERROR_INVALID_STATE);
    if (error == nullptr) {
        return TF_LUNA_ERROR_NONE;
    }
    CHECK_TF_LUNA(readRegisterWord(REG_ERROR, error));
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readSignature(uint8_t* sig) const {
    CHECK_TE_LUNA_TRUE(valid_, TF_LUNA_ERROR_INVALID_OBJ);
    memset(sig, '\0', 4);
    CHECK_TF_LUNA(readRegisters(REG_SIG, sig, 4));
    return TF_LUNA_ERROR_NONE;
}


// Private methods
TfLuna::TfLuna(TwoWire* i2c, uint8_t enPin, uint8_t slaveAddr, const char* name)
        : i2c_(i2c), enPin_(enPin), slaveAddr_(slaveAddr), configured_(false), name_(name) {
    valid_ = true;
    if (!name) {
        name_ = "TF-Luna";
    }
}

TfLuna::TfLuna() {
    valid_ = false;
}

TfLuna::~TfLuna() {
}

int TfLuna::waitReady() const {
    uint64_t start = millis();
    while (readRegister(REG_SIG, nullptr) != 0 && (millis() - start < TF_LUNA_TIMEOUT_MS)) {
        delay(10);
    }
    if (millis() - start > TF_LUNA_TIMEOUT_MS) {
        return TF_LUNA_ERROR_TIMEOUT;
    }
    return TF_LUNA_ERROR_NONE;
}

const char* TfLuna::name() const {
    return name_;
}

int TfLuna::writeRegister(uint8_t regAddr, uint8_t value) const {
    i2c_->beginTransmission(slaveAddr_);
    i2c_->write(regAddr);
    i2c_->write(value);
    int r = i2c_->endTransmission(true);
    if (r != 0) {
        // Log.error("i2c_->endTransmission: %d", r);
        return TF_LUNA_ERROR_I2C_WRITE;
    }
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::writeRegisters(uint8_t regAddr, uint8_t* value, uint8_t len) const {
    i2c_->beginTransmission(slaveAddr_);
    i2c_->write(regAddr);
    i2c_->write(value, len);
    int r = i2c_->endTransmission(true);
    if (r != 0) {
        // Log.error("i2c_->endTransmission: %d", r);
        return TF_LUNA_ERROR_I2C_WRITE;
    }
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readRegister(uint8_t regAddr, uint8_t* value) const {
    i2c_->beginTransmission(slaveAddr_);
    i2c_->write(regAddr);
    int r = i2c_->endTransmission(false);
    if (r != 0) {
        // Log.error("i2c_->endTransmission: %d", r);
        return TF_LUNA_ERROR_I2C_WRITE;
    }
    r = i2c_->requestFrom(slaveAddr_, 1);
    if (r != 1) {
        // Log.error("i2c_->requestFrom: %d", r);
        return TF_LUNA_ERROR_I2C_READ;
    }
    *value = i2c_->read();
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readRegisterWord(uint8_t regAddr, uint16_t* value) const {
    uint8_t buf[2] = {0};
    CHECK_TF_LUNA(readRegisters(regAddr, buf, 2));
    *value = (buf[1] << 8) | buf[0];
    return TF_LUNA_ERROR_NONE;
}

int TfLuna::readRegisters(uint8_t regAddr, uint8_t* value, uint8_t len) const {
    i2c_->beginTransmission(slaveAddr_);
    i2c_->write(regAddr);
    uint8_t r = i2c_->endTransmission(false);
    if (r != 0) {
        // Log.error("i2c_->endTransmission: %d", r);
        return TF_LUNA_ERROR_I2C_WRITE;
    }
    uint8_t ret = i2c_->requestFrom(slaveAddr_, len);
    if (ret != len) {
        // Log.error("i2c_->requestFrom: %d", ret);
        return TF_LUNA_ERROR_I2C_READ;
    }
    for (int i = 0; i < ret; i++) {
        value[i] = i2c_->read();
    }
    return TF_LUNA_ERROR_NONE;
}
