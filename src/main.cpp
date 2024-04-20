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

#include "Particle.h"
#include "tracker_config.h"
#include "tracker.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if TRACKER_PRODUCT_NEEDED
PRODUCT_ID(TRACKER_PRODUCT_ID);
#endif // TRACKER_PRODUCT_NEEDED
PRODUCT_VERSION(TRACKER_PRODUCT_VERSION);

STARTUP(
    Tracker::startup();
);

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

uint8_t vibPin = A2;
uint8_t buzzPin = A3;
uint8_t btn1Pin = A5;
uint8_t btn2Pin = A6;
uint8_t btn3Pin = A7;

uint8_t senREn = A4;
uint8_t senLEn = D8;
uint8_t senMEn = D9;
uint8_t senPwr = CAN_PWR;

#define I2C_DEEFAULT_SLAVE_ADDRESS          0x10

void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    Wire.write(value);
    int r = Wire.endTransmission(true);
    // Log.info("Wire.endTransmission 1: %d", r);
}

void writeRegisters(uint8_t slaveAddr, uint8_t regAddr, uint8_t* value, uint8_t len) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    Wire.write(value, len);
    int r = Wire.endTransmission(true);
    // Log.info("Wire.endTransmission 3: %d", r);
}

void readRegister(uint8_t slaveAddr, uint8_t regAddr, uint8_t* value) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    Wire.endTransmission(false);
    Wire.requestFrom(slaveAddr, 1);
    *value = Wire.read();
}

void readRegisters(uint8_t slaveAddr, uint8_t regAddr, uint8_t* value, uint8_t len) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    uint8_t r = Wire.endTransmission(false);
    // Log.info("Wire.endTransmission 2: %d", r);
    uint8_t ret = Wire.requestFrom(slaveAddr, len);
    // Log.info("Wire.requestFrom: %d", ret);
    for (int i = 0; i < ret; i++) {
        value[i] = Wire.read();
    }
}

void tfLunaI2CInit(uint8_t enPin, uint8_t newSlaveAddr) {
    Wire.begin();

    digitalWrite(enPin, HIGH);
    delay(500);

    uint8_t sig[5] = {'\0'};
    readRegisters(I2C_DEEFAULT_SLAVE_ADDRESS, 0x3C, sig, 4); // Verified using the default address will no longer work
    Log.info("TF-Luna Signature (Default): \"%c%c%c%c\"", sig[0], sig[1], sig[2], sig[3]);

    writeRegister(I2C_DEEFAULT_SLAVE_ADDRESS, 0x22, newSlaveAddr); // Set new slave address
    writeRegister(I2C_DEEFAULT_SLAVE_ADDRESS, 0x28, 1); // low power mode
    writeRegister(I2C_DEEFAULT_SLAVE_ADDRESS, 0x20, 1); // Save to non-volatile memory
    writeRegister(I2C_DEEFAULT_SLAVE_ADDRESS, 0x21, 2); // Reboot
    delay(1000);

    uint8_t sig0[5] = {'\0'};
    readRegisters(newSlaveAddr, 0x3C, sig0, 4); // Verified using the default address will no longer work
    Log.info("TF-Luna Signature (Reboot): \"%c%c%c%c\"", sig0[0], sig0[1], sig0[2], sig0[3]);
    // Power cycle the sensor if cannot read the signature

    digitalWrite(enPin, LOW);
    delay(100);
    digitalWrite(enPin, HIGH);
    delay(500);
    uint8_t sig1[5] = {'\0'};
    readRegisters(newSlaveAddr, 0x3C, sig1, 4); // Verified the address is persistent
    Log.info("TF-Luna Signature (Power cycle): \"%c%c%c%c\"", sig1[0], sig1[1], sig1[2], sig1[3]);

    uint8_t buf[4] = {0};
    readRegisters(newSlaveAddr, 0x28, buf, 1);
    Log.info("low power: %d", buf[0]);

    // uint8_t outputFreq[2] = {1, 0x00};
    // writeRegisters(0x26, outputFreq, 2); // Output frequency in Hz
    writeRegister(newSlaveAddr, 0x23, 1); // Software-trigger measuement
    writeRegister(newSlaveAddr, 0x24, 1); // Trigger

    readRegisters(newSlaveAddr, 0x00, buf, 4);
    uint16_t dist = (buf[1] << 8) | buf[0];
    uint16_t amp = (buf[3] << 8) | buf[2];
    readRegisters(newSlaveAddr, 0x08, buf, 2);
    uint16_t error = (buf[1] << 8) | buf[0];
    Log.info("Distance: %dcm, AMP: %d, error:  %d", dist, amp, error);
}

void setup()
{
    pinMode(vibPin, OUTPUT);
    digitalWrite(vibPin, LOW); // Off
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(vibPin, HIGH);
        delay(200);
        digitalWrite(vibPin, LOW);
        delay(100);
    }

    pinMode(buzzPin, OUTPUT);
    digitalWrite(buzzPin, LOW); // Off
    tone(buzzPin, 1000, 300);

    pinMode(btn1Pin, INPUT);
    pinMode(btn2Pin, INPUT);
    pinMode(btn3Pin, INPUT);

    pinMode(senREn, OUTPUT);
    digitalWrite(senREn, LOW); // Off
    pinMode(senLEn, OUTPUT);
    digitalWrite(senLEn, LOW); // Off
    pinMode(senMEn, OUTPUT);
    digitalWrite(senMEn, LOW); // Off
    pinMode(senPwr, OUTPUT);
    digitalWrite(senPwr, HIGH); // On

    delay(5s);

    tfLunaI2CInit(senREn, 0x11);
    tfLunaI2CInit(senMEn, 0x13);
    tfLunaI2CInit(senLEn, 0x12);

    Tracker::instance().init();
}

void loop()
{
    Tracker::instance().loop();

    if (digitalRead(btn1Pin) == LOW) {
        digitalWrite(vibPin, HIGH);
        delay(100);
        digitalWrite(vibPin, LOW);
        delay(100);

        digitalWrite(senREn, HIGH);
        digitalWrite(senLEn, LOW);
        digitalWrite(senMEn, LOW);
    }
    if (digitalRead(btn2Pin) == LOW) {
        digitalWrite(vibPin, HIGH);
        delay(200);
        digitalWrite(vibPin, LOW);
        delay(200);

        digitalWrite(senREn, LOW);
        digitalWrite(senLEn, HIGH);
        digitalWrite(senMEn, LOW);
    }
    if (digitalRead(btn3Pin) == LOW) {
        digitalWrite(vibPin, HIGH);
        delay(300);
        digitalWrite(vibPin, LOW);
        delay(300);

        digitalWrite(senREn, LOW);
        digitalWrite(senLEn, LOW);
        digitalWrite(senMEn, HIGH);
    }

    // uint8_t buf[4] = {0};
    // readRegisters(0x00, buf, 4);
    // uint16_t dist = (buf[1] << 8) | buf[0];
    // uint16_t amp = (buf[3] << 8) | buf[2];
    // readRegisters(0x08, buf, 2);
    // uint16_t error = (buf[1] << 8) | buf[0];
    // Log.info("Distance: %dcm, AMP: %d, error:  %d", dist, amp, error);
}
