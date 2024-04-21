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
#include "tf_luna.h"

using namespace tf_luna;

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

uint8_t senPwr = CAN_PWR;

#define I2C_DEEFAULT_SLAVE_ADDRESS          0x10

void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(regAddr);
    Wire.write(value);
    int r = Wire.endTransmission(true);
    // Log.info("Wire.endTransmission 1: %d", r);
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

    if (digitalRead(btn1Pin) == LOW) {
        delay(5s); // To let user have sufficient time to connect the serial monnitor
        TF_LUNA1.configure();
        TF_LUNA2.configure();
        TF_LUNA3.configure();
    }

    TF_LUNA1.init();
    TF_LUNA2.init();
    TF_LUNA3.init();

    uint8_t sig0[5] = {'\0'};
    TF_LUNA1.readSignature(sig0);
    Log.info("TF-Luna Signature 0: \"%c%c%c%c\"", sig0[0], sig0[1], sig0[2], sig0[3]);
    TF_LUNA2.readSignature(sig0);
    Log.info("TF-Luna Signature 1: \"%c%c%c%c\"", sig0[0], sig0[1], sig0[2], sig0[3]);
    TF_LUNA3.readSignature(sig0);
    Log.info("TF-Luna Signature 2: \"%c%c%c%c\"", sig0[0], sig0[1], sig0[2], sig0[3]);

    Tracker::instance().init();
}

void loop()
{
    Tracker::instance().loop();
}