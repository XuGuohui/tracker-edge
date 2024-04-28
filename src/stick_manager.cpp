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

#include "stick_manager.h"

#define SERIAL_DEBUG        0

using namespace stick_manager;

StickManager& StickManager::instance() {
    static StickManager inst(A3, A2, A5, A6, A7, CAN_PWR, &TF_LUNA1, &TF_LUNA2, &TF_LUNA3);
    return inst;
}

int StickManager::init() {
    // Buzzer
    pinMode(buzzPin_, OUTPUT);
    digitalWrite(buzzPin_, BUZZ_OFF); // Off
    tone(buzzPin_, 1000, 100); // 1kHz for 100ms

    // Vibration motor
    pinMode(vibPin_, OUTPUT);
    digitalWrite(vibPin_, VIB_ON); // On
    delay(200);
    digitalWrite(vibPin_, VIB_OFF); // Off

    // Buttons
    pinMode(btn1Pin_, INPUT_PULLUP);
    pinMode(btn2Pin_, INPUT_PULLUP);
    pinMode(btn3Pin_, INPUT_PULLUP);

    // Sensor power
    pinMode(senPwrPin_, OUTPUT);
    digitalWrite(senPwrPin_, HIGH); // On
    delay(50);

#if SERIAL_DEBUG
    while (!Serial.isConnected()) {
        delay(1);
    }
#endif

    RGB.control(true);

    if (digitalRead(btn1Pin_) == BTN_PRESSED) {
        left_->configure();
        middle_->configure();
        right_->configure();
    }

    // We have to enable all the sensors so that the I2C pull-ups can make the bus stable.
    for (const auto inst : tfLunas) {
        inst->enable();
    }
    for (const auto inst : tfLunas) {
        if (inst->init() < 0) {
            Log.error("Failed to initialize %s", inst->name());
            return -1;
        }
        uint8_t sig[5] = {'\0'};
        inst->readSignature(sig);
        Log.info("%s Signature: \"%s\"", inst->name(), sig);
    }

    lastSenseTime_ = millis();

    return 0;
}

void StickManager::loop() {
    if (millis() - lastSenseTime_ >= SENSE_INTERVAL_MS) {
        lastSenseTime_ = millis();
        for (const auto inst : tfLunas) {
            if (inst->triggerMeasurement() == 0) {
                uint16_t dist = 0;
                inst->readDistance(&dist);
                Log.info("%s distance: %dcm", inst->name(), dist);
            }
        }
    }
    static uint16_t interval = 500;
    vibOn(interval);
    interval -= 10;
    if (interval < 100) {
        interval = 500;
    }
}

StickManager::StickManager(uint8_t buzzPin, uint8_t vibPin, uint8_t btn1Pin, uint8_t btn2Pin, uint8_t btn3Pin, uint8_t senPwrPin,
                           TfLuna* left, TfLuna* middle, TfLuna* right)
        : buzzPin_(buzzPin), vibPin_(vibPin), btn1Pin_(btn1Pin), btn2Pin_(btn2Pin), btn3Pin_(btn3Pin), senPwrPin_(senPwrPin),
          left_(left), middle_(middle), right_(right) {
    tfLunas[0] = left_;
    tfLunas[1] = middle_;
    tfLunas[2] = right_;
}

StickManager::~StickManager() {
}

void StickManager::rgbSos() {
    // TODO
}

void StickManager::vibOn(uint32_t interval) {
    digitalWrite(vibPin_, VIB_ON);
    delay(150); // FIXME: Blocking
    digitalWrite(vibPin_, VIB_OFF);
    delay(interval);
}
