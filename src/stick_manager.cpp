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

// Subscriber: https://go.particle.io/shared_apps/663878b9b029670017f3861b

#include "stick_manager.h"

#define SERIAL_DEBUG        0

using namespace stick_manager;

static Logger stickLog("app.stick");

StickManager& StickManager::instance() {
    static StickManager inst(A3, A2, A5, A6, A7, CAN_PWR, &TF_LUNA1, &TF_LUNA2, &TF_LUNA3);
    return inst;
}

void StickManager::onDataReceivedStatic(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
	auto tfLuna = (TfLuna*)context;
	StickManager::instance().updateSettings(tfLuna, data, len);
}

int StickManager::init() {
    // Buzzer
    pinMode(buzzPin_, OUTPUT);
    digitalWrite(buzzPin_, BUZZ_OFF); // Off
    tone(buzzPin_, 1000, 100); // 1kHz for 100ms

    // Vibration motor
    pinMode(vibPin_, OUTPUT);
    digitalWrite(vibPin_, VIB_ON); // On
    delay(100);
    digitalWrite(vibPin_, VIB_OFF); // Off

    // Buttons
    pinMode(btn1Pin_, INPUT_PULLUP);
    pinMode(btn2Pin_, INPUT_PULLUP);
    pinMode(btn3Pin_, INPUT_PULLUP);

    // Sensor power
    pinMode(senPwrPin_, OUTPUT);
    digitalWrite(senPwrPin_, HIGH); // On
    delay(50);

    System.on(button_click, onButtonClick);

    BleCharacteristic tfLuna1Char("TF-Luna1", BleCharacteristicProperty::WRITE_WO_RSP, TF_LUNA1_UUID, SVC_UUID, onDataReceivedStatic, &TF_LUNA1);
    BleCharacteristic tfLuna2Char("TF-Luna2", BleCharacteristicProperty::WRITE_WO_RSP, TF_LUNA2_UUID, SVC_UUID, onDataReceivedStatic, &TF_LUNA2);
    BleCharacteristic tfLuna3Char("TF-Luna3", BleCharacteristicProperty::WRITE_WO_RSP, TF_LUNA3_UUID, SVC_UUID, onDataReceivedStatic, &TF_LUNA3);
    BLE.addCharacteristic(tfLuna1Char);
    BLE.addCharacteristic(tfLuna2Char);
    BLE.addCharacteristic(tfLuna3Char);
    BLE.advertise();

#if SERIAL_DEBUG
    while (!Serial.isConnected() && !BLE.connected()) {
        delay(1);
    }
    delay(2s);
#endif

    if (digitalRead(btn1Pin_) == BTN_PRESSED) {
        left_->configure();
        middle_->configure();
        right_->configure();
    }

    // Even if the sensors failed to initialized, we should still enable these threads
    sosTh_ = Thread("rgb_thread", std::bind(&StickManager::sosThread, this), OS_THREAD_PRIORITY_DEFAULT);
    btnTh_ = Thread("btn_thread", std::bind(&StickManager::btnThread, this), OS_THREAD_PRIORITY_DEFAULT);

    // We have to enable all the sensors so that the I2C pull-ups can make the bus stable.
    for (const auto inst : tfLunas) {
        inst->enable();
    }
    for (const auto inst : tfLunas) {
        if (inst->init() < 0) {
            stickLog.error("Failed to initialize %s", inst->name());
            return -1;
        }
        uint8_t sig[5] = {'\0'};
        inst->readSignature(sig);
        stickLog.info("%s Signature: \"%s\"", inst->name(), sig);
    }

    setSenseMode(SENSE_MODE_MANUAL);
    senseTh_ = Thread("sense_thread", std::bind(&StickManager::senseThread, this), OS_THREAD_PRIORITY_DEFAULT);
    return 0;
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

void StickManager::onButtonClick(system_event_t ev, int button_data) {
    StickManager::instance().setButtonPressed();
}

void StickManager::setSenseMode(SenseMode mode) {
    senseMode_ = mode;
    if (senseMode_ == SENSE_MODE_AUTO) {
        lastSenseTime_ = millis();
        senseWindowStart_ = millis();
        senseQuietStart_ = 0;
    }
}

void StickManager::senseThread(void* arg) {
    auto manager = static_cast<StickManager*>(arg);
    if (manager) {
        while (true) {
            if (manager->sos_) {
                delay(100);
                // If user is asking for help, we should not sense the environment.
                continue;
            }
            if (manager->senseMode_ == SENSE_MODE_MANUAL) {
                if (digitalRead(manager->btn3Pin_) == BTN_PRESSED) {
                    delay(50); // debunce
                    if (digitalRead(manager->btn3Pin_) == BTN_PRESSED) {
                        manager->senseAndAction();
                    }
                }
                else {
                    manager->vibMode(0);
                    // The next first sense is not reliable
                    manager->skipFirstSense_ = true;
                }
            } else if (manager->senseMode_ == SENSE_MODE_AUTO) {
                if (millis() - manager->senseWindowStart_ > AUTO_SENSE_WINDOW_MS) {
                    if (manager->senseQuietStart_ == 0) {
                        manager->senseQuietStart_ = millis();
                    }
                    if (millis() - manager->senseQuietStart_ > AUTO_SENSE_QUIET_MS) {
                        // Restart a new sensing window
                        manager->setSenseMode(SENSE_MODE_AUTO);
                    }
                    delay(10);
                    continue;
                }
                // Sensing procedure
                if (millis() - manager->lastSenseTime_ >= AUTO_SENSE_INTERVAL_MS) {
                    manager->lastSenseTime_ = millis();
                    manager->senseAndAction();
                }
            } else {
                // Unknown mode
                delay(100);
            }
        }
    }
}

void StickManager::btnThread(void* arg) {
    auto manager = static_cast<StickManager*>(arg);
    if (manager) {
        while (true) {
            if (digitalRead(manager->btn1Pin_) == BTN_PRESSED) {
                delay(50); // debunce
                if (digitalRead(manager->btn1Pin_) == BTN_PRESSED) {
                    while (digitalRead(manager->btn1Pin_) == BTN_PRESSED); // Wait until the button is released
                    manager->sos_ = !manager->sos_;
                }
            }

            if (digitalRead(manager->btn2Pin_) == BTN_PRESSED) {
                delay(50); // debunce
                if (digitalRead(manager->btn2Pin_) == BTN_PRESSED) {
                    while (digitalRead(manager->btn2Pin_) == BTN_PRESSED); // Wait until the button is released
                    manager->helpMe_ = !manager->helpMe_;
                    if (manager->helpMe_) {
                        VibLock lock;
                        manager->vibMode(255, 150, 3);
                    } else {
                        VibLock lock;
                        manager->vibMode(255, 150, 2);
                    }
                    if (!Particle.connected()) {
                        Particle.connect();
                    }
                    manager->helpMeSent_ = false;
                }
            }
            manager->sendRemoteRequest();

            if (manager->modeBtnPressed_) {
                manager->modeBtnPressed_ = false;
                manager->senseMode_++;
                manager->senseMode_ %= (uint8_t)SENSE_MODE_MAX;
                manager->setSenseMode((SenseMode)manager->senseMode_);
                VibLock lock;
                manager->vibMode(255, 150, manager->senseMode_ + 1);
            }
        }
    }
}

void StickManager::sosThread(void* arg) {
    auto manager = static_cast<StickManager*>(arg);
    uint64_t lastSosEndTime = 0;
    if (manager) {
        while (true) {
            if (manager->sos_) {
                if (!RGB.controlled()) {
                    RGB.control(true);
                }
                if (millis() - lastSosEndTime > SOS_INNTERVAL_MS) {
                    for (uint8_t i = 0; i < 3; i++) {
                        for (uint8_t j = 0; j < 3; j++) {
                            if (!manager->sos_) {
                                break;
                            }
                            RGB.color(255, 0, 0);
                            tone(manager->buzzPin_, 1000, 2000/*duration*/);
                            manager->vibMode(255);
                            delay(manager->SOS_PATTERN[i]);

                            if (!manager->sos_) {
                                break;
                            }
                            RGB.color(0, 0, 0);
                            noTone(manager->buzzPin_);
                            digitalWrite(manager->vibPin_, VIB_OFF);
                            manager->vibMode(0);
                            if (i == 0 && j == 2) {
                                delay(manager->SOS_PATTERN[i + 1]);
                            } else {
                                delay(manager->SOS_PATTERN[i]);
                            }
                        }
                    }
                    lastSosEndTime = millis();
                    RGB.color(0, 0, 0);
                    noTone(manager->buzzPin_);
                    manager->vibMode(0);
                } else {
                    delay(10);
                }
            } else {
                RGB.control(false);
                delay(10);
            }
        }
    }
}

void StickManager::updateSettings(TfLuna* tfLuna, const uint8_t* data, size_t len) {
    if (len != 2) {
        return;
    }
    uint8_t idx = 0;
    if (tfLuna == left_) {
        idx = 0;
    } else if (tfLuna == middle_) {
        idx = 1;
    } else if (tfLuna == right_) {
        idx = 2;
    }
    safeDistCm[idx] = data[0];
    riskDistCm[idx] = data[1];
}

void StickManager::senseAndAction() {
    if (skipFirstSense_) {
        skipFirstSense_ = false;
        for (const auto inst : tfLunas) {
            if (inst->triggerMeasurement() == 0) {
                delay(10);
                uint16_t dist = 0;
                inst->readDistance(&dist);
                (void)dist;
            }
            return;
        }
    }
    uint8_t alertLevel = 0;
    uint8_t tfLunaIdx = 0;
    for (uint8_t i = 0; i < 3; i++) {
        if (tfLunas[i]->triggerMeasurement() == 0) {
            delay(10);
            uint16_t dist = 0;
            tfLunas[i]->readDistance(&dist);
            stickLog.info("%s distance: %dcm", tfLunas[i]->name(), dist);
            if (dist > safeDistCm[i]) {
                // Continue
            } else {
                if (dist > riskDistCm[i]) {
                    if (alertLevel < 1) {
                        alertLevel = 1;
                        tfLunaIdx = i;
                    }
                } else {
                    if (alertLevel < 2) {
                        alertLevel = 2;
                        tfLunaIdx = i;
                    }
                }
            }
        }
    }
    if (alertLevel == 0) {
        vibMode(0);
        alert_ = false;
        return;
    }
    uint8_t strength = ((alertLevel == 1) ? 160 : 255);
    uint16_t interval = 0;
    if (tfLunaIdx == 0) {
        interval = 0;
    } else if (tfLunaIdx == 1) {
        interval = 1;
    } else {
        interval = 50;
    }
    vibMode(strength, interval, interval == 0 ? 0 : 1);
    alert_ = true;
}

int StickManager::sendRemoteRequest() {
    if (Particle.connected() && !helpMeSent_) {
        VibLock lock;
        if (helpMe_) {
            Particle.publish("Alert", "helpme", PRIVATE);
        } else {
            Particle.publish("Alert", "cancel", PRIVATE);
        }
        helpMeSent_ = true;
    }
    return 0;
}

void StickManager::vibMode(uint8_t strength, uint16_t interval, uint16_t pulses) {
    std::lock_guard<RecursiveMutex> lock(vibMutex_);
    if (pulses == 0) {
        if (strength == 0) {
            digitalWrite(vibPin_, VIB_OFF);
        } else if (strength == 255) {
            digitalWrite(vibPin_, VIB_ON);
        } else {
            analogWrite(vibPin_, strength, 1000/*Hz*/);
        }
        return;
    }
    for (uint32_t i = 0; i < pulses; i++) {
        analogWrite(vibPin_, strength, 1000/*Hz*/);
        delay(100);
        digitalWrite(vibPin_, VIB_OFF);
        delay(interval);
    }
}
