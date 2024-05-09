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

#if SERIAL_DEBUG
    while (!Serial.isConnected()) {
        delay(1);
    }
#endif

    // Even if the sensors failed to initialized, we should still enable these threads
    sosTh_ = Thread("rgb_thread", std::bind(&StickManager::sosThread, this), OS_THREAD_PRIORITY_DEFAULT);
    btnTh_ = Thread("btn_thread", std::bind(&StickManager::btnThread, this), OS_THREAD_PRIORITY_DEFAULT);

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
                    if (!Particle.connected()) {
                        Particle.connect();
                        waitFor(Particle.connected, 30000);
                    }
                    manager->sendRemoteRequest();
                }
            }
        }
    }
}

void StickManager::sosThread(void* arg) {
    auto manager = static_cast<StickManager*>(arg);
    if (manager) {
        while (true) {
            if (manager->sos_) {
                if (!RGB.controlled()) {
                    RGB.control(true);
                }
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
                RGB.color(0, 0, 0);
                noTone(manager->buzzPin_);
                manager->vibMode(0);
            } else {
                RGB.control(false);
            }
            if (manager->sos_) {
                delay(1s);
            } else {
                delay(10);
            }
        }
    }
}

void StickManager::senseAndAction() {
    uint16_t minDist = 255;
    for (const auto inst : tfLunas) {
        if (inst->triggerMeasurement() == 0) {
            delay(10);
            uint16_t dist = 0;
            inst->readDistance(&dist);
            Log.info("%s distance: %dcm", inst->name(), dist);
            if (dist < minDist) {
                minDist = dist;
            }
        }
    }
    if (minDist > SAFE_DISTANCE_CM) {
        vibMode(0);
        alert_ = false;
        return;
    } else if (minDist > RISK_DISTANCE_CM) {
        vibMode(128);
    } else {
        vibMode(255);
    }
    alert_ = true;
}

int StickManager::sendRemoteRequest() {
    auto sos = sos_;
    auto alert = alert_;
    if (Particle.connected()) {
        sos_ = false;
        if (sos || alert) {
            vibMutex_.lock();
            vibMode(0);
            delay(1s);
        }
        if (helpMe_) {
            Particle.publish("Alert", "helpme", PRIVATE);
            vibMode(255, 200, 3);
        } else {
            Particle.publish("Alert", "cancel", PRIVATE);
            vibMode(255, 200, 2);
        }
        if (sos || alert) {
            delay(1s);
            vibMutex_.unlock();
        }
        sos_ = sos;
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
        delay(interval);
        digitalWrite(vibPin_, VIB_OFF);
        delay(interval);
    }
}
