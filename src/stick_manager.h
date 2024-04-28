
#pragma once

#include "Particle.h"
#include "tf_luna.h"

namespace stick_manager {

using namespace tf_luna;

class StickManager {
public:
    int init();
    void loop();

    static StickManager& instance();

private:
    StickManager(uint8_t buzzPin, uint8_t vibPin, uint8_t btn1Pin, uint8_t btn2Pin, uint8_t btn3Pin, uint8_t senPwrPin,
                 TfLuna* left, TfLuna* middle, TfLuna* right);
    ~StickManager();

    void rgbSos();
    void vibOn(uint32_t interval);

    uint8_t buzzPin_;
    uint8_t vibPin_;
    uint8_t btn1Pin_;
    uint8_t btn2Pin_;
    uint8_t btn3Pin_;
    uint8_t senPwrPin_;
    TfLuna* left_;
    TfLuna* middle_;
    TfLuna* right_;
    TfLuna* tfLunas[3];

    uint64_t lastSenseTime_ = 0;

    static constexpr uint8_t BUZZ_ON = HIGH;
    static constexpr uint8_t BUZZ_OFF = LOW;
    static constexpr uint8_t VIB_ON = HIGH;
    static constexpr uint8_t VIB_OFF = LOW;
    static constexpr uint8_t BTN_PRESSED = LOW;
    static constexpr uint8_t BTN_RELEASED = HIGH;

    static constexpr uint64_t SENSE_INTERVAL_MS = 1000;
};

} // namespace stick_manager
