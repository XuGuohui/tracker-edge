
#pragma once

#include "Particle.h"
#include "tf_luna.h"

namespace stick_manager {

using namespace tf_luna;

enum SenseMode {
    SENSE_MODE_AUTO = 0,
    SENSE_MODE_MANUAL = 1,
    SENSE_MODE_MAX = 2
};

class StickManager {
public:
    int init();

    static StickManager& instance();

    void setButtonPressed() {
        modeBtnPressed_ = true;
    }

    void vibLock() {
        preSos_ = sos_;
        preAlert_ = alert_;
        sos_ = false;
        if (preSos_ || preAlert_) {
            vibMutex_.lock();
            vibMode(0);
            delay(500);
        }
    }

    void vibUnlock() {
        if (preSos_ || preAlert_) {
            delay(500);
            vibMutex_.unlock();
        }
        sos_ = preSos_;
    }

private:
    StickManager(uint8_t buzzPin, uint8_t vibPin, uint8_t btn1Pin, uint8_t btn2Pin, uint8_t btn3Pin, uint8_t senPwrPin,
                 TfLuna* left, TfLuna* middle, TfLuna* right);
    ~StickManager();
    void vibMode(uint8_t strength, uint16_t interval = 0, uint16_t pulses = 0 /* 0: infinite */);
    int sendRemoteRequest();
    void setSenseMode(SenseMode mode);
    void senseAndAction();
    static void sosThread(void* arg);
    static void btnThread(void* arg);
    static void senseThread(void* arg);
    static void onButtonClick(system_event_t ev, int button_data);

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
    uint64_t senseWindowStart_ = 0;
    uint64_t senseQuietStart_ = 0;
    Thread sosTh_; // Local SOS signal
    Thread btnTh_; // Button detection
    Thread senseTh_; // Sensing
    RecursiveMutex vibMutex_;
    volatile bool sos_ = false;
    volatile bool preSos_ = false;
    volatile bool helpMe_ = false;
    volatile bool helpMeSent_ = false;
    volatile bool alert_ = false;
    volatile bool preAlert_ = false;
    volatile bool skipFirstSense_ = false;
    volatile bool modeBtnPressed_ = false;
    volatile uint8_t senseMode_ = SENSE_MODE_AUTO;

    static constexpr uint16_t SOS_PATTERN[3] = {150, 300, 150};
    static constexpr uint8_t BUZZ_ON = HIGH;
    static constexpr uint8_t BUZZ_OFF = LOW;
    static constexpr uint8_t VIB_ON = HIGH;
    static constexpr uint8_t VIB_OFF = LOW;
    static constexpr uint8_t BTN_PRESSED = LOW;
    static constexpr uint8_t BTN_RELEASED = HIGH;

    // ^          ^          ^          ^          ^                                  ^          ^          ^          ^          ^
    // | interval | interval | interval |                                  | interval | interval | interval |
    // |                 window         |                quiet             |                 window         |
    static constexpr uint64_t AUTO_SENSE_INTERVAL_MS = 50;
    static constexpr uint64_t AUTO_SENSE_WINDOW_MS = 1000;
    static constexpr uint64_t AUTO_SENSE_QUIET_MS = 0;

    // Distance more than this value will not trigger alert
    static constexpr uint32_t SAFE_DISTANCE_CM = 150;
    // Distance more than this value will trigger slight alert.
    // And Distance less than this value will trigger strong alert.
    static constexpr uint32_t RISK_DISTANCE_CM = 50;

    static constexpr uint32_t SOS_INNTERVAL_MS = 1000;
};

class VibLock {
public:
    VibLock() {
        StickManager::instance().vibLock();
    }
    ~VibLock() {
        StickManager::instance().vibUnlock();
    }
};

} // namespace stick_manager
