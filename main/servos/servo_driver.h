#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_DEGREE -90
#define SERVO_MAX_DEGREE 90

class ServoDriver {
public:
    ServoDriver();
    ~ServoDriver();

    void Attach(int pin, ledc_channel_t channel, bool rev = false);
    void Detach();
    void SetTrim(int trim) { trim_ = trim; }
    void SetPosition(int position);
    void SetAngle(int angle);
    int GetPosition() { return pos_; }

private:
    void Write(int position);

    bool is_attached_;
    int pos_;
    int pin_;
    int trim_;
    bool rev_;
    int diff_limit_;
    long previous_servo_command_millis_;

    ledc_channel_t ledc_channel_;
    ledc_mode_t ledc_speed_mode_;
};

#endif
