#include "servo_driver.h"

#include <driver/ledc.h>
#include <esp_timer.h>
#include <inttypes.h>

#include <algorithm>

static const char* TAG = "ServoDriver";

static unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

ServoDriver::ServoDriver() {
    trim_ = 0;
    diff_limit_ = 0;
    is_attached_ = false;
    rev_ = false;
    pos_ = 90;
    previous_servo_command_millis_ = 0;
}

ServoDriver::~ServoDriver() {
    Detach();
}

void ServoDriver::Attach(int pin, ledc_channel_t channel, bool rev) {
    if (is_attached_) {
        Detach();
    }

    pin_ = pin;
    rev_ = rev;
    ledc_channel_ = channel;
    ledc_speed_mode_ = LEDC_LOW_SPEED_MODE;

    static bool timer_configured = false;
    if (!timer_configured) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_13_BIT,
            .timer_num = LEDC_TIMER_2,
            .freq_hz = 50,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
        timer_configured = true;
        ESP_LOGI(TAG, "LEDC timer 2 configured at 50Hz");
    }

    ledc_channel_config_t ledc_channel_cfg = {
        .gpio_num = pin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ledc_channel_,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cfg));

    previous_servo_command_millis_ = millis();
    is_attached_ = true;
}

void ServoDriver::Detach() {
    if (!is_attached_)
        return;

    ESP_ERROR_CHECK(ledc_stop(ledc_speed_mode_, ledc_channel_, 0));
    is_attached_ = false;
}

void ServoDriver::SetPosition(int position) {
    Write(position);
}

void ServoDriver::SetAngle(int angle) {
    Write(angle);
}

void ServoDriver::Write(int position) {
    if (!is_attached_)
        return;

    long currentMillis = millis();
    if (diff_limit_ > 0) {
        int limit = std::max(
            1, (((int)(currentMillis - previous_servo_command_millis_)) * diff_limit_) / 1000);
        if (abs(position - pos_) > limit) {
            pos_ += position < pos_ ? -limit : limit;
        } else {
            pos_ = position;
        }
    } else {
        pos_ = position;
    }
    previous_servo_command_millis_ = currentMillis;

    int angle = pos_ + trim_;
    angle = std::min(std::max(angle, 0), 180);

    uint32_t duty = (uint32_t)(((angle / 180.0) * 2.0 + 0.5) * 8191 / 20.0);

    ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(ledc_speed_mode_, ledc_channel_, duty, 0));
    ESP_ERROR_CHECK(ledc_update_duty(ledc_speed_mode_, ledc_channel_));
    ESP_ERROR_CHECK(ledc_bind_channel_timer(ledc_speed_mode_, ledc_channel_, LEDC_TIMER_2));

    ESP_LOGI(TAG, "GPIO%d ch%d angle=%d duty=%" PRIu32,
             pin_, ledc_channel_, angle, duty);
}

void ServoDriver::ReinitChannel() {
    if (!is_attached_) return;

    int angle = pos_ + trim_;
    angle = std::min(std::max(angle, 0), 180);
    uint32_t duty = (uint32_t)(((angle / 180.0) * 2.0 + 0.5) * 8191 / 20.0);

    ledc_channel_config_t ledc_channel_cfg = {
        .gpio_num = pin_,
        .speed_mode = ledc_speed_mode_,
        .channel = ledc_channel_,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = duty,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cfg));
    ESP_LOGI(TAG, "Reinit GPIO%d ch%d duty=%" PRIu32, pin_, ledc_channel_, duty);
}

