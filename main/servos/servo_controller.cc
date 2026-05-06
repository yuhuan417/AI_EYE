#include "servo_controller.h"

static ServoController* g_servo_controller = nullptr;

void InitializeServoController() {
    if (g_servo_controller == nullptr) {
        g_servo_controller = new ServoController();
        ESP_LOGI(TAG, "Servo controller initialized");
    }
}

bool IsMusicPlaying() {
    return music_playing_;
}
