#include "servo_controller.h"
#include "servo_driver.h"
#include "mcp_server.h"
#include "application.h"

#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

static const char* TAG = "ServoController";

// Servo mapping (verified):
//   servo[0] = GPIO11, ch1 = 右脚: 60°=脚尖抬起, 90°=平放, 120°=脚尖压下
//   servo[1] = GPIO12, ch2 = 右腿: 60°=内旋, 90°=中立, 120°=外旋展髋
//   servo[2] = GPIO13, ch3 = 左腿: 60°=外旋展髋, 90°=中立, 120°=内旋
//   servo[3] = GPIO14, ch4 = 左脚: 60°=脚尖压下, 90°=平放, 120°=脚尖抬起
//   Note: 左腿/左脚与右腿/右脚的角度方向相反
static const int kServoGpios[4] = {11, 12, 13, 14};
static const ledc_channel_t kServoChannels[4] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

class ServoController {
public:
    ServoController() {
        for (int i = 0; i < 4; i++) {
            servos_[i].Attach(kServoGpios[i], kServoChannels[i]);
            servos_[i].SetPosition(90);
        }

        RegisterMcpTools();
    }

    void ReinitPwm() {
        for (int i = 0; i < 4; i++) {
            servos_[i].ReinitChannel();
        }
    }

    ~ServoController() {
        for (int i = 0; i < 4; i++) {
            servos_[i].Detach();
        }
    }

private:
    ServoDriver servos_[4];

    void MoveServo(int servo_id, int angle) {
        if (servo_id < 1 || servo_id > 4) return;
        angle = std::min(std::max(angle, 0), 180);
        servos_[servo_id - 1].SetPosition(angle);
    }

    void Stand() {
        ESP_LOGI(TAG, "Standing");
        for (int i = 0; i < 4; i++) {
            servos_[i].SetPosition(90);
        }
    }

    // Otto-style walk (from OttoDIYLib).
    // Otto servo order: 0=LL, 1=RL, 2=LF, 3=RF
    // Ours:              0=RF, 1=RL, 2=LL, 3=LF
    //
    // A[4] = {30, 30, 20, 20}  — legs amp 30, feet amp 20
    // O[4] = {0, 0, 4, -4}     — tip-toe foot offset
    // phase_diff[4] = {0, 0, -90°, -90°}  — legs in phase, feet trail by 90°
    void Walk(int steps, int speed) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);

        const int kAmpLeg  = 30;  // Otto: leg amplitude
        const int kAmpFoot = 20;  // Otto: foot amplitude
        const int kOffsetLF = 4;   // Otto: left foot tip-toe
        const int kOffsetRF = -4;  // Otto: right foot tip-toe
        const int interval_ms = 15;
        int samples = period / interval_ms;
        if (samples < 20) samples = 20;
        const float kPi = 3.14159265f;
        float dphase = 2.0f * kPi / samples;

        ESP_LOGI(TAG, "Walking %d steps, period=%dms, samples=%d", steps, period, samples);

        auto setPose = [this](float leg_sin, float foot_sin) {
            servos_[0].SetPosition(90 + kOffsetRF + (int)(kAmpFoot * foot_sin));
            servos_[1].SetPosition(90 + (int)(kAmpLeg * leg_sin));
            servos_[2].SetPosition(90 + (int)(kAmpLeg * leg_sin));
            servos_[3].SetPosition(90 + kOffsetLF + (int)(kAmpFoot * foot_sin));
        };

        // Ramp into oscillation
        for (int i = 0; i < samples; i++) {
            float phase = i * dphase;
            float ramp = (float)i / samples;
            setPose(ramp * std::sin(phase), ramp * std::sin(phase + kPi / 2.0f));
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }

        // Full oscillation for N cycles
        for (int s = 0; s < steps; s++) {
            for (int i = 0; i < samples; i++) {
                float phase = i * dphase;
                setPose(std::sin(phase), std::sin(phase + kPi / 2.0f));
                vTaskDelay(pdMS_TO_TICKS(interval_ms));
            }
        }

        // Ramp out of oscillation
        for (int i = 0; i < samples; i++) {
            float phase = i * dphase;
            float ramp = 1.0f - (float)i / samples;
            setPose(ramp * std::sin(phase), ramp * std::sin(phase + kPi / 2.0f));
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }

        // Ensure standing
        for (int i = 0; i < 4; i++) {
            servos_[i].SetPosition(90);
        }
    }

    void RegisterMcpTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool(
            "self.servo.test",
            "Test a single servo for identification. servo_id: 1-4, angle: 0-180 degrees.",
            PropertyList({
                Property("servo_id", kPropertyTypeInteger, 1, 4),
                Property("angle", kPropertyTypeInteger, 90, 0, 180),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int servo_id = properties["servo_id"].value<int>();
                int angle = properties["angle"].value<int>();
                ESP_LOGI(TAG, "Test servo %d -> %d degrees", servo_id, angle);
                Application::GetInstance().Schedule([this, servo_id, angle]() {
                    MoveServo(servo_id, angle);
                });
                return true;
            });

        mcp.AddTool(
            "self.servo.move",
            "Move a single servo to a specified angle. servo_id: 1-4, angle: 0-180.",
            PropertyList({
                Property("servo_id", kPropertyTypeInteger, 1, 4),
                Property("angle", kPropertyTypeInteger, 90, 0, 180),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int servo_id = properties["servo_id"].value<int>();
                int angle = properties["angle"].value<int>();
                ESP_LOGI(TAG, "Move servo %d -> %d degrees", servo_id, angle);
                Application::GetInstance().Schedule([this, servo_id, angle]() {
                    MoveServo(servo_id, angle);
                });
                return true;
            });

        mcp.AddTool(
            "self.servo.stand",
            "Reset all servos to standing position (90 degrees).",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                Application::GetInstance().Schedule([this]() {
                    Stand();
                });
                return true;
            });

        mcp.AddTool(
            "self.servo.walk",
            "Sinusoidal side-sway walk (Otto-style). All 4 servos oscillate on a shared sine wave. steps: 1-10, speed: 200-2000ms per cycle (smaller=faster).",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                std::thread([this, steps, speed]() {
                    Walk(steps, speed);
                }).detach();
                return true;
            });
    }
};

static ServoController* g_servo_controller = nullptr;

void InitializeServoController() {
    if (g_servo_controller == nullptr) {
        g_servo_controller = new ServoController();
        ESP_LOGI(TAG, "Servo controller initialized");
    }
}

void ReinitServoPwm() {
    if (g_servo_controller) {
        ESP_LOGI(TAG, "Reinitializing servo PWM channels...");
        g_servo_controller->ReinitPwm();
        ESP_LOGI(TAG, "Servo PWM reinit complete");
    }
}
