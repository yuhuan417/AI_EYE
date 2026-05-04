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
#include <thread>
#include <vector>

static const char* TAG = "ServoController";

// GPIO 11,12,13,14 -> channels 1,2,3,4
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

    void Walk(int steps, int speed) {
        steps = std::min(std::max(steps, 1), 10);
        int delay_ms = std::min(std::max(speed, 200), 2000);

        ESP_LOGI(TAG, "Walking %d steps, delay=%dms", steps, delay_ms);

        for (int s = 0; s < steps; s++) {
            // Step 1: lean right, lift left foot
            servos_[1].SetPosition(70);  // left leg forward
            servos_[3].SetPosition(110); // left foot up
            vTaskDelay(pdMS_TO_TICKS(delay_ms));

            // Step 2: place left foot, shift weight left
            servos_[1].SetPosition(90);
            servos_[3].SetPosition(90);
            vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));

            // Step 3: lean left, lift right foot
            servos_[0].SetPosition(110); // right leg forward
            servos_[2].SetPosition(70);  // right foot up
            vTaskDelay(pdMS_TO_TICKS(delay_ms));

            // Step 4: place right foot, center
            servos_[0].SetPosition(90);
            servos_[2].SetPosition(90);
            vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));
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
            "Walk forward with bipedal gait. steps: 1-10, speed: 200-2000ms (smaller=faster).",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                // Walk runs in a background thread since it uses vTaskDelay
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
