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
//
// Otto servo order: 0=LL, 1=RL, 2=LF, 3=RF
// Our order:        0=RF, 1=RL, 2=LL, 3=LF
//
// Direction correction: Otto forward uses feet phase -90° (feet trail legs).
// Our mechanics require +90° (feet lead legs) for forward.
// So we flip the sign of all foot phase_diff values vs Otto.

static const int kServoGpios[4] = {11, 12, 13, 14};
static const ledc_channel_t kServoChannels[4] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

#define DEG2RAD(g) ((g)*3.14159265f / 180.0f)

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
        for (int i = 0; i < 4; i++) {
            servos_[i].SetPosition(90);
        }
    }

    // Run sinusoidal oscillation for N steps with ramp in/out.
    // Arrays are in OUR servo order: [RF, RL, LL, LF].
    void RunOscillation(int steps, int period,
                        const int amp[4], const int offset[4], const float phase[4]) {
        const int interval_ms = 15;
        int samples = period / interval_ms;
        if (samples < 20) samples = 20;
        const float kPi = 3.14159265f;
        float dphase = 2.0f * kPi / samples;

        // Ramp in
        for (int i = 0; i < samples; i++) {
            float p = i * dphase;
            float ramp = (float)i / samples;
            for (int j = 0; j < 4; j++)
                servos_[j].SetPosition(90 + offset[j] + (int)(ramp * amp[j] * std::sin(p + phase[j])));
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }

        // Full oscillation
        for (int s = 0; s < steps; s++) {
            for (int i = 0; i < samples; i++) {
                float p = i * dphase;
                for (int j = 0; j < 4; j++)
                    servos_[j].SetPosition(90 + offset[j] + (int)(amp[j] * std::sin(p + phase[j])));
                vTaskDelay(pdMS_TO_TICKS(interval_ms));
            }
        }

        // Ramp out
        for (int i = 0; i < samples; i++) {
            float p = i * dphase;
            float ramp = 1.0f - (float)i / samples;
            for (int j = 0; j < 4; j++)
                servos_[j].SetPosition(90 + offset[j] + (int)(ramp * amp[j] * std::sin(p + phase[j])));
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }

        Stand();
    }

    // Otto: A={30,30,30,30}, O={0,0,5,-5}, phase={0,0,-90°*dir,-90°*dir}
    // Our forward needs +90° for feet (flip sign).
    void Walk(int steps, int speed, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);

        int amp[4] = {30, 30, 30, 30};
        int offset[4] = {-5, 0, 0, 5};
        float phase[4] = {
            DEG2RAD(dir * 90),   // RF: foot
            DEG2RAD(0),          // RL: leg
            DEG2RAD(0),          // LL: leg
            DEG2RAD(dir * 90),   // LF: foot
        };

        ESP_LOGI(TAG, "Walking %d steps, period=%dms, dir=%d", steps, period, dir);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Turn: directly mapped from Otto.
    // Otto LEFT  (dir= 1): LL=30, RL= 0  — left leg swings, pivots around right leg
    // Otto RIGHT (dir=-1): LL= 0, RL=30  — right leg swings, pivots around left leg
    //
    // IMPORTANT: turn uses Otto's original foot phase (-90°) instead of our
    // walk's flipped +90°.  Only one leg is swinging, so the foot must press
    // (grip) when the swinging leg passes through neutral to provide a pivot.
    // With +90° timing the foot lifts at neutral and the robot can't turn.
    void Turn(int steps, int speed, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);

        // Our order: [RF, RL, LL, LF]
        // LEFT  (dir= 1): LL=30, RL= 0  → amp[1]=0,  amp[2]=30
        // RIGHT (dir=-1): RL=30, LL= 0  → amp[1]=30, amp[2]=0
        int amp[4] = {30, 0, 30, 30};   // LEFT: left leg swinging
        if (dir == -1) {                 // RIGHT: right leg swinging
            amp[1] = 30;
            amp[2] = 0;
        }

        int offset[4] = {-5, 0, 0, 5};
        float phase[4] = {
            DEG2RAD(-90),  // RF: foot (Otto original, foot presses at leg neutral)
            DEG2RAD(0),    // RL: leg
            DEG2RAD(0),    // LL: leg
            DEG2RAD(-90),  // LF: foot (Otto original)
        };

        ESP_LOGI(TAG, "Turning %d steps, period=%dms, dir=%d", steps, period, dir);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto UpDown: A={0,0,h,h}, O={0,0,h,-h}, phase={0,0,-90°,90°}
    // Feet 180° out of phase, legs still. Bouncing motion.
    void Bounce(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height, 0, 0, height};
        float phase[4] = {
            DEG2RAD(90),    // RF: foot starts at extreme
            DEG2RAD(0),     // RL
            DEG2RAD(0),     // LL
            DEG2RAD(-90),   // LF: foot 180° out of phase
        };

        ESP_LOGI(TAG, "Bouncing %d steps, period=%dms, h=%d", steps, period, height);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto Swing: A={0,0,h,h}, O={0,0,h/2,-h/2}, phase={0,0,0,0}
    // Feet in phase, offset half amplitude. Side sway.
    void Sway(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height / 2, 0, 0, height / 2};
        float phase[4] = {0, 0, 0, 0};

        ESP_LOGI(TAG, "Swaying %d steps, period=%dms, h=%d", steps, period, height);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto TiptoeSwing: A={0,0,h,h}, O={0,0,h,-h}, phase={0,0,0,0}
    // Like swing but full offset keeps heels up.
    void Tiptoe(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height, 0, 0, height};
        float phase[4] = {0, 0, 0, 0};

        ESP_LOGI(TAG, "Tiptoe %d steps, period=%dms, h=%d", steps, period, height);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto Jitter: A={h,h,0,0}, O={0,0,0,0}, phase={-90°,90°,0,0}, h≤25
    // Legs 180° out of phase, feet still. Leg shake.
    void Shake(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(height, 25);

        int amp[4] = {0, height, height, 0};
        int offset[4] = {0, 0, 0, 0};
        float phase[4] = {
            DEG2RAD(0),     // RF: foot still
            DEG2RAD(90),    // RL: leg
            DEG2RAD(-90),   // LL: leg 180° out of phase
            DEG2RAD(0),     // LF: foot still
        };

        ESP_LOGI(TAG, "Shaking %d steps, period=%dms, h=%d", steps, period, height);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto Moonwalker: A={0,0,h,h}, O={0,0,h/2+2,-h/2-2}
    // phi=-dir*90, phase={0,0,phi,-60*dir+phi}
    // Feet only, traveling wave at 60° offset. dir: 1=left, -1=right.
    void Moonwalk(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        float phi = DEG2RAD(-dir * 90);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height / 2 - 2, 0, 0, height / 2 + 2};
        float phase[4] = {
            DEG2RAD(-60 * dir) + phi,  // RF
            DEG2RAD(0),                 // RL
            DEG2RAD(0),                 // LL
            phi,                        // LF
        };

        ESP_LOGI(TAG, "Moonwalk %d steps, period=%dms, h=%d, dir=%d", steps, period, height, dir);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto Crusaito: A={25,25,h,h}, O={0,0,h/2+4,-h/2-4}
    // phase={90°,90°,0,-60°*dir}
    // All 4 servos. dir: 1=forward, -1=backward.
    void Crusaito(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 25, 25, height};
        int offset[4] = {-height / 2 - 4, 0, 0, height / 2 + 4};
        float phase[4] = {
            DEG2RAD(-60 * dir),  // RF: foot (Otto original, no flip needed)
            DEG2RAD(90),         // RL: leg
            DEG2RAD(90),         // LL: leg
            DEG2RAD(0),          // LF: foot
        };

        ESP_LOGI(TAG, "Crusaito %d steps, period=%dms, h=%d, dir=%d", steps, period, height, dir);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto Flapping: A={12,12,h,h}, O={0,0,h-10,-h+10}
    // phase={0,180°,-90°*dir,90°*dir}
    // Legs 180° out of phase (flapping), feet counter-phase.
    // dir: 1=forward, -1=backward.
    void Flap(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 12, 12, height};
        int offset[4] = {-height + 10, 0, 0, height - 10};
        float phase[4] = {
            DEG2RAD(90 * dir),    // RF: foot (flipped from Otto's 90*dir)
            DEG2RAD(180),         // RL: leg
            DEG2RAD(0),           // LL: leg
            DEG2RAD(-90 * dir),   // LF: foot (flipped from Otto's -90*dir)
        };

        ESP_LOGI(TAG, "Flapping %d steps, period=%dms, h=%d, dir=%d", steps, period, height, dir);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Otto AscendingTurn: A={h,h,h,h}, O={0,0,h+4,-h+4}, phase={-90°,90°,-90°,90°}, h≤13
    // All 4 servos, each pair 180° out of phase.
    void Ascend(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        int period = std::min(std::max(speed, 200), 2000);
        height = std::min(height, 13);

        int amp[4] = {height, height, height, height};
        int offset[4] = {-height + 4, 0, 0, height + 4};
        float phase[4] = {
            DEG2RAD(90),    // RF
            DEG2RAD(90),    // RL
            DEG2RAD(-90),   // LL
            DEG2RAD(-90),   // LF
        };

        ESP_LOGI(TAG, "Ascending %d steps, period=%dms, h=%d", steps, period, height);
        RunOscillation(steps, period, amp, offset, phase);
    }

    // Two right turns in a row = 180° turn around
    void TurnAround(int steps, int speed) {
        ESP_LOGI(TAG, "Turning around (2x right turn) %d steps", steps);
        Turn(steps, speed, -1);
        Turn(steps, speed, -1);
    }

    // Stand at ease: left foot (servo 4) turns 30° to 120° (toes up)
    void Rest() {
        ESP_LOGI(TAG, "Resting (left foot up)");
        servos_[3].SetPosition(120);
    }

    void DispatchAction(std::function<void()> action) {
        std::thread([action]() {
            action();
        }).detach();
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
            "self.servo.rest",
            "Stand at ease: left foot (servo 4) rotates 30° to 120° for a relaxed pose.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                Application::GetInstance().Schedule([this]() {
                    Rest();
                });
                return true;
            });

        // -- Locomotion --

        mcp.AddTool(
            "self.servo.walk",
            "Otto-style sinusoidal walk. steps: 1-10, speed: 200-2000ms per cycle (smaller=faster), direction: 1=forward, -1=backward.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("direction", kPropertyTypeInteger, 1, -1, 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int dir = properties["direction"].value<int>();
                DispatchAction([this, steps, speed, dir]() { Walk(steps, speed, dir); });
                return true;
            });

        mcp.AddTool(
            "self.servo.turn",
            "Turn in place by driving one leg more than the other. steps: 1-10, speed: 200-2000ms, direction: 1=left, -1=right.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("direction", kPropertyTypeInteger, 1, -1, 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int dir = properties["direction"].value<int>();
                DispatchAction([this, steps, speed, dir]() { Turn(steps, speed, dir); });
                return true;
            });

        mcp.AddTool(
            "self.servo.turn_around",
            "Turn 180 degrees around (two consecutive right turns). steps: 1-10, speed: 200-2000ms.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                DispatchAction([this, steps, speed]() { TurnAround(steps, speed); });
                return true;
            });

        // -- Feet-only motions --

        mcp.AddTool(
            "self.servo.bounce",
            "Bounce up and down (feet 180° out of phase, legs still). steps: 1-10, speed: 200-2000ms, height: 5-30.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                DispatchAction([this, steps, speed, height]() { Bounce(steps, speed, height); });
                return true;
            });

        mcp.AddTool(
            "self.servo.sway",
            "Side-to-side sway (feet in phase, legs still). steps: 1-10, speed: 200-2000ms, height: 5-30.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                DispatchAction([this, steps, speed, height]() { Sway(steps, speed, height); });
                return true;
            });

        mcp.AddTool(
            "self.servo.tiptoe",
            "Tiptoe side-to-side swing (heels stay up). steps: 1-10, speed: 200-2000ms, height: 5-30.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                DispatchAction([this, steps, speed, height]() { Tiptoe(steps, speed, height); });
                return true;
            });

        // -- Leg-only motion --

        mcp.AddTool(
            "self.servo.shake",
            "Shake legs rapidly (legs 180° out of phase, feet still). steps: 1-10, speed: 200-2000ms, height: 5-25.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 25),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                DispatchAction([this, steps, speed, height]() { Shake(steps, speed, height); });
                return true;
            });

        // -- Directional special motions --

        mcp.AddTool(
            "self.servo.moonwalk",
            "Moonwalk slide (feet-only traveling wave). steps: 1-10, speed: 200-2000ms, height: 5-30, direction: 1=left, -1=right.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
                Property("direction", kPropertyTypeInteger, 1, -1, 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                int dir = properties["direction"].value<int>();
                DispatchAction([this, steps, speed, height, dir]() { Moonwalk(steps, speed, height, dir); });
                return true;
            });

        mcp.AddTool(
            "self.servo.crusaito",
            "Cross-step sideways walk like a crab (all 4 servos moving together). steps: 1-10, speed: 200-2000ms, height: 5-30, direction: 1=forward, -1=backward.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
                Property("direction", kPropertyTypeInteger, 1, -1, 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                int dir = properties["direction"].value<int>();
                DispatchAction([this, steps, speed, height, dir]() { Crusaito(steps, speed, height, dir); });
                return true;
            });

        mcp.AddTool(
            "self.servo.flap",
            "Flapping motion (legs 180° out of phase like wings, feet counter-phase). steps: 1-10, speed: 200-2000ms, height: 5-30, direction: 1=forward, -1=backward.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 15, 5, 30),
                Property("direction", kPropertyTypeInteger, 1, -1, 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                int dir = properties["direction"].value<int>();
                DispatchAction([this, steps, speed, height, dir]() { Flap(steps, speed, height, dir); });
                return true;
            });

        mcp.AddTool(
            "self.servo.ascend",
            "Compact wiggling turn (all 4 servos oscillate in small range, bot twists in place). steps: 1-10, speed: 200-2000ms, height: 5-13.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 2, 1, 10),
                Property("speed", kPropertyTypeInteger, 800, 200, 2000),
                Property("height", kPropertyTypeInteger, 10, 5, 13),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int height = properties["height"].value<int>();
                DispatchAction([this, steps, speed, height]() { Ascend(steps, speed, height); });
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
