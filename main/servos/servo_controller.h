#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "servo_driver.h"
#include "mcp_server.h"
#include "application.h"
#include "board.h"
#include "settings.h"

#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_pthread.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

#include "smooth_criminal_mp3.h"
#include "single_ladies_mp3.h"
#include <mp3dec.h>

#define DEG2RAD(g) ((g)*3.14159265f / 180.0f)

void InitializeServoController();
void ReinitServoPwm();
bool IsMusicPlaying();

static const char* TAG = "ServoController";
static std::atomic<bool> music_playing_{false};
static constexpr int kServoCount = 4;
static const int kServoGpios[kServoCount] = {11, 12, 13, 14};
static const ledc_channel_t kServoChannels[kServoCount] = {
    LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};
static const int kDefaultServoTrims[kServoCount] = {5, 0, 0, 10};
static const char* const kServoTrimKeys[kServoCount] = {
    "trim_1", "trim_2", "trim_3", "trim_4"};
static const char* const kServoNames[kServoCount] = {"RF", "RL", "LL", "LF"};
static const char* const kServoCalibrationNamespace = "servo";
static const char* const kServoCalibrationFlagKey = "has_calibration";

class TrimmedServoDriver {
public:
    void Attach(int pin, ledc_channel_t channel, int trim, bool rev = false) {
        trim_ = trim;
        logical_pos_ = 90;
        driver_.Attach(pin, channel, rev);
    }

    void Detach() {
        driver_.Detach();
    }

    void SetTrim(int trim) {
        int physical_pos = GetPhysicalPosition();
        trim_ = trim;
        logical_pos_ = std::min(std::max(physical_pos - trim_, 0), 180);
        driver_.SetPosition(logical_pos_ + trim_);
    }

    void SetPosition(int position) {
        logical_pos_ = std::min(std::max(position, 0), 180);
        driver_.SetPosition(logical_pos_ + trim_);
    }

    void SetAngle(int angle) {
        SetPosition(angle);
    }

    int GetPosition() const {
        return logical_pos_;
    }

    int GetTrim() const {
        return trim_;
    }

    int GetPhysicalPosition() const {
        return std::min(std::max(logical_pos_ + trim_, 0), 180);
    }

private:
    ServoDriver driver_;
    int trim_ = 0;
    int logical_pos_ = 90;
};

class ServoController {
public:
    ServoController() {
        LoadServoCalibration();

        for (int i = 0; i < kServoCount; i++) {
            servos_[i].Attach(kServoGpios[i], kServoChannels[i], servo_trims_[i]);
            servos_[i].SetPosition(90);
            old_pos_[i] = 90;
        }

        ESP_LOGI(TAG, "Servo calibration loaded: %s", FormatServoCalibration().c_str());
        RegisterMcpTools();
    }

    ~ServoController() {
        for (int i = 0; i < kServoCount; i++) {
            servos_[i].Detach();
        }
    }

private:
    TrimmedServoDriver servos_[kServoCount];
    int servo_trims_[kServoCount] = {0};
    int old_pos_[kServoCount];
    bool servo_trims_loaded_from_nvs_ = false;
    std::atomic<bool> stop_requested_{false};

    bool ShouldStop() {
        if (stop_requested_.exchange(false)) {
            Stand();
            return true;
        }
        return false;
    }

    static void MusicTask(void* arg) ;

    void PlayMusic() ;

    static void MusicTaskSl(void* arg) ;

    void PlaySingleLadies() ;

    void MoveServo(int servo_id, int angle) {
        if (servo_id < 1 || servo_id > kServoCount) return;
        angle = std::min(std::max(angle, 0), 180);
        servos_[servo_id - 1].SetPosition(angle);
        old_pos_[servo_id - 1] = angle;
    }

    void Stand() {
        for (int i = 0; i < kServoCount; i++) {
            servos_[i].SetPosition(90);
            old_pos_[i] = 90;
        }
    }

    void LoadServoCalibration() {
        servo_trims_loaded_from_nvs_ = false;
        for (int i = 0; i < kServoCount; i++) {
            servo_trims_[i] = kDefaultServoTrims[i];
        }

        Settings settings(kServoCalibrationNamespace, false);
        if (settings.GetInt(kServoCalibrationFlagKey, 0) == 0) {
            return;
        }

        servo_trims_loaded_from_nvs_ = true;
        for (int i = 0; i < kServoCount; i++) {
            servo_trims_[i] = settings.GetInt(kServoTrimKeys[i], kDefaultServoTrims[i]);
        }
    }

    void SaveServoCalibration() {
        Settings settings(kServoCalibrationNamespace, true);
        settings.SetInt(kServoCalibrationFlagKey, 1);
        for (int i = 0; i < kServoCount; i++) {
            settings.SetInt(kServoTrimKeys[i], servo_trims_[i]);
        }
        servo_trims_loaded_from_nvs_ = true;
    }

    std::string FormatServoCalibration() const {
        std::string result = "{\"source\":\"";
        result += servo_trims_loaded_from_nvs_ ? "nvs" : "default";
        result += "\",\"order\":[";
        for (int i = 0; i < kServoCount; i++) {
            if (i > 0) {
                result += ",";
            }
            result += "\"";
            result += kServoNames[i];
            result += "\"";
        }
        result += "],\"trims\":[";
        for (int i = 0; i < kServoCount; i++) {
            if (i > 0) {
                result += ",";
            }
            result += std::to_string(servo_trims_[i]);
        }
        result += "],\"neutral\":[";
        for (int i = 0; i < kServoCount; i++) {
            if (i > 0) {
                result += ",";
            }
            result += std::to_string(90 + servo_trims_[i]);
        }
        result += "]}";
        return result;
    }

    std::string SaveCurrentPoseAsNeutral() {
        for (int i = 0; i < kServoCount; i++) {
            servo_trims_[i] = servos_[i].GetPhysicalPosition() - 90;
        }
        for (int i = 0; i < kServoCount; i++) {
            servos_[i].SetTrim(servo_trims_[i]);
        }
        SaveServoCalibration();
        Stand();
        ESP_LOGI(TAG, "Servo calibration saved: %s", FormatServoCalibration().c_str());
        return FormatServoCalibration();
    }

    // Otto-style oscillation: sample the sine wave every 30ms, but interpolate
    // all 4 servos across that sample window instead of jumping once per sample.
    // `phase_bias` lets a motion enter the gait from a more stable point in the
    // cycle, and `preload_to_start` moves there smoothly before oscillation.
    void OttoOscillate(int steps, int T, const int amp[4],
                       const int offset[4], const float phase[4],
                       float phase_bias = 0.0f,
                       bool preload_to_start = false) {
        const int kSampleMs = 30;
        int samples = T / kSampleMs;
        if (samples < 1) samples = 1;
        const float kPi = 3.14159265f;
        float dphase = 2.0f * kPi / samples;
        int positions[4];

        if (preload_to_start) {
            for (int j = 0; j < kServoCount; j++) {
                positions[j] =
                    90 + offset[j] + (int)std::lround(amp[j] * std::sin(phase_bias + phase[j]));
            }
            MoveNServos(std::max(kSampleMs * 3, T / 6), positions);
        }

        for (int s = 0; s < steps; s++) {
            for (int i = 0; i < samples; i++) {
                if (stop_requested_) {
                    Stand();
                    return;
                }

                float p = phase_bias + i * dphase;
                for (int j = 0; j < kServoCount; j++) {
                    positions[j] =
                        90 + offset[j] + (int)std::lround(amp[j] * std::sin(p + phase[j]));
                }
                MoveNServos(kSampleMs, positions);
            }
        }

        Stand();
    }

    // Otto: A={30,30,30,30}, O={0,0,5,-5}, phase={0,0,-90°*dir,-90°*dir}
    // Feet phase is flipped relative to Otto because our foot installation is mirrored.
    void Walk(int steps, int speed, int dir) {
        steps = std::min(std::max(steps, 1), 10);

        int amp[4] = {30, 30, 30, 30};
        int offset[4] = {-5, 0, 0, 5};
        float phase[4] = {
            DEG2RAD(dir * 90),   // RF: foot
            DEG2RAD(0),          // RL: leg
            DEG2RAD(0),          // LL: leg
            DEG2RAD(dir * 90),   // LF: foot
        };

        ESP_LOGI(TAG, "Walking %d steps, period=%dms, dir=%d", steps, speed, dir);
        if (dir > 0) {
            // Forward: rely on the global left-foot trim and only keep the
            // stabilized gait entry phase.
            OttoOscillate(steps, speed, amp, offset, phase, DEG2RAD(90), true);
            return;
        }
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Turn: directly mapped from Otto.
    // Otto LEFT  (dir= 1): LL=30, RL= 0  — left leg swings, pivots around right leg
    // Otto RIGHT (dir=-1): LL= 0, RL=30  — right leg swings, pivots around left leg
    //
    void Turn(int steps, int speed, int dir) {
        steps = std::min(std::max(steps, 1), 10);

        // Our order: [RF, RL, LL, LF]
        // LEFT  (dir= 1): left leg swinging
        // RIGHT (dir=-1): right leg swinging
        int amp[4] = {30, 0, 30, 30};   // LEFT: left leg swinging
        float foot_phase = DEG2RAD(-90);
        float phase_bias = DEG2RAD(-90);
        if (dir == -1) {                 // RIGHT: right leg swinging
            amp[1] = 30;
            amp[2] = 0;
        }

        int offset[4] = {-5, 0, 0, 5};
        float phase[4] = {
            foot_phase,    // RF: foot
            DEG2RAD(0),    // RL: leg
            DEG2RAD(0),    // LL: leg
            foot_phase,    // LF: foot
        };

        ESP_LOGI(TAG, "Turning %d steps, period=%dms, dir=%d", steps, speed, dir);
        OttoOscillate(steps, speed, amp, offset, phase, phase_bias, true);
    }

    // Otto UpDown: A={0,0,h,h}, O={0,0,h,-h}, phase={0,0,-90°,90°}
    // Feet 180° out of phase, legs still. Bouncing motion.
    void Bounce(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height, 0, 0, height};
        float phase[4] = {
            DEG2RAD(90),    // RF: foot starts at extreme
            DEG2RAD(0),     // RL
            DEG2RAD(0),     // LL
            DEG2RAD(-90),   // LF: foot 180° out of phase
        };

        ESP_LOGI(TAG, "Bouncing %d steps, period=%dms, h=%d", steps, speed, height);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto Swing: A={0,0,h,h}, O={0,0,h/2,-h/2}, phase={0,0,0,0}
    // Feet in phase, offset half amplitude. Side sway.
    void Sway(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height / 2, 0, 0, height / 2};
        float phase[4] = {0, 0, 0, 0};

        ESP_LOGI(TAG, "Swaying %d steps, period=%dms, h=%d", steps, speed, height);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto TiptoeSwing: A={0,0,h,h}, O={0,0,h,-h}, phase={0,0,0,0}
    // Like swing but full offset keeps heels up.
    void Tiptoe(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 0, 0, height};
        int offset[4] = {-height, 0, 0, height};
        float phase[4] = {0, 0, 0, 0};

        ESP_LOGI(TAG, "Tiptoe %d steps, period=%dms, h=%d", steps, speed, height);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto Jitter: A={h,h,0,0}, O={0,0,0,0}, phase={-90°,90°,0,0}, h≤25
    // Legs 180° out of phase, feet still. Leg shake.
    void Shake(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(height, 25);

        int amp[4] = {0, height, height, 0};
        int offset[4] = {0, 0, 0, 0};
        float phase[4] = {
            DEG2RAD(0),     // RF: foot still
            DEG2RAD(90),    // RL: leg
            DEG2RAD(-90),   // LL: leg 180° out of phase
            DEG2RAD(0),     // LF: foot still
        };

        ESP_LOGI(TAG, "Shaking %d steps, period=%dms, h=%d", steps, speed, height);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto Moonwalker: A={0,0,h,h}, O={0,0,h/2+2,-h/2-2}
    // phi=-dir*90, phase={0,0,phi,-60*dir+phi}
    // Feet only, traveling wave at 60° offset. dir: 1=left, -1=right.
    void Moonwalk(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
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

        ESP_LOGI(TAG, "Moonwalk %d steps, period=%dms, h=%d, dir=%d", steps, speed, height, dir);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto Crusaito: A={25,25,h,h}, O={0,0,h/2+4,-h/2-4}
    // phase={90°,90°,0,-60°*dir}
    // All 4 servos. dir: 1=forward, -1=backward.
    void Crusaito(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 25, 25, height};
        int offset[4] = {-height / 2 - 4, 0, 0, height / 2 + 4};
        float phase[4] = {
            DEG2RAD(-60 * dir),  // RF: foot (Otto original, no flip needed)
            DEG2RAD(90),         // RL: leg
            DEG2RAD(90),         // LL: leg
            DEG2RAD(0),          // LF: foot
        };

        ESP_LOGI(TAG, "Crusaito %d steps, period=%dms, h=%d, dir=%d", steps, speed, height, dir);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto Flapping: A={12,12,h,h}, O={0,0,h-10,-h+10}
    // phase={0,180°,-90°*dir,90°*dir}
    // Legs 180° out of phase (flapping), feet counter-phase.
    // dir: 1=forward, -1=backward.
    void Flap(int steps, int speed, int height, int dir) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(std::max(height, 5), 30);

        int amp[4] = {height, 12, 12, height};
        int offset[4] = {-height + 10, 0, 0, height - 10};
        float phase[4] = {
            DEG2RAD(90 * dir),    // RF: foot (flipped from Otto's 90*dir)
            DEG2RAD(180),         // RL: leg
            DEG2RAD(0),           // LL: leg
            DEG2RAD(-90 * dir),   // LF: foot (flipped from Otto's -90*dir)
        };

        ESP_LOGI(TAG, "Flapping %d steps, period=%dms, h=%d, dir=%d", steps, speed, height, dir);
        OttoOscillate(steps, speed, amp, offset, phase);
    }

    // Otto AscendingTurn: A={h,h,h,h}, O={0,0,h+4,-h+4}, phase={-90°,90°,-90°,90°}, h≤13
    // All 4 servos, each pair 180° out of phase.
    void Ascend(int steps, int speed, int height) {
        steps = std::min(std::max(steps, 1), 10);
        height = std::min(height, 13);

        int amp[4] = {height, height, height, height};
        int offset[4] = {-height + 4, 0, 0, height + 4};
        float phase[4] = {
            DEG2RAD(90),    // RF
            DEG2RAD(90),    // RL
            DEG2RAD(-90),   // LL
            DEG2RAD(-90),   // LF
        };

        ESP_LOGI(TAG, "Ascending %d steps, period=%dms, h=%d", steps, speed, height);
        OttoOscillate(steps, speed, amp, offset, phase);
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

    // --- Keyframe interpolation (Otto Smooth Criminal moveNServos) ---
    void MoveNServos(int time_ms, const int positions[4]) {
        const int kInterval = 10;
        int steps = time_ms / kInterval;
        if (steps < 1) steps = 1;
        int start_pos[4];
        float inc[4];
        for (int i = 0; i < kServoCount; i++) {
            start_pos[i] = servos_[i].GetPosition();
            old_pos_[i] = start_pos[i];
            inc[i] = (float)(positions[i] - start_pos[i]) / steps;
        }

        for (int s = 1; s <= steps; s++) {
            if (stop_requested_) {
                Stand();
                return;
            }
            for (int i = 0; i < kServoCount; i++) {
                servos_[i].SetPosition(start_pos[i] + (int)std::lround(s * inc[i]));
            }
            vTaskDelay(pdMS_TO_TICKS(kInterval));
        }
        for (int i = 0; i < kServoCount; i++) {
            servos_[i].SetPosition(positions[i]);
            old_pos_[i] = positions[i];
        }
    }

    void ResetOldPositions() {
        for (int i = 0; i < kServoCount; i++) old_pos_[i] = 90;
    }

    // -- Smooth Criminal dance moves --
    // t = 495ms per beat at BPM 121.

    void GoingUp(int tempo) ;

    void Drunk(int tempo) ;

    void NoGravity(int tempo) ;

    void KickLeft(int tempo) ;

    void KickRight(int tempo) ;

    void LateralFuerte(bool side, int tempo) {
        for (int i = 0; i < kServoCount; i++) servos_[i].SetPosition(90);
        if (side) servos_[0].SetPosition(40);
        else servos_[3].SetPosition(140);
        vTaskDelay(pdMS_TO_TICKS(tempo / 2));
        servos_[0].SetPosition(90);
        servos_[3].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo / 2));
    }

    void PrimeraParte(int t) ;

    void SegundaParte(int t) ;

    void SmoothCriminal() ;

    // ── Single Ladies dance moves ──
    // SM order: [RF, LF, RL, LL] → Our: [RF, RL, LL, LF]
    // our[0]=sm[0], our[1]=sm[2], our[2]=sm[3], our[3]=sm[1]

    // Oscillation helpers with SL-specific parameters, converted to our order
    void SlWalk(int steps, int T);
    void SlBackyard(int steps, int T);
    void SlRun(int steps, int T);
    void SlMoonWalkLeft(int steps, int T);
    void SlMoonWalkRight(int steps, int T);
    void SlCrusaito(int steps, int T);
    void SlFlapping(int steps, int T);
    void SlSwing(int steps, int T);

    // Keyframe moves — all positions in our order [RF, RL, LL, LF]
    void SlPasitos(int steps, int tempo);

    void SlPatada(int tempo);

    void SlTwist(int steps, int tempo);

    void SlReverencia1(int steps, int tempo);

    void SlReverencia2(int steps, int tempo);

    void SlSaludo(int steps, int tempo);

    void SlUpDown(int steps, int tempo);

    void SlKickLeft(int tempo);

    void SlKickRight(int tempo);

    void SlDrunk(int tempo);

    void SingleLadies() ;

    void DispatchAction(std::function<void()> action) {
        std::thread([action]() {
            action();
        }).detach();
    }

    void RegisterMcpTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool(
            "self.servo.test",
            "测试单个舵机。ID: 1=右脚 2=右腿 3=左腿 4=左脚。脚上下翘，腿水平转。角度0-180，90=中立位。",
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
            "移动单个舵机到指定角度。舵机ID: 1=右脚 2=右腿 3=左腿 4=左脚。脚上下翘，腿水平转。左脚翘起用4号，右脚翘起用1号。角度0-180。",
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
            "self.servo.get_calibration",
            "查看当前舵机中立位校准数据。返回各舵机的微调偏移量和物理中立角度，顺序为[右脚,右腿,左腿,左脚]。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return FormatServoCalibration();
            });

        mcp.AddTool(
            "self.servo.save_current_as_neutral",
            "将当前4个舵机的姿态保存为新的中立位，写入NVS存储，之后每次开机都使用这个中立位。仅在机器人静止且已摆好目标姿态时调用。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return SaveCurrentPoseAsNeutral();
            });

        mcp.AddTool(
            "self.servo.stand",
            "立正：所有舵机回到中立站立姿态（全部90度）。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                Application::GetInstance().Schedule([this]() {
                    Stand();
                });
                return true;
            });

        mcp.AddTool(
            "self.servo.rest",
            "稍息：左脚(4号舵机)翘起30度到120度，呈现稍息的放松站姿。",
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
            "正弦波步态行走。步数1-10，速度200-2000ms/周期（越小越快），方向1=前进 -1=后退。",
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
            "原地转身，一条腿摆动另一条腿当支点。步数1-10，速度200-2000ms，方向1=左转 -1=右转。",
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
            "180度转身（连续两次右转）。步数1-10，速度200-2000ms。",
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
            "上下弹跳（双脚180度反向摆动，腿不动）。步数1-10，速度200-2000ms，高度5-30。",
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
            "左右摇摆（双脚同相摆动，腿不动）。步数1-10，速度200-2000ms，幅度5-30。",
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
            "踮脚左右摇摆（脚跟保持抬起）。步数1-10，速度200-2000ms，幅度5-30。",
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
            "快速抖腿（双腿180度反向摆动，脚不动）。步数1-10，速度200-2000ms，幅度5-25。",
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
            "太空步侧滑（仅双脚行波运动）。步数1-10，速度200-2000ms，幅度5-30，方向1=左滑 -1=右滑。",
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
            "螃蟹步横移（4个舵机协调运动）。步数1-10，速度200-2000ms，幅度5-30，方向1=前进 -1=后退。",
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
            "扑翼步态（双腿如翅膀180度反向摆动，脚反相配合）。步数1-10，速度200-2000ms，幅度5-30，方向1=前进 -1=后退。",
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
            "原地小幅度扭动转身（4舵机同时小幅摆动）。步数1-10，速度200-2000ms，幅度5-13。",
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

        // -- Dance sequences --

        mcp.AddTool(
            "self.servo.stop",
            "立即停止当前所有舵机动作或舞蹈。当用户说停、停下、别动时调用。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                stop_requested_ = true;
                return true;
            });

        mcp.AddTool(
            "self.servo.dance_smooth_criminal",
            "表演迈克尔·杰克逊的Smooth Criminal完整舞蹈，约60秒的机器人编舞。当用户说跳舞、展示才艺、或提到Smooth Criminal时调用。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                DispatchAction([this]() { SmoothCriminal(); });
                return true;
            });

        mcp.AddTool(
            "self.servo.dance_single_ladies",
            "表演碧昂丝的Single Ladies完整舞蹈，约3分钟的机器人编舞。当用户提到Beyonce、Single Ladies、或想换个舞时调用。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                DispatchAction([this]() { SingleLadies(); });
                return true;
            });
    }
};

#endif
