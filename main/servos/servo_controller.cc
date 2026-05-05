#include "servo_controller.h"
#include "servo_driver.h"
#include "mcp_server.h"
#include "application.h"
#include "board.h"

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

static const char* TAG = "ServoController";
static std::atomic<bool> music_playing_{false};

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
    int old_pos_[4];
    std::atomic<bool> stop_requested_{false};

    bool ShouldStop() {
        if (stop_requested_.exchange(false)) {
            Stand();
            return true;
        }
        return false;
    }

    static void MusicTask(void* arg) {
        auto self = (ServoController*)arg;
        auto codec = Board::GetInstance().GetAudioCodec();
        if (!codec) { music_playing_ = false; vTaskDelete(NULL); return; }

        codec->EnableOutput(true);
        music_playing_ = true;

        HMP3Decoder decoder = MP3InitDecoder();
        if (!decoder) { music_playing_ = false; vTaskDelete(NULL); return; }

        unsigned char* ptr = (unsigned char*)smooth_criminal_mp3;
        const unsigned char* end = ptr + smooth_criminal_mp3_len;

        while (!self->stop_requested_ && ptr < end) {
            int bytesLeft = end - ptr;
            short pcm[MAX_NSAMP * MAX_NCHAN];
            int result = MP3Decode(decoder, &ptr, &bytesLeft, pcm, 0);
            if (result != ERR_MP3_NONE) {
                if (result == ERR_MP3_INDATA_UNDERFLOW) break;
                int sync = MP3FindSyncWord(ptr, bytesLeft);
                ptr += (sync > 0) ? sync : 1;
                continue;
            }

            MP3FrameInfo info;
            MP3GetLastFrameInfo(decoder, &info);
            if (info.outputSamps <= 0) continue;

            int ns = info.outputSamps;
            if (info.nChans > 1)
                for (int i = 0; i < ns; i++) pcm[i] = pcm[i * info.nChans];

            std::vector<int16_t> frame(pcm, pcm + ns);
            codec->OutputData(frame);
            vTaskDelay(pdMS_TO_TICKS(ns * 1000 / 16000));
        }

        MP3FreeDecoder(decoder);
        codec->EnableOutput(false);
        music_playing_ = false;
        vTaskDelete(NULL);
    }

    void PlayMusic() {
        xTaskCreate(MusicTask, "music", 8192, this, 5, NULL);
    }

    static void MusicTaskSl(void* arg) {
        auto self = (ServoController*)arg;
        auto codec = Board::GetInstance().GetAudioCodec();
        if (!codec) { music_playing_ = false; vTaskDelete(NULL); return; }

        codec->EnableOutput(true);
        music_playing_ = true;

        HMP3Decoder decoder = MP3InitDecoder();
        if (!decoder) { music_playing_ = false; vTaskDelete(NULL); return; }

        unsigned char* ptr = (unsigned char*)single_ladies_mp3;
        const unsigned char* end = ptr + single_ladies_mp3_len;

        while (!self->stop_requested_ && ptr < end) {
            int bytesLeft = end - ptr;
            short pcm[MAX_NSAMP * MAX_NCHAN];
            int result = MP3Decode(decoder, &ptr, &bytesLeft, pcm, 0);
            if (result != ERR_MP3_NONE) {
                if (result == ERR_MP3_INDATA_UNDERFLOW) break;
                int sync = MP3FindSyncWord(ptr, bytesLeft);
                ptr += (sync > 0) ? sync : 1;
                continue;
            }

            MP3FrameInfo info;
            MP3GetLastFrameInfo(decoder, &info);
            if (info.outputSamps <= 0) continue;

            int ns = info.outputSamps;
            if (info.nChans > 1)
                for (int i = 0; i < ns; i++) pcm[i] = pcm[i * info.nChans];

            std::vector<int16_t> frame(pcm, pcm + ns);
            codec->OutputData(frame);
            vTaskDelay(pdMS_TO_TICKS(ns * 1000 / 16000));
        }

        MP3FreeDecoder(decoder);
        codec->EnableOutput(false);
        music_playing_ = false;
        vTaskDelete(NULL);
    }

    void PlaySingleLadies() {
        xTaskCreate(MusicTaskSl, "music_sl", 8192, this, 5, NULL);
    }

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

    // Pure sinusoidal oscillation — matching Otto's oscillate().
    // No ramp in/out, no period clamp, 30ms sampling (Otto default).
    // Arrays are in OUR servo order: [RF, RL, LL, LF].
    void OttoOscillate(int steps, int T, const int amp[4],
                       const int offset[4], const float phase[4]) {
        const int kSampleMs = 30;
        int samples = T / kSampleMs;
        if (samples < 1) samples = 1;
        const float kPi = 3.14159265f;
        float dphase = 2.0f * kPi / samples;

        for (int s = 0; s < steps; s++) {
            for (int i = 0; i < samples; i++) {
                float p = i * dphase;
                for (int j = 0; j < 4; j++)
                    servos_[j].SetPosition(90 + offset[j] + (int)(amp[j] * std::sin(p + phase[j])));
                vTaskDelay(pdMS_TO_TICKS(kSampleMs));
            }
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

    // --- Keyframe interpolation (Otto Smooth Criminal moveNServos) ---
    void MoveNServos(int time_ms, const int positions[4]) {
        const int kInterval = 10;
        int steps = time_ms / kInterval;
        if (steps < 1) steps = 1;
        float inc[4];
        for (int i = 0; i < 4; i++)
            inc[i] = (float)(positions[i] - old_pos_[i]) / steps;

        for (int s = 1; s <= steps; s++) {
            for (int i = 0; i < 4; i++)
                servos_[i].SetPosition(old_pos_[i] + (int)(s * inc[i]));
            vTaskDelay(pdMS_TO_TICKS(kInterval));
        }
        for (int i = 0; i < 4; i++) {
            servos_[i].SetPosition(positions[i]);
            old_pos_[i] = positions[i];
        }
    }

    void ResetOldPositions() {
        for (int i = 0; i < 4; i++) old_pos_[i] = 90;
    }

    // -- Smooth Criminal dance moves --
    // t = 495ms per beat at BPM 121.

    void GoingUp(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(83); servos_[3].SetPosition(97);  vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(76); servos_[3].SetPosition(104); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(69); servos_[3].SetPosition(111); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(62); servos_[3].SetPosition(118); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(55); servos_[3].SetPosition(125); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(48); servos_[3].SetPosition(132); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(40); servos_[3].SetPosition(140); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

    void Drunk(int tempo) {
        int m1[4] = {60, 90, 90, 70};
        int m2[4] = {110, 90, 90, 120};
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
    }

    void NoGravity(int tempo) {
        for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
        int m1[4] = {120, 90, 90, 140};
        int m2[4] = {140, 90, 90, 140};
        int m3[4] = {90, 90, 90, 90};
        MoveNServos(tempo * 2, m1);
        MoveNServos(tempo * 2, m2);
        vTaskDelay(pdMS_TO_TICKS(tempo * 2));
        MoveNServos(tempo * 2, m1);
        MoveNServos(tempo * 2, m3);
    }

    void KickLeft(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(50); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(30); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(30); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

    void KickRight(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(110); servos_[3].SetPosition(130); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(110); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(150); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(150); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

    void LateralFuerte(bool side, int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        if (side) servos_[0].SetPosition(40);
        else servos_[3].SetPosition(140);
        vTaskDelay(pdMS_TO_TICKS(tempo / 2));
        servos_[0].SetPosition(90);
        servos_[3].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo / 2));
    }

    void PrimeraParte(int t) {
        int m1[4] = {60, 90, 90, 120};
        int m2[4] = {90, 90, 90, 90};
        int m3[4] = {40, 90, 90, 140};

        for (int x = 0; x < 3; x++) {
            for (int i = 0; i < 3; i++) {
                LateralFuerte(true, t/2);
                LateralFuerte(false, t/4);
                LateralFuerte(true, t/4);
                vTaskDelay(pdMS_TO_TICKS(t));
            }
            for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
            MoveNServos(t * 0.4, m1);
            MoveNServos(t * 0.4, m2);
            vTaskDelay(pdMS_TO_TICKS(t * 2));
        }

        for (int i = 0; i < 2; i++) {
            LateralFuerte(true, t/2);
            LateralFuerte(false, t/4);
            LateralFuerte(true, t/4);
            vTaskDelay(pdMS_TO_TICKS(t));
        }

        for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
        Crusaito(1, t * 1.4, 15, 1);
        MoveNServos(t * 1, m3);
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(t * 4));
    }

    void SegundaParte(int t) {
        int m1[4]  = {90, 80, 100, 90};
        int m2[4]  = {90, 100, 80, 90};
        int m5[4]  = {40, 80, 100, 140};
        int m6[4]  = {40, 100, 80, 140};

        for (int x = 0; x < 7; x++) {
            for (int i = 0; i < 3; i++) {
                MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
                MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
                vTaskDelay(pdMS_TO_TICKS(t));
            }
            MoveNServos(t * 0.15, m5); MoveNServos(t * 0.15, m6);
            MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
            vTaskDelay(pdMS_TO_TICKS(t));
        }

        for (int i = 0; i < 3; i++) {
            MoveNServos(t * 0.15, m5); MoveNServos(t * 0.15, m6);
            MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
            vTaskDelay(pdMS_TO_TICKS(t));
        }
    }

    void SmoothCriminal() {
        int t = 508;  // 118 BPM

        ESP_LOGI(TAG, "Smooth Criminal dance starting...");
        stop_requested_ = false;
        PlayMusic();
        ResetOldPositions();
        Stand();

        PrimeraParte(t);
        if (ShouldStop()) return;
        SegundaParte(t);
        if (ShouldStop()) return;

        Moonwalk(4, t * 2, 30, 1);
        Moonwalk(4, t * 2, 30, -1);
        Moonwalk(4, t * 2, 30, 1);
        Moonwalk(4, t * 2, 30, -1);
        if (ShouldStop()) return;

        PrimeraParte(t);
        if (ShouldStop()) return;

        Crusaito(1, t * 8, 30, 1);
        Crusaito(1, t * 7, 30, 1);
        for (int i = 0; i < 4; i++) {
            if (stop_requested_) break;
            Flap(1, t / 4, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(3 * t / 4));
        }
        if (ShouldStop()) return;

        Moonwalk(4, t * 2, 30, -1);
        Moonwalk(4, t * 2, 30, 1);
        Moonwalk(4, t * 2, 30, -1);
        Moonwalk(4, t * 2, 30, 1);
        if (ShouldStop()) return;

        Drunk(t * 4); Drunk(t * 4); Drunk(t * 4); Drunk(t * 4);
        KickLeft(t); KickRight(t);
        Drunk(t * 8); Drunk(t * 4); Drunk(t / 2);
        vTaskDelay(pdMS_TO_TICKS(t * 4));
        Drunk(t / 2);
        vTaskDelay(pdMS_TO_TICKS(t * 4));
        if (ShouldStop()) return;

        Walk(2, t * 2, 1);
        Walk(2, t * 2, -1);
        if (ShouldStop()) return;

        GoingUp(t * 2); GoingUp(t * 1);
        NoGravity(t * 2);
        if (ShouldStop()) return;

        Crusaito(1, t * 2, 30, 1);  Crusaito(1, t * 8, 30, 1);
        Crusaito(1, t * 2, 30, 1);  Crusaito(1, t * 8, 30, 1);
        if (ShouldStop()) return;

        PrimeraParte(t);
        if (ShouldStop()) return;

        for (int i = 0; i < 28; i++) {
            if (stop_requested_) break;
            Flap(1, t / 2, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(t / 2));
        }

        Stand();
        ESP_LOGI(TAG, "Smooth Criminal dance %s", stop_requested_ ? "interrupted" : "complete");
        stop_requested_ = false;
    }

    // ── Single Ladies dance moves ──
    // SM order: [RF, LF, RL, LL] → Our: [RF, RL, LL, LF]
    // our[0]=sm[0], our[1]=sm[2], our[2]=sm[3], our[3]=sm[1]

    // Oscillation helpers with SL-specific parameters, converted to our order
    void SlWalk(int steps, int T){
        int A[4] = {15, 30, 30, 15};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlBackyard(int steps, int T){
        int A[4] = {15, 30, 30, 15};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlRun(int steps, int T){
        int A[4] = {10, 10, 10, 10};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlMoonWalkLeft(int steps, int T){
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(60)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlMoonWalkRight(int steps, int T){
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(-60)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlCrusaito(int steps, int T){
        int A[4] = {25, 30, 30, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(-60)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlFlapping(int steps, int T){
        int A[4] = {15, 8, 8, 15};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(-90), DEG2RAD(180)};
        OttoOscillate(steps, T, A, O, ph);
    }
    void SlSwing(int steps, int T){
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }

    // Keyframe moves — all positions in our order [RF, RL, LL, LF]
    void SlPasitos(int steps, int tempo){
        int m1[4] = {90, 60, 60, 120};   // SM {90,120,60,60}
        int m2[4] = {90, 90, 90, 90};
        int m3[4] = {60, 120, 120, 90};  // SM {60,90,120,120}
        for (int i = 0; i < steps; i++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            MoveNServos(tempo * 0.25, m1);
            MoveNServos(tempo * 0.25, m2);
            MoveNServos(tempo * 0.25, m3);
            MoveNServos(tempo * 0.25, m2);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlPatada(int tempo){
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        servos_[0].SetPosition(115); servos_[3].SetPosition(120); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(115); servos_[3].SetPosition(70);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(100); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(90);  servos_[3].SetPosition(90);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
    }

    void SlTwist(int steps, int tempo){
        int m1[4] = {90, 50, 130, 90};  // SM {90,90,50,130}
        int m2[4] = {90, 90, 90, 90};
        for (int x = 0; x < steps; x++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            MoveNServos(tempo * 0.1, m1);
            MoveNServos(tempo * 0.1, m2);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlReverencia1(int steps, int tempo){
        int m1[4] = {130, 90, 90, 50};  // SM {130,50,90,90}
        int m2[4] = {90, 90, 90, 90};
        for (int x = 0; x < steps; x++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
            MoveNServos(tempo * 0.3, m1);
            vTaskDelay(pdMS_TO_TICKS(tempo * 0.2));
            MoveNServos(tempo * 0.3, m2);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlReverencia2(int steps, int tempo){
        int m1[4] = {130, 90, 90, 50};   // SM {130,50,90,90}
        int m2[4] = {130, 60, 120, 50};  // SM {130,50,60,120}
        int m3[4] = {90, 90, 90, 90};
        for (int x = 0; x < steps; x++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
            vTaskDelay(pdMS_TO_TICKS(tempo * 0.2));
            MoveNServos(tempo * 0.05, m1);
            MoveNServos(tempo * 0.05, m2);
            MoveNServos(tempo * 0.05, m1);
            MoveNServos(tempo * 0.05, m2);
            vTaskDelay(pdMS_TO_TICKS(tempo * 0.2));
            MoveNServos(tempo * 0.1, m3);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlSaludo(int steps, int tempo){
        int m1[4] = {60, 90, 90, 60};   // SM {60,60,90,90}
        int m2[4] = {120, 90, 90, 60};  // SM {120,60,90,90}
        for (int x = 0; x < steps; x++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
            MoveNServos(tempo * 0.25, m1);
            MoveNServos(tempo * 0.25, m2);
            MoveNServos(tempo * 0.25, m1);
            MoveNServos(tempo * 0.25, m2);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlUpDown(int steps, int tempo){
        int m1[4] = {50, 90, 90, 130};  // SM {50,130,90,90}
        int m2[4] = {90, 90, 90, 90};
        for (int x = 0; x < steps; x++) {
            int64_t t0 = esp_timer_get_time() / 1000;
            MoveNServos(tempo * 0.2, m1);
            vTaskDelay(pdMS_TO_TICKS(tempo * 0.4));
            MoveNServos(tempo * 0.2, m2);
            int64_t elapsed = esp_timer_get_time() / 1000 - t0;
            if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
        }
    }

    void SlKickLeft(int tempo){
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(50); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(40); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(40); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo));
    }

    void SlKickRight(int tempo){
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(120); servos_[3].SetPosition(130); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(120); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(140); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(140); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

    void SlDrunk(int tempo){
        int m1[4] = {60, 90, 90, 70};   // SM {60,70,90,90}
        int m2[4] = {110, 90, 90, 120}; // SM {110,120,90,90}
        int m5[4] = {90, 90, 90, 90};
        int64_t t0 = esp_timer_get_time() / 1000;
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
        MoveNServos(tempo * 0.06, m5);
        int64_t elapsed = esp_timer_get_time() / 1000 - t0;
        if (elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)elapsed));
    }

    void SingleLadies() {
        int t = 620;  // BPM 97

        ESP_LOGI(TAG, "Single Ladies dance starting...");
        stop_requested_ = false;
        PlaySingleLadies();
        ResetOldPositions();
        Stand();

        SlPasitos(8, t * 2);
        if (ShouldStop()) return;
        SlCrusaito(1, t);
        SlPatada(t);
        vTaskDelay(pdMS_TO_TICKS(t));
        SlTwist(2, t);
        SlTwist(3, t / 2);
        SlUpDown(1, t * 2);
        SlPatada(t * 2);
        SlDrunk(t * 2);
        SlFlapping(1, t * 2);
        SlWalk(2, t);
        SlWalk(1, t * 2);
        SlBackyard(2, t);
        SlPatada(t * 2);
        SlFlapping(1, t * 2);
        SlPatada(t * 2);
        SlTwist(8, t / 2);
        SlMoonWalkLeft(2, t);
        SlCrusaito(1, t * 2);
        if (ShouldStop()) return;

        for (int i = 0; i < 2; i++) {
            LateralFuerte(false, t);
            LateralFuerte(true, t);
            SlUpDown(1, t * 2);
            if (ShouldStop()) return;
        }

        SlSaludo(1, t * 2);
        SlSaludo(1, t);
        vTaskDelay(pdMS_TO_TICKS(t));
        SlSwing(3, t);
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(t));
        if (ShouldStop()) return;

        LateralFuerte(false, t);
        LateralFuerte(true, t);
        LateralFuerte(false, t / 2);
        LateralFuerte(true, t / 2);
        LateralFuerte(false, t / 2);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        LateralFuerte(false, t);
        LateralFuerte(true, t);
        LateralFuerte(false, t / 2);
        LateralFuerte(true, t / 2);
        vTaskDelay(pdMS_TO_TICKS(t));

        SlPasitos(1, t * 2);
        SlPasitos(1, t);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlPasitos(1, t * 2);
        SlPasitos(1, t);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        if (ShouldStop()) return;

        SlCrusaito(2, t); SlCrusaito(1, t * 2);
        SlCrusaito(2, t); SlCrusaito(1, t * 2);
        SlCrusaito(2, t); SlCrusaito(1, t * 2);

        SlUpDown(2, t);
        SlCrusaito(1, t * 2);
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlPasitos(2, t * 2);
        SlPasitos(2, t);
        SlFlapping(1, t * 2);
        SlUpDown(2, t);
        SlUpDown(1, t * 2);
        if (ShouldStop()) return;

        for (int i = 0; i < 4; i++) {
            SlPasitos(1, t);
            vTaskDelay(pdMS_TO_TICKS(t));
            if (ShouldStop()) return;
        }
        SlReverencia1(1, t * 4);
        SlReverencia2(1, t * 4);
        SlUpDown(1, t);
        SlRun(2, t / 2);
        SlPatada(t * 2);
        if (ShouldStop()) return;

        LateralFuerte(false, t);
        LateralFuerte(true, t);
        SlUpDown(2, t);
        LateralFuerte(false, t);
        LateralFuerte(true, t);
        SlUpDown(2, t);
        SlPasitos(4, t);
        LateralFuerte(false, t);
        LateralFuerte(true, t);
        SlUpDown(2, t);

        SlPatada(t * 2);
        SlPasitos(2, t);
        SlPatada(t * 2);
        SlPasitos(2, t);
        SlSwing(2, t * 2);
        SlPasitos(4, t);
        if (ShouldStop()) return;

        for (int i = 0; i < 4; i++) {
            LateralFuerte(false, t);
            LateralFuerte(true, t);
            LateralFuerte(false, t / 2);
            LateralFuerte(true, t / 2);
            LateralFuerte(false, t / 2);
            vTaskDelay(pdMS_TO_TICKS(t / 2));
            if (ShouldStop()) return;
        }

        SlPasitos(6, t);
        vTaskDelay(pdMS_TO_TICKS(t));
        SlPasitos(1, t);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlPasitos(3, t);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlSwing(4, t);
        if (ShouldStop()) return;

        SlTwist(2, t / 2); vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlTwist(2, t / 2); vTaskDelay(pdMS_TO_TICKS(t / 2));

        SlDrunk(t * 2); SlDrunk(t / 2); SlDrunk(t * 2);
        vTaskDelay(pdMS_TO_TICKS(t / 2));
        SlWalk(1, t); SlBackyard(1, t);

        servos_[0].SetPosition(110);
        servos_[3].SetPosition(130);
        vTaskDelay(pdMS_TO_TICKS(t));
        if (ShouldStop()) return;

        SlCrusaito(3, t); SlCrusaito(1, 2 * t);
        SlUpDown(1, t * 2); SlUpDown(2, t / 2);

        SlKickLeft(t / 2); SlKickRight(t / 2);
        SlMoonWalkLeft(1, t * 2); SlMoonWalkLeft(2, t);
        SlMoonWalkRight(1, t * 2); SlMoonWalkRight(2, t);

        SlWalk(4, t); SlBackyard(4, t);

        LateralFuerte(false, t); LateralFuerte(false, t);
        LateralFuerte(true, t); LateralFuerte(true, t);
        SlWalk(2, t); SlBackyard(2, t);
        if (ShouldStop()) return;

        SlPasitos(6, t * 2);
        SlSwing(1, t); SlUpDown(1, t);
        vTaskDelay(pdMS_TO_TICKS(t));
        SlUpDown(6, t);
        vTaskDelay(pdMS_TO_TICKS(t));

        for (int i = 0; i < 4; i++) {
            LateralFuerte(false, t);
            LateralFuerte(true, t);
            if (ShouldStop()) return;
        }

        vTaskDelay(pdMS_TO_TICKS(t));
        for (int i = 0; i < 8; i++) {
            SlPasitos(2, t); SlSwing(2, t);
            if (ShouldStop()) return;
        }

        SlPasitos(1, t); SlCrusaito(1, t * 2); SlUpDown(1, t);

        vTaskDelay(pdMS_TO_TICKS(2000));

        Stand();
        ESP_LOGI(TAG, "Single Ladies dance %s", stop_requested_ ? "interrupted" : "complete");
        stop_requested_ = false;
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

        // -- Dance sequences --

        mcp.AddTool(
            "self.servo.stop",
            "Immediately stop any running servo motion or dance. Use this when the user asks to stop, halt, or interrupt the current action.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                stop_requested_ = true;
                return true;
            });

        mcp.AddTool(
            "self.servo.dance_smooth_criminal",
            "Perform the full Smooth Criminal dance by Michael Jackson. A 60-second choreographed robot dance sequence. Use this when the user asks for a dance, wants to show off, or mentions Smooth Criminal.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                DispatchAction([this]() { SmoothCriminal(); });
                return true;
            });

        mcp.AddTool(
            "self.servo.dance_single_ladies",
            "Perform the full Single Ladies dance by Beyonce. A 3-minute choreographed robot dance sequence. Use this when the user asks for Beyonce, Single Ladies, or wants a different dance.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                DispatchAction([this]() { SingleLadies(); });
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

bool IsMusicPlaying() {
    return music_playing_;
}
