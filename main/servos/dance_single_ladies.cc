#include "servo_controller.h"


void ServoController::MusicTaskSl(void* arg) {
        auto self = (ServoController*)arg;
        auto codec = Board::GetInstance().GetAudioCodec();
        if (!codec) { music_playing_ = false; vTaskDelete(NULL); return; }

        auto& app = Application::GetInstance();
        app.BeginExclusiveAudioPlayback();
        codec->EnableOutput(true);

        HMP3Decoder decoder = MP3InitDecoder();
        if (!decoder) {
            codec->EnableOutput(false);
            app.EndExclusiveAudioPlayback();
            music_playing_ = false;
            vTaskDelete(NULL);
            return;
        }

        music_playing_ = true;

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
        app.EndExclusiveAudioPlayback();
        music_playing_ = false;
        vTaskDelete(NULL);
    }

void ServoController::PlaySingleLadies() {
        xTaskCreate(MusicTaskSl, "music_sl", 8192, this, 5, NULL);
    }

void ServoController::SlWalk(int steps, int T) {
        int A[4] = {15, 30, 30, 15};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlBackyard(int steps, int T) {
        int A[4] = {15, 30, 30, 15};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlRun(int steps, int T) {
        int A[4] = {10, 10, 10, 10};
        int O[4] = {0, 0, 0, 0};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlMoonWalkLeft(int steps, int T) {
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(60)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlMoonWalkRight(int steps, int T) {
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(-60)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlCrusaito(int steps, int T) {
        int A[4] = {25, 30, 30, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(-60)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlFlapping(int steps, int T) {
        int A[4] = {15, 8, 8, 15};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(-90), DEG2RAD(180)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlSwing(int steps, int T) {
        int A[4] = {25, 0, 0, 25};
        int O[4] = {-15, 0, 0, 15};
        float ph[4] = {DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0)};
        OttoOscillate(steps, T, A, O, ph);
    }

void ServoController::SlPasitos(int steps, int tempo) {
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

void ServoController::SlPatada(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        servos_[0].SetPosition(115); servos_[3].SetPosition(120); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(115); servos_[3].SetPosition(70);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(100); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(90);  servos_[3].SetPosition(90);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
    }

void ServoController::SlTwist(int steps, int tempo) {
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

void ServoController::SlReverencia1(int steps, int tempo) {
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

void ServoController::SlReverencia2(int steps, int tempo) {
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

void ServoController::SlSaludo(int steps, int tempo) {
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

void ServoController::SlUpDown(int steps, int tempo) {
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

void ServoController::SlKickLeft(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(50); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(40); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(40); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(60);  vTaskDelay(pdMS_TO_TICKS(tempo));
    }

void ServoController::SlKickRight(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(120); servos_[3].SetPosition(130); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(120); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(140); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(140); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(120); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

void ServoController::SlDrunk(int tempo) {
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

void ServoController::SingleLadies() {
        int t = 620;  // BPM 97

        ESP_LOGI(TAG, "Single Ladies dance starting...");
        stop_requested_ = false;
        PlaySingleLadies();
        { int waits = 0; while (!music_playing_ && !stop_requested_ && waits++ < 200) vTaskDelay(pdMS_TO_TICKS(10)); }
        if (ShouldStop() || !music_playing_) return;
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
