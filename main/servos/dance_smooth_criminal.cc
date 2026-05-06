#include "servo_controller.h"


void ServoController::MusicTask(void* arg) {
        auto self = (ServoController*)arg;
        auto codec = Board::GetInstance().GetAudioCodec();
        if (!codec) { music_playing_ = false; vTaskDelete(NULL); return; }

        codec->EnableOutput(true);

        HMP3Decoder decoder = MP3InitDecoder();
        if (!decoder) { music_playing_ = false; vTaskDelete(NULL); return; }

        music_playing_ = true;

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

void ServoController::PlayMusic() {
        xTaskCreate(MusicTask, "music", 8192, this, 5, NULL);
    }

void ServoController::GoingUp(int tempo) {
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

void ServoController::Drunk(int tempo) {
        int m1[4] = {60, 90, 90, 70};
        int m2[4] = {110, 90, 90, 120};
        int64_t d_t0 = esp_timer_get_time() / 1000;
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
        MoveNServos(tempo * 0.235, m1);
        MoveNServos(tempo * 0.235, m2);
        int64_t d_elapsed = esp_timer_get_time() / 1000 - d_t0;
        if (d_elapsed < tempo) vTaskDelay(pdMS_TO_TICKS(tempo - (int)d_elapsed));
    }

void ServoController::NoGravity(int tempo) {
        for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
        int m1[4] = {120, 90, 90, 140};
        int m2[4] = {140, 90, 90, 140};
        int m3[4] = {90, 90, 90, 90};
        MoveNServos(tempo * 2, m1);
        MoveNServos(tempo * 2, m2);
        vTaskDelay(pdMS_TO_TICKS(tempo * 2));
        if (stop_requested_) return;
        MoveNServos(tempo * 2, m1);
        MoveNServos(tempo * 2, m3);
    }

void ServoController::KickLeft(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(50); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(30); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(30); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(80); servos_[3].SetPosition(70); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

void ServoController::KickRight(int tempo) {
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(110); servos_[3].SetPosition(130); vTaskDelay(pdMS_TO_TICKS(tempo));
        servos_[0].SetPosition(110); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(150); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(80);  vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(150); vTaskDelay(pdMS_TO_TICKS(tempo/4));
        servos_[0].SetPosition(110); servos_[3].SetPosition(100); vTaskDelay(pdMS_TO_TICKS(tempo));
    }

void ServoController::PrimeraParte(int t) {
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
            int64_t pp_t0 = esp_timer_get_time() / 1000;
            for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
            MoveNServos(t * 0.4, m1);
            MoveNServos(t * 0.4, m2);
            int64_t pp_elapsed = esp_timer_get_time() / 1000 - pp_t0;
            if (pp_elapsed < t * 2) vTaskDelay(pdMS_TO_TICKS(t * 2 - (int)pp_elapsed));
            if (stop_requested_) return;
        }

        if (stop_requested_) return;
        for (int i = 0; i < 2; i++) {
            LateralFuerte(true, t/2);
            LateralFuerte(false, t/4);
            LateralFuerte(true, t/4);
            vTaskDelay(pdMS_TO_TICKS(t));
        }

        int64_t pp_t0 = esp_timer_get_time() / 1000;
        for (int i = 0; i < 4; i++) { servos_[i].SetPosition(90); old_pos_[i] = 90; }
        Crusaito(1, t * 1.4, 15, 1);
        MoveNServos(t * 1, m3);
        for (int i = 0; i < 4; i++) servos_[i].SetPosition(90);
        int64_t pp_elapsed = esp_timer_get_time() / 1000 - pp_t0;
        if (pp_elapsed < t * 4) vTaskDelay(pdMS_TO_TICKS(t * 4 - (int)pp_elapsed));
    }

void ServoController::SegundaParte(int t) {
        int m1[4]  = {90, 80, 100, 90};
        int m2[4]  = {90, 100, 80, 90};
        int m5[4]  = {40, 80, 100, 140};
        int m6[4]  = {40, 100, 80, 140};

        for (int x = 0; x < 8; x++) {
            for (int i = 0; i < 3; i++) {
                int64_t sp_t0 = esp_timer_get_time() / 1000;
                MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
                MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
                int64_t sp_elapsed = esp_timer_get_time() / 1000 - sp_t0;
                if (sp_elapsed < t) vTaskDelay(pdMS_TO_TICKS(t - (int)sp_elapsed));
            }
            int64_t sp_t0 = esp_timer_get_time() / 1000;
            MoveNServos(t * 0.15, m5); MoveNServos(t * 0.15, m6);
            MoveNServos(t * 0.15, m1); MoveNServos(t * 0.15, m2);
            int64_t sp_elapsed = esp_timer_get_time() / 1000 - sp_t0;
            if (sp_elapsed < t) vTaskDelay(pdMS_TO_TICKS(t - (int)sp_elapsed));
            if (x % 4 == 3 && stop_requested_) return;
        }
    }

void ServoController::SmoothCriminal() {
        int t = 508;  // 118 BPM

        ESP_LOGI(TAG, "Smooth Criminal dance starting...");
        stop_requested_ = false;
        PlayMusic();
        { int waits = 0; while (!music_playing_ && !stop_requested_ && waits++ < 200) vTaskDelay(pdMS_TO_TICKS(10)); }
        if (ShouldStop() || !music_playing_) return;
        ResetOldPositions();
        Stand();

        for (int i = 0; i < 4; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;
        for (int i = 0; i < 4; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;
        for (int i = 0; i < 4; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;
        for (int i = 0; i < 4; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;

        PrimeraParte(t);
        if (ShouldStop()) return;
        SegundaParte(t);
        if (ShouldStop()) return;

        for (int i = 0; i < 2; i++) Moonwalk(4, t * 2, 30, 1);
        if (ShouldStop()) return;
        for (int i = 0; i < 2; i++) Moonwalk(4, t * 2, 30, -1);
        if (ShouldStop()) return;

        PrimeraParte(t);
        if (ShouldStop()) return;

        for (int i = 0; i < 2; i++) Crusaito(1, t * 8, 30, 1);
        if (ShouldStop()) return;
        for (int i = 0; i < 2; i++) Crusaito(1, t * 8, 30, 1);
        if (ShouldStop()) return;
        for (int i = 0; i < 16; i++) {
            if (stop_requested_) break;
            Flap(1, t / 4, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(3 * t / 4));
        }
        if (ShouldStop()) return;
        for (int i = 0; i < 16; i++) {
            if (stop_requested_) break;
            Flap(1, t / 4, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(3 * t / 4));
        }
        if (ShouldStop()) return;

        for (int i = 0; i < 2; i++) Moonwalk(4, t * 2, 30, -1);
        if (ShouldStop()) return;
        for (int i = 0; i < 2; i++) Moonwalk(4, t * 2, 30, 1);
        if (ShouldStop()) return;

        for (int i = 0; i < 4; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;
        KickLeft(t); KickRight(t);
        if (ShouldStop()) return;
        Drunk(t * 8);
        if (ShouldStop()) return;
        for (int i = 0; i < 3; i++) { Drunk(t * 4); if (stop_requested_) break; }
        if (ShouldStop()) return;
        Drunk(t); Drunk(t);
        vTaskDelay(pdMS_TO_TICKS(t * 2));
        if (ShouldStop()) return;

        for (int i = 0; i < 2; i++) Walk(2, t * 2, 1);
        if (ShouldStop()) return;
        for (int i = 0; i < 2; i++) Walk(2, t * 2, -1);
        if (ShouldStop()) return;

        GoingUp(t * 2);
        if (ShouldStop()) return;
        GoingUp(t * 1);
        if (ShouldStop()) return;
        NoGravity(t * 2);
        if (ShouldStop()) return;
        vTaskDelay(pdMS_TO_TICKS(t * 4));
        if (ShouldStop()) return;

        Crusaito(1, t * 2, 30, 1);  Crusaito(1, t * 8, 30, 1);
        if (ShouldStop()) return;
        Crusaito(1, t * 2, 30, 1);  Crusaito(1, t * 4, 30, 1);
        if (ShouldStop()) return;

        PrimeraParte(t);
        if (ShouldStop()) return;

        for (int i = 0; i < 14; i++) {
            if (stop_requested_) break;
            Flap(1, t / 2, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(t / 2));
        }
        if (ShouldStop()) return;
        for (int i = 0; i < 14; i++) {
            if (stop_requested_) break;
            Flap(1, t / 2, 15, 1);
            vTaskDelay(pdMS_TO_TICKS(t / 2));
        }

        Stand();
        ESP_LOGI(TAG, "Smooth Criminal dance %s", stop_requested_ ? "interrupted" : "complete");
        stop_requested_ = false;
    }
