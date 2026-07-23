#include <TA_BOKS_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <time.h>

#include "HX711.h"
#include <FirebaseESP32.h>

// ======================================================
// ======================= WIFI =========================
// ======================================================

const char* ssid = "YOUR_SSID_WIFI";
const char* password = "YOUR_PASSWORD_WIFI";

// ======================================================
// ===================== TELEGRAM =======================
// ======================================================

#define BOT_TOKEN "YOUR_BOT_TOKEN_TELEGRAM"
#define CHAT_ID "YOUR_CHAT_ID_TELEGRAM"

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// ======================================================
// ===================== FIREBASE =======================
// ======================================================

#define FIREBASE_HOST "YOUR_FIREBASE_HOST"
#define FIREBASE_AUTH "YOUR_FIREBASE_AUTH"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ======================================================
// ======================== TIME ========================
// ======================================================

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// ======================================================
// ==================== ULTRASONIC ======================
// ======================================================

#define TRIG_PIN 3
#define ECHO_PIN 2

const int JARAK_TUTUP_MIN = 33;
const int JARAK_TUTUP_MAX = 44;

const unsigned long VALIDASI_BUKA_MS = 500;
const unsigned long STATUS_COOLDOWN  = 3000;
const unsigned long ultrasonicInterval = 100;

// ======================================================
// ====================== LOADCELL ======================
// ======================================================

const int HX711_dout = 41;
const int HX711_sck  = 42;

HX711 LoadCell;

// ======================================================
// ====================== GETAR =========================
// ======================================================

#define SENSOR_PIN 19
#define RELAY_PIN 48

bool alarmPlayed = false;

const int MIN_GETAR_TIME = 1200;
const int TRIGGER_LIMIT  = 5;
const int COOLDOWN_TIME  = 3000;

unsigned long startGetar = 0;
unsigned long lastAlarm  = 0;

int triggerCount = 0;

volatile bool getarTerdeteksi = false; 

// ======================================================
// ==================== STATE SYSTEM ====================
// ======================================================

enum State {
    IDLE,
    PINTU_BUKA
};

State currentState = IDLE;

// ======================================================
// ================== VARIABLE GLOBAL ===================
// ======================================================

long currentDistance = 0;
long lastDistance    = 35;

unsigned long lastUltrasonic   = 0;
unsigned long waktuMulaiBuka   = 0;
unsigned long lastStatusChange = 0;

// ===== TRIGGER =====
bool triggerAktif = false;
bool prosesPengecekanSelesai = false; 

// ===== TELEGRAM =====
bool paketSudahDikirim = false;

float beratTerakhirTerkirim = 0;
const float MIN_TAMBAHAN_BERAT = 5.0;

// ===== LOADCELL =====
float beratSebelumnya = 0;
float beratSekarangGlobal = 0;

bool loadcellSudahStabil = false;

// ===== JEDA WAKTU PAKET JATUH =====
bool sedangTungguPaketStabil = false;
unsigned long waktuPaketJatuh = 0;
const unsigned long DELAY_PAKET_STABIL = 3000; 

// ===== TIMEOUT & DEBOUNCING ML =====
bool deteksiSedangBerjalan = false;
unsigned long waktuMulaiDeteksi = 0;
const unsigned long BATAS_WAKTU_DETEKSI = 15000; 

int framePaketTerdeteksi = 0;
const int SYARAT_FRAME_VALID = 2; // Butuh 1 frame konfirmasi positif

// ===== FIREBASE =====
String statusBoxFirebase = "box kosong";
String statusPaketFirebase = "belum ada paket"; 
unsigned long lastFirebaseUpdate = 0; 
const int firebaseUpdateInterval = 2000; 

// ===== OPTIMASI =====
unsigned long lastInfer = 0;
const int inferInterval = 1000; 

unsigned long lastSend = 0;
const int sendCooldown = 2000;

// ===== CAMERA =====
uint8_t *snapshot_buf;

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15

#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11

#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 160
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 120
#define EI_CAMERA_FRAME_BYTE_SIZE 3

static bool debug_nn = false;
static bool is_initialised = false;

// ======================================================
// ==================== CAMERA CONFIG ===================
// ======================================================

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QQVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST
};

// ======================================================
// ====================== FUNCTION ======================
// ======================================================

bool ei_camera_init(void);
bool ei_camera_capture(uint32_t, uint32_t, uint8_t*);
static int ei_camera_get_data(size_t, size_t, float*);

void IRAM_ATTR getarISR() {
    getarTerdeteksi = true;
}

// ======================================================
// ================== READ ULTRASONIC ===================
// ======================================================

long readDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0) {
        return lastDistance;
    }

    long distance = duration * 0.034 / 2;
    return distance;
}

String getTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Waktu error";
    }

    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}

// ======================================================
// ==================== FIREBASE ========================
// ======================================================

void sendDataFirebase() {
    String path = "/monitoring_paket";
    FirebaseJson json;

    json.set("timestamp", getTimestamp());
    json.set("jarak_cm", currentDistance);
    json.set("berat_gram", beratSekarangGlobal);
    json.set("status_box", statusBoxFirebase);
    json.set("status_paket", statusPaketFirebase);
    
    Firebase.updateNode(fbdo, path, json);
}

void sendKeamananFirebase(int getarStatus) {
    String path = "/keamanan";
    FirebaseJson json;

    json.set("timestamp", getTimestamp());
    json.set("sensor_getar", getarStatus);

    if (getarStatus == 1) {
        json.set("status", "ada getaran");
    } else {
        json.set("status", "tidak ada getaran");
    }

    Firebase.updateNode(fbdo, path, json);
}

// FUNGSI BARU: Mengirim Riwayat Paket
void sendRiwayatFirebase(String event) {
    String path = "/riwayat_paket";
    FirebaseJson json;

    json.set("timestamp", getTimestamp());
    json.set("event", event);
    json.set("berat_gram", beratSekarangGlobal);

    // pushJSON digunakan untuk menambah data berurut (membuat unique ID otomatis)
    if (Firebase.pushJSON(fbdo, path, json)) {
        Serial.println("✅ Riwayat tersimpan di Firebase: " + event);
    } else {
        Serial.println("❌ Gagal simpan riwayat: " + fbdo.errorReason());
    }
}

// ======================================================
// ================== TELEGRAM PHOTO ====================
// ======================================================

bool sendPhotoTelegram(String caption) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    if (!client.connect("api.telegram.org", 443)) {
        esp_camera_fb_return(fb);
        return false;
    }

    String head = "--ESP32\r\n";
    head += "Content-Disposition: form-data; name=\"chat_id\";\r\n\r\n";
    head += String(CHAT_ID) + "\r\n";

    head += "--ESP32\r\n";
    head += "Content-Disposition: form-data; name=\"caption\";\r\n\r\n";
    head += caption + "\r\n";

    head += "--ESP32\r\n";
    head += "Content-Disposition: form-data; name=\"photo\"; filename=\"esp32.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";

    String tail = "\r\n--ESP32--\r\n";

    client.println("POST /bot" + String(BOT_TOKEN) + "/sendPhoto HTTP/1.1");
    client.println("Host: api.telegram.org");
    client.println("Content-Type: multipart/form-data; boundary=ESP32");
    client.println("Content-Length: " + String(head.length() + fb->len + tail.length()));
    client.println();

    client.print(head);
    client.write(fb->buf, fb->len);
    client.print(tail);

    while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
            break;
        }
    }
    client.stop();

    esp_camera_fb_return(fb);
    return true;
}

// ======================================================
// ====================== ALARM =========================
// ======================================================

void playAlarm() {
    Serial.println("🚨 GETARAN TERDETEKSI!");

    String message = "⚠️ GETARAN TERDETEKSI!\n";
    message += "Kemungkinan box dibuka paksa\n\n";
    message += "Waktu: " + getTimestamp();

    bot.sendMessage(CHAT_ID, message, "");
    digitalWrite(RELAY_PIN, HIGH);
}

// ======================================================
// ======================== SETUP =======================
// ======================================================

void setup() {
    Serial.begin(115200);

    pinMode(SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), getarISR, RISING);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    client.setInsecure();

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    psramInit();

    snapshot_buf = (uint8_t*)malloc(
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
        EI_CAMERA_FRAME_BYTE_SIZE
    );

    ei_camera_init();

    LoadCell.begin(HX711_dout, HX711_sck);
    LoadCell.set_scale(-90.27);
    LoadCell.tare();

    sendKeamananFirebase(0);

    Serial.println("✅ Sistem Siap");
}

// ======================================================
// ========================= LOOP =======================
// ======================================================

void loop() {
    delay(5);

    // ==================================================
    // =================== KEAMANAN =====================
    // ==================================================

    int getar = digitalRead(SENSOR_PIN);

    if (alarmPlayed) {
        if (millis() - lastAlarm >= 3000) { 
            digitalWrite(RELAY_PIN, LOW); 
            sendKeamananFirebase(0);      
            
            alarmPlayed = false;
            startGetar = 0;
            triggerCount = 0;
            lastAlarm = millis();         
            
            Serial.println("✅ Alarm dimatikan, sistem kembali memantau gerakan.");
        }
    } 
    else if (millis() - lastAlarm >= COOLDOWN_TIME) {
        if (getar == HIGH) {
            triggerCount++;
            if (startGetar == 0) startGetar = millis();
            if ((millis() - startGetar >= MIN_GETAR_TIME) && (triggerCount >= TRIGGER_LIMIT)) {
                sendKeamananFirebase(1);
                playAlarm();
                alarmPlayed = true;
                lastAlarm = millis(); 
            }
        } else {
            if (startGetar != 0 && (millis() - startGetar > 1000)) {
                startGetar = 0;
                triggerCount = 0;
            }
        }
    }

    // ==================================================
    // ================= ULTRASONIC =====================
    // ==================================================

    if (millis() - lastUltrasonic > ultrasonicInterval) {
        lastUltrasonic = millis();

        currentDistance = readDistanceCM();
        lastDistance = currentDistance;

        bool pintuTertutup = 
            (currentDistance >= JARAK_TUTUP_MIN && 
             currentDistance <= JARAK_TUTUP_MAX);

        if (currentState == IDLE && millis() - lastStatusChange > STATUS_COOLDOWN) {
            if (!pintuTertutup) {
                if (waktuMulaiBuka == 0) waktuMulaiBuka = millis();
                if (millis() - waktuMulaiBuka > VALIDASI_BUKA_MS) {
                    currentState = PINTU_BUKA;
                    triggerAktif = false;
                    prosesPengecekanSelesai = false; 
                    lastStatusChange = millis();
                    Serial.println("\n🚪 STATUS: PINTU TERBUKA");
                }
            } else {
                waktuMulaiBuka = 0;
            }
        }
        else if (currentState == PINTU_BUKA && millis() - lastStatusChange > STATUS_COOLDOWN) {
            if (pintuTertutup) {
                currentState = IDLE;
                triggerAktif = true; 
                lastStatusChange = millis();
                Serial.println("\n🚪 STATUS: PINTU DITUTUP");
            }
        }
    }

    if (!triggerAktif) return; 

    // ==================================================
    // ==================== LOADCELL ====================
    // ==================================================

    float beratSekarang = LoadCell.get_units(3);
    if (beratSekarang < 20.0) beratSekarang = 0;

    if (!loadcellSudahStabil) {
        beratSebelumnya = beratSekarang;
        loadcellSudahStabil = true;
        Serial.println("✅ Loadcell stabil");
        return;
    }

    // Abaikan fluktuasi jika sedang menunggu paket stabil
    if (!sedangTungguPaketStabil) {
        if (abs(beratSekarang - beratSebelumnya) < 20.0) {
            beratSekarang = beratSebelumnya; 
        } else {
            if ((beratSekarang - beratSebelumnya) > 20.0) {
                Serial.println("📦 Lonjakan berat terdeteksi! Menunggu paket stabil di dasar...");
                
                sedangTungguPaketStabil = true;
                waktuPaketJatuh = millis();

            } else if ((beratSebelumnya - beratSekarang) > 20.0) {
                Serial.println("⚠️ Paket diambil");
                beratSebelumnya = beratSekarang;
                beratTerakhirTerkirim = beratSekarang; 

                if (beratSekarang <= 0) { 
                    Serial.println("📦 Box sudah kosong sepenuhnya.");
                    beratSekarangGlobal = 0;
                    beratTerakhirTerkirim = 0;
                    paketSudahDikirim = false;
                    
                    sedangTungguPaketStabil = false; 
                    triggerAktif = false; 
                    prosesPengecekanSelesai = false; 
                    deteksiSedangBerjalan = false; 
                    framePaketTerdeteksi = 0;
                    
                    statusBoxFirebase = "box kosong";
                    statusPaketFirebase = "paket di ambil";
                    sendDataFirebase();

                    // MENCATAT RIWAYAT: PAKET DIAMBIL
                    sendRiwayatFirebase("paket diambil");
                }
                return;
            }
        }
    }

    beratSekarangGlobal = beratSekarang;

    if (millis() - lastFirebaseUpdate >= firebaseUpdateInterval) {
        lastFirebaseUpdate = millis();
        sendDataFirebase();
        Serial.print("Weight: ");
        Serial.print(beratSekarangGlobal, 2);
        Serial.println(" gram");
    }

    // ==================================================
    // =================== VALIDASI =====================
    // ==================================================

    if (beratSekarangGlobal <= 0) return; 

    bool adaPenambahanPaket = (beratSekarangGlobal - beratTerakhirTerkirim) > MIN_TAMBAHAN_BERAT;
    bool kirimPertama = !paketSudahDikirim;
    bool kirimTambahan = paketSudahDikirim && adaPenambahanPaket;
    
    bool butuhKamera = (!prosesPengecekanSelesai && (kirimPertama || adaPenambahanPaket));

    if (!butuhKamera) {
        sedangTungguPaketStabil = false;
        deteksiSedangBerjalan = false;
        return; 
    }

    // ==================================================
    // ======= JEDA MENUNGGU PAKET JATUH & STABIL =======
    // ==================================================
    if (sedangTungguPaketStabil) {
        if (millis() - waktuPaketJatuh < DELAY_PAKET_STABIL) {
            return; 
        } else {
            sedangTungguPaketStabil = false;
            
            // Simpan berat stabilnya di sini
            beratSebelumnya = beratSekarangGlobal;
            
            deteksiSedangBerjalan = true;
            waktuMulaiDeteksi = millis(); 
            framePaketTerdeteksi = 0; 
            
            Serial.println("✅ Paket sudah stabil. Mulai deteksi kamera (Maks 10 detik)...");
        }
    } else if (!deteksiSedangBerjalan) {
        deteksiSedangBerjalan = true;
        waktuMulaiDeteksi = millis();
        framePaketTerdeteksi = 0; 
    }

    // ==================================================
    // ================= CEK TIMEOUT ====================
    // ==================================================
    if (deteksiSedangBerjalan && (millis() - waktuMulaiDeteksi >= BATAS_WAKTU_DETEKSI)) {
        Serial.println("❌ Waktu habis! Objek tidak diyakini sebagai paket secara konsisten. Batal kirim.");
        
        // --- PENAMBAHAN BARU UNTUK FIREBASE ---
        statusPaketFirebase = "objek tidak dikenali";
        sendDataFirebase(); 
        sendRiwayatFirebase("objek tidak dikenali"); 
        // --------------------------------------

        prosesPengecekanSelesai = true; 
        paketSudahDikirim = true;       
        beratTerakhirTerkirim = beratSekarangGlobal; 
        deteksiSedangBerjalan = false;  
        
        return; 
    }

    // ==================================================
    // ================= MACHINE LEARNING ===============
    // ==================================================

    if (millis() - lastInfer < inferInterval) return;
    lastInfer = millis();

    Serial.println("🧠 Memproses frame kamera...");

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    bool ml_berhasil = false;
    bool frameSaatIniValid = false;

    if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        ei_impulse_result_t result = {0};

        if (run_classifier(&signal, &result, debug_nn) == EI_IMPULSE_OK) {
            ml_berhasil = true;
            
            // --- TAMBAHAN: Cetak semua hasil deteksi ke Serial Monitor sebelum divalidasi ---
            Serial.printf("🔍 Hasil Deteksi (%d objek):\n", result.bounding_boxes_count);
            if (result.bounding_boxes_count == 0) {
                Serial.println("   -> Tidak ada objek yang terdeteksi");
            } else {
                for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                    Serial.printf("   -> Label: '%s' | Confidence: %.2f\n", result.bounding_boxes[i].label, result.bounding_boxes[i].value);
                }
            }
            // --------------------------------------------------------------------------------
            
            for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                auto bb = result.bounding_boxes[i];
                
                if (bb.value < 0.7) continue; 
                
                if (String(bb.label) == "paket") {
                    frameSaatIniValid = true; 
                    break;
                }
            }
        }
    }

    if (ml_berhasil) {
        if (frameSaatIniValid) {
            framePaketTerdeteksi++;
            Serial.print("✅ Konfirmasi visual paket: Frame ke-");
            Serial.print(framePaketTerdeteksi);
            Serial.print(" dari ");
            Serial.println(SYARAT_FRAME_VALID);
        }
    }

    // ==================================================
    // ================== TELEGRAM ======================
    // ==================================================

    bool validasiKameraLolos = false;

    if (framePaketTerdeteksi >= SYARAT_FRAME_VALID) {
        validasiKameraLolos = true;
    }

    if (validasiKameraLolos && millis() - lastSend > sendCooldown) {

        lastSend = millis();

        String caption;
        if (kirimPertama) caption = "📦 Paket baru terdeteksi!\n";
        else caption = "📦 Ada penambahan paket baru!\n";

        caption += "Jarak: " + String(currentDistance) + " cm\n";

        if (beratSekarangGlobal < 1000.0) caption += "Berat Total: " + String(beratSekarangGlobal, 2) + " gram\n";
        else caption += "Berat Total: " + String(beratSekarangGlobal / 1000.0, 2) + " kg\n";

        caption += "Waktu: " + getTimestamp();

        if (sendPhotoTelegram(caption)) {
            paketSudahDikirim = true;
            beratTerakhirTerkirim = beratSekarangGlobal;
            
            prosesPengecekanSelesai = true; 
            deteksiSedangBerjalan = false; 

            statusBoxFirebase = "box ada paket";
            if (kirimPertama) statusPaketFirebase = "paket di dalam";
            else statusPaketFirebase = "penambahan paket baru";

            Serial.println("📨 Telegram (FOTO) terkirim");
            delay(1500);
            lastFirebaseUpdate = millis(); 

            // MENCATAT RIWAYAT: PAKET MASUK / PENAMBAHAN PAKET
            if (kirimPertama) {
                sendRiwayatFirebase("paket masuk");
            } else {
                sendRiwayatFirebase("penambahan paket baru");
            }

        } else {
            Serial.println("❌ Gagal kirim Telegram");
        }
    } 
    else if (!validasiKameraLolos) {
        Serial.print("⏳ Mengumpulkan bukti visual... (Sisa waktu: ");
        Serial.print((BATAS_WAKTU_DETEKSI - (millis() - waktuMulaiDeteksi)) / 1000);
        Serial.println(" detik)");
    }
}

// ======================================================
// ====================== CAMERA ========================
// ======================================================

bool ei_camera_init(void) {
    if (is_initialised) return true;
    if (esp_camera_init(&camera_config) != ESP_OK) return false;
    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t w, uint32_t h, uint8_t *out_buf) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
    esp_camera_fb_return(fb);

    if (!converted) return false;

    if (w != EI_CAMERA_RAW_FRAME_BUFFER_COLS || h != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, w, h
        );
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        pixel_ix += 3;
    }
    return 0;
}