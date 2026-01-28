#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>               // Port 80: 控制與網頁
#include <ArduinoOTA.h>              
#include <ESPmDNS.h>                 
#include "esp_ota_ops.h"             
#include "esp_partition.h"           
#include "esp_task_wdt.h"            

// --- 新增：相機與串流庫 ---
#include "esp_camera.h"
#include "esp_http_server.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>             
#include <freertos/FreeRTOS.h>       
#include <freertos/task.h>

// ==========================================
// 1. 腳位定義 (ESP32-S3 Freenove 特規)
// ==========================================
#include "esp32s3_gpio.h"
/*
// --- 相機腳位 ---
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15   // Freenove S3 使用 15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// --- 馬達腳位 (已避開相機) ---
#define NSLEEP_PIN 48   
#define AIN1_PIN   1    // 馬達 A
#define AIN2_PIN   2
#define BIN1_PIN   14   // 馬達 B
#define BIN2_PIN   21
*/
// ==========================================
// 2. 全域變數與設定
// ==========================================
String globalHostname;               
WebServer server(80);                // Port 80: 控制 Web Server
httpd_handle_t stream_httpd = NULL;  // Port 81: 影像串流 Server

// LEDC PWM 設定
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
// ESP32-S3 Arduino Core 3.0+ 建議直接操作 Pin，但為了相容舊版寫法保留 Channel 定義
const int LEDC_CH_A1 = 0;
const int LEDC_CH_A2 = 1;
const int LEDC_CH_B1 = 2;
const int LEDC_CH_B2 = 3;

// --- 馬達參數結構體 ---
typedef struct {
    unsigned long controlTimeoutMs; 
    int pwmEffectiveLimitT; 
    int rampAccelStepT; 
    int pwmStartKickT; 
    int pwmEffectiveLimitS; 
    int rampAccelStepS; 
    int pwmStartKickS;
} MotorConfig_t;

Preferences preferences;             
MotorConfig_t motorConfig;           

volatile unsigned long lastControlTime = 0; 
volatile int targetSpeedT = 0;             
volatile int currentSpeedT = 0;            
volatile int targetSpeedS = 0;             
volatile int currentSpeedS = 0;            
const int RAMP_INTERVAL_MS = 10;           
unsigned long lastRampTime = 0;

// --- BLE UUID 定義 ---
const char* CONFIG_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; 
const char* MOTOR_SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c0ffee00dead"; 
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e" 
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e" 
#define MOTOR_CONTROL_CHAR_UUID "4fafc202-1fb5-459e-8fcc-c0ffee01feed" 
#define MOTOR_CONFIG_CHAR_UUID  "4fafc203-1fb5-459e-8fcc-c0ffee02dead" 

BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
BLECharacteristic *pSsidCharacteristic = NULL;
BLECharacteristic *pPassCharacteristic = NULL;
BLECharacteristic *pMotorConfigCharacteristic = NULL;
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;
volatile bool should_restart_advertising = false;
bool servicesStarted = false;

// ==========================================
// 3. 相機初始化與串流 Server
// ==========================================

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; // 相機專用 LEDC
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;       // 10MHz 穩定性較高
    config.pixel_format = PIXFORMAT_JPEG; 

    // S3 通常有 PSRAM，使用它來獲得更好的緩衝
    if(psramFound()){
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera Init Failed! Error: 0x%x\n", err);
        return false;
    }
    
    // 調整方向 (視鏡頭安裝方向調整)
    sensor_t * s = esp_camera_sensor_get();
    // 自動判斷鏡頭型號
    if (s->id.PID == OV3660_PID) {
        // --- 情況 A: 插的是 OV3660 ---
        Serial.println("📷 偵測到鏡頭: OV3660");
        s->set_vflip(s, 1);   // 需要翻轉
        s->set_hmirror(s, 1); // 需要鏡像
        
        // OV3660 專屬優化 (解決顏色偏淡問題)
        s->set_brightness(s, 1);  
        s->set_saturation(s, -2); 
    } 
    else {
        // --- 情況 B: 插的是 OV2640 (或其他) ---
        Serial.println("📷 偵測到鏡頭: OV2640");
        s->set_vflip(s, 0);   // 不需要翻轉
        s->set_hmirror(s, 0); // 不需要鏡像
    }
/*    
    if (s) {
        s->set_vflip(s, 0);   
        s->set_hmirror(s, 0); 
    }
*/    
    return true;
}

// 串流處理函式 (運行於 Port 81)
esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    char * part_buf[64];

    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    if(res != ESP_OK) return res;

    while(true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            res = ESP_FAIL;
        } else {
            size_t hlen = snprintf((char *)part_buf, 64, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }

        if(res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if(res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);

        if(fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
        } else if(res != ESP_OK) {
            break;
        }
    }
    return res;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81; // 設定為 81 埠
    config.ctrl_port = 32768; 

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("✅ Stream Server Started on Port 81");
    }
}

// ==========================================
// 4. HTML 網頁 (FPV 風格)
// ==========================================
const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>VibeRacer FPV</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background-color: #000; color: #fff; margin: 0; overflow: hidden; height: 100vh; display: flex; flex-direction: column; }
        
        /* 影像層 */
        #cam-container {
            position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 0;
            display: flex; justify-content: center; align-items: center;
        }
        #video { width: 100%; height: 100%; object-fit: contain; }

        /* UI 層 */
        #ui-layer { position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 10; pointer-events: none; }
        .info-bar { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.5); padding: 5px 10px; border-radius: 5px; font-size: 12px; font-family: monospace; pointer-events: auto; }

        /* 搖桿樣式 */
        .stick-zone {
            position: absolute; bottom: 40px; width: 120px; height: 120px;
            background: rgba(255, 255, 255, 0.1); border-radius: 50%;
            pointer-events: auto; touch-action: none;
        }
        #stickL { left: 30px; }
        #stickR { right: 30px; }
        
        .knob {
            position: absolute; top: 50%; left: 50%; width: 50px; height: 50px;
            background: rgba(79, 70, 229, 0.8); border-radius: 50%;
            transform: translate(-50%, -50%); box-shadow: 0 0 10px rgba(79, 70, 229, 0.5);
        }
    </style>
</head>
<body>
    <div id="cam-container">
        <img id="video" src="">
    </div>

    <div id="ui-layer">
        <div class="info-bar">
            Host: <span id="hostname">%HOSTNAME%</span><br>
            IP: <span id="ipaddress">%IPADDRESS%</span><br>
            Status: <span id="status">Ready</span>
        </div>
        
        <div id="stickL" class="stick-zone"><div class="knob"></div></div>
        <div id="stickR" class="stick-zone"><div class="knob"></div></div>
    </div>

    <script>
        // 設定影像來源 (Port 81)
        const streamUrl = `http://${location.hostname}:81/stream`;
        document.getElementById('video').src = streamUrl;

        const maxRadius = 60; 
        const baseIp = ''; 

        // 搖桿邏輯
        function setupJoystick(id, callback) {
            const zone = document.getElementById(id);
            const knob = zone.querySelector('.knob');
            let dragging = false;
            let intervalId = null;
            let lastVal = 0;

            function update(x, y) {
                const dist = Math.hypot(x, y);
                const limit = Math.min(dist, maxRadius);
                const angle = Math.atan2(y, x);
                
                const moveX = limit * Math.cos(angle);
                const moveY = limit * Math.sin(angle);
                
                knob.style.transform = `translate(calc(-50% + ${moveX}px), calc(-50% + ${moveY}px))`;
                
                // 計算歸一化數值 -1.0 ~ 1.0
                let val = 0;
                // StickL (轉向) 使用 X軸, StickR (油門) 使用 -Y軸
                if(id === 'stickL') val = moveX / maxRadius;
                if(id === 'stickR') val = -moveY / maxRadius; 
                
                // Deadzone (前端處理)
                if (Math.abs(val) < 0.1) val = 0;
                
                lastVal = Math.round(val * 255);
            }

            function start(e) {
                dragging = true;
                knob.style.transition = 'none';
                handleMove(e);
                if(intervalId) clearInterval(intervalId);
                intervalId = setInterval(() => callback(lastVal), 100);
            }

            function end() {
                dragging = false;
                knob.style.transition = '0.2s';
                knob.style.transform = 'translate(-50%, -50%)';
                lastVal = 0;
                callback(0);
                if(intervalId) clearInterval(intervalId);
            }

            function handleMove(e) {
                if(!dragging) return;
                e.preventDefault();
                const touch = e.touches ? e.touches[0] : e;
                const rect = zone.getBoundingClientRect();
                const centerX = rect.left + rect.width / 2;
                const centerY = rect.top + rect.height / 2;
                update(touch.clientX - centerX, touch.clientY - centerY);
            }

            zone.addEventListener('mousedown', start);
            zone.addEventListener('touchstart', start);
            window.addEventListener('mousemove', handleMove);
            window.addEventListener('touchmove', handleMove);
            window.addEventListener('mouseup', end);
            window.addEventListener('touchend', end);
        }

        let motorT = 0, motorS = 0;

        function sendControl() {
             fetch(`${baseIp}/control?t=${motorT}&s=${motorS}`).catch(()=>{});
             document.getElementById('status').innerText = `T:${motorT} S:${motorS}`;
        }

        // 綁定搖桿
        setupJoystick('stickL', (val) => {
            if(motorS !== val) { motorS = val; sendControl(); }
        });

        setupJoystick('stickR', (val) => {
            if(motorT !== val) { motorT = val; sendControl(); }
        });
    </script>
</body>
</html>
)rawliteral";

// ==========================================
// 5. 系統邏輯 (Hostname, Config, PWM)
// ==========================================

void generateHostname() {    
    globalHostname = "esp32s3-" + WiFi.macAddress(); 
    globalHostname.replace(":", ""); 
    globalHostname.toLowerCase(); 
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}

void loadMotorConfig() {
    MotorConfig_t defaultConfig = {
        .controlTimeoutMs = 500, 
        .pwmEffectiveLimitT = 255, .rampAccelStepT = 10, .pwmStartKickT = 200, 
        .pwmEffectiveLimitS = 255, .rampAccelStepS = 10, .pwmStartKickS = 200
    };

    preferences.begin("motor-config", true);
    size_t size = preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();

    if (size != sizeof(MotorConfig_t)) {
        motorConfig = defaultConfig; 
        Serial.println("❌ NVS 無參數，使用預設值。");
    } else {
        Serial.println("✅ NVS 參數載入成功。");
    }
}

void saveMotorConfig() {
    preferences.begin("motor-config", false);
    preferences.putBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();
}

void setMotorPwm(int speedT, int speedS) {
    // T 馬達 (速度)
    if (speedT > 0) { 
        ledcWrite(LEDC_CH_A2, 0); ledcWrite(LEDC_CH_A1, speedT);
    } else if (speedT < 0) { 
        ledcWrite(LEDC_CH_A1, 0); ledcWrite(LEDC_CH_A2, -speedT); 
    } else { 
        ledcWrite(LEDC_CH_A1, 0); ledcWrite(LEDC_CH_A2, 0);
    }

    // S 馬達 (轉向)
    if (speedS > 0) { 
        ledcWrite(LEDC_CH_B1, 0); ledcWrite(LEDC_CH_B2, speedS);
    } else if (speedS < 0) { 
        ledcWrite(LEDC_CH_B2, 0); ledcWrite(LEDC_CH_B1, -speedS); 
    } else { 
        ledcWrite(LEDC_CH_B1, 0); ledcWrite(LEDC_CH_B2, 0);
    }
}

unsigned long sMotorStartTime = 0;
const unsigned long S_MOTOR_MAX_ON_TIME = 800;

void motorRampTask() {
    if (millis() - lastRampTime < RAMP_INTERVAL_MS) return;
    lastRampTime = millis();
    
    // T Motor Logic
    if (targetSpeedT == 0) currentSpeedT = 0; 
    else {
        if (currentSpeedT == 0) currentSpeedT = (targetSpeedT > 0) ? motorConfig.pwmStartKickT : -motorConfig.pwmStartKickT;
        int diffT = targetSpeedT - currentSpeedT;
        if (abs(diffT) <= motorConfig.rampAccelStepT) currentSpeedT = targetSpeedT;
        else currentSpeedT += (diffT > 0) ? motorConfig.rampAccelStepT : -motorConfig.rampAccelStepT;
    }

    // S Motor Logic
    if (targetSpeedS == 0) {
        currentSpeedS = 0; sMotorStartTime = 0;
    } else {
        if (sMotorStartTime == 0) sMotorStartTime = millis();
        if (currentSpeedS == 0) currentSpeedS = (targetSpeedS > 0) ? motorConfig.pwmStartKickS : -motorConfig.pwmStartKickS;
        
        int diffS = targetSpeedS - currentSpeedS;
        if (abs(diffS) <= motorConfig.rampAccelStepS) currentSpeedS = targetSpeedS;
        else currentSpeedS += (diffS > 0) ? motorConfig.rampAccelStepS : -motorConfig.rampAccelStepS;

        if (millis() - sMotorStartTime > S_MOTOR_MAX_ON_TIME) currentSpeedS = constrain(currentSpeedS, -150, 150); 
    }

    currentSpeedT = constrain(currentSpeedT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT);
    currentSpeedS = constrain(currentSpeedS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
    
    setMotorPwm(currentSpeedT, currentSpeedS); 
}

// ==========================================
// 6. Web Server Handlers (Port 80)
// ==========================================
void handleRoot() {
    String html = HTML_CONTENT; 
    String ipAddress = WiFi.localIP().toString();
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);
    server.send(200, "text/html", html);
}

void handleControl() {
    if (server.hasArg("t") && server.hasArg("s")) {
        int rawT = server.arg("t").toInt();
        int rawS = server.arg("s").toInt();
        targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT); 
        targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
        lastControlTime = millis();
        server.send(200, "text/plain", "OK"); 
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleMotorConfig() {
    if (server.method() == HTTP_GET) {
        String json = "{\"controlTimeoutMs\":" + String(motorConfig.controlTimeoutMs) + "}"; // 簡化範例
        server.send(200, "application/json", json);
    } else if (server.method() == HTTP_POST) {
        if (server.hasArg("timeout")) motorConfig.controlTimeoutMs = server.arg("timeout").toInt();
        // ... (其他參數解析省略，邏輯同前)
        saveMotorConfig();
        server.send(200, "text/plain", "Saved");
    }
}

void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/control", HTTP_GET, handleControl);
    server.on("/config", HTTP_ANY, handleMotorConfig);
    server.onNotFound([](){ server.send(404); });
    server.begin();
    Serial.println("HTTP Server Started (Port 80)");
}

// ==========================================
// 7. BLE & WiFi 連線邏輯
// ==========================================
void connectToSavedWiFi() {
    preferences.begin("wifi-config", true); 
    String saved_ssid = preferences.getString("ssid", "");
    String saved_pass = preferences.getString("pass", "");
    preferences.end();
    
    if (saved_ssid.length() > 0) {
        Serial.printf("Connecting to WiFi: %s\n", saved_ssid.c_str());
        WiFi.setHostname(globalHostname.c_str());
        WiFi.mode(WIFI_STA); 
        WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
    } else {
        Serial.println("No saved WiFi. Waiting for BLE config.");
    }
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { Serial.println("BLE Connected"); }
    void onDisconnect(BLEServer* pServer) { should_restart_advertising = true; }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String command = String(rxValue.c_str());
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0) {
                targetSpeedT = command.substring(0, commaIndex).toInt();
                targetSpeedS = command.substring(commaIndex + 1).toInt();
                lastControlTime = millis(); 
            }
        }
    }
};

class ConfigCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String data = String(pCharacteristic->getValue().c_str());
        if (pCharacteristic == pSsidCharacteristic) ble_ssid = data;
        else if (pCharacteristic == pPassCharacteristic) {
            ble_pass = data;
            wifi_config_received = true;
        }
    }
};

void setupBleServer() {
    BLEDevice::init(globalHostname.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pControl = pServer->createService(MOTOR_SERVICE_UUID);
    pControlCharacteristic = pControl->createCharacteristic(MOTOR_CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pControlCharacteristic->addDescriptor(new BLE2902());
    pControlCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pControl->start();

    BLEService *pConfig = pServer->createService(CONFIG_SERVICE_UUID);
    pSsidCharacteristic = pConfig->createCharacteristic(SSID_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pSsidCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    pPassCharacteristic = pConfig->createCharacteristic(PASS_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pPassCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    pConfig->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(MOTOR_SERVICE_UUID);
    pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
    pAdvertising->start();
    Serial.println("BLE Started");
}

// ==========================================
// 8. Main Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // 0. 載入設定
    loadMotorConfig();

    // 1. 初始化相機 (S3 優先初始化相機以配置 PSRAM)
    if(initCamera()) {
        Serial.println("✅ Camera Initialized");
    } else {
        Serial.println("❌ Camera Failed");
    }

    // 2. 初始化馬達 (LEDC)
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH);
    
    // 這裡使用 v2 API 以相容舊代碼結構，若為 Core 3.0 請改用 ledcAttach
    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);
    
    // 3. 啟動連線
    generateHostname();
    setupBleServer();
    connectToSavedWiFi();
}

void loop() {
    // 1. 安全檢查
    if ((targetSpeedT != 0 || targetSpeedS != 0) && (millis() - lastControlTime > motorConfig.controlTimeoutMs)) {
        targetSpeedT = 0; targetSpeedS = 0;
    }

    // 2. BLE 重連
    if (should_restart_advertising) {
        pServer->getAdvertising()->start();
        should_restart_advertising = false;
    }

    // 3. Wi-Fi 連線後處理
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        setupWebServer();
        startCameraServer(); // 啟動影像串流
        ArduinoOTA.begin();
        servicesStarted = true;
        Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    }

    // 4. 服務 Loop
    if (servicesStarted) {
        server.handleClient();
        ArduinoOTA.handle();
    }

    // 5. BLE Config 處理
    if (wifi_config_received) {
        preferences.begin("wifi-config", false);
        preferences.putString("ssid", ble_ssid);
        preferences.putString("pass", ble_pass);
        preferences.end();
        wifi_config_received = false;
        delay(100); ESP.restart();
    }

    // 6. 馬達 Ramping
    motorRampTask();
    delay(1);
}