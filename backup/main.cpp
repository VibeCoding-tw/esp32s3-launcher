// 包含必要的庫
#include <WiFi.h>
#include <ESPAsync_WiFiManager.h>    // 非同步 WiFiManager 庫
#include <ESPAsyncDNSServer.h>       // 用於 Captive Portal 的 DNS 伺服器
#include <ESPAsyncWebServer.h>       // 非同步 Web Server 庫
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_camera.h"              // 相機庫
#include "esp32s3_gpio.h"

// --- 全域變數 ---
String globalHostname;              // 基於 MAC 位址的唯一 Hostname
bool isConfigurationMode = false;   // 標記是否處於 Wi-Fi 配置模式

AsyncWebServer server(80);     // 實例化 Async Web Server
ESPAsync_WiFiManager *wm;      // 實例化 Async WiFiManager
AsyncDNSServer dns;

// LEDC PWM 設定
const int PWM_FREQ = 20000;        // 頻率 (Hz)
const int PWM_RESOLUTION = 8;      // 解析度 8-bit (0-255)

// PWM 通道
const int LEDC_CH_A1 = 0;       // 馬達 T 輸入 1 - AIN1
const int LEDC_CH_A2 = 1;       // 馬達 T 輸入 2 - AIN2
const int LEDC_CH_B1 = 2;       // 馬達 S 輸入 1 - BIN1
const int LEDC_CH_B2 = 3;       // 馬達 S 輸入 2 - BIN2

// --- Corrected camera_config_t Initialization ---
camera_config_t camera_config = {
    // 1. Pins
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // 2. Clock
    .xclk_freq_hz = 20000000,          // XCLK frequency (20 MHz)
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // 3. Image Format
    .pixel_format = PIXFORMAT_JPEG,    // Pixel format
    .frame_size = FRAMESIZE_SVGA,      // Frame size
    .jpeg_quality = 10,                // JPEG quality (0-63, lower is better)
    .fb_count = 1,                     // Frame buffer count
    .fb_location = CAMERA_FB_IN_PSRAM, // Frame buffer location
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0                 // SCCB I2C port (usually 0)
};
/*
camera_config_t camera_config = {
    // 1. Pins
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // 2. Clock
    .xclk_freq_hz = 20000000,          // XCLK frequency (20 MHz)
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // 3. Image Format
    .pixel_format = PIXFORMAT_JPEG,    // Pixel format
    .frame_size = FRAMESIZE_QVGA,      // Frame size
    .jpeg_quality = 12,                // JPEG quality (0-63, lower is better)
    .fb_count = 2,                     // Frame buffer count
    .grab_mode = CAMERA_GRAB_LATEST,   // Grab mode
    .sccb_i2c_port = 0                 // SCCB I2C port (usually 0)
};
*/
// --- HTML 網頁內容 (內嵌虛擬搖桿與影像串流) ---
const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 遙控車與串流</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        /* 確保全螢幕高度和柔軟的背景色 */
        body { 
            background-color: #1f2937; 
            color: #f9fafb; 
            font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, "Noto Sans", sans-serif;
            display: flex; 
            justify-content: center; 
            align-items: center; 
            min-height: 100vh; 
            margin: 0; 
            padding: 1rem;
            flex-direction: column; 
        }
        .main-content { 
            max-width: 400px; 
            width: 100%; 
            padding: 20px; 
        }
        /* 影像串流容器 */
        #stream-container {
            width: 100%;
            border-radius: 0.75rem; 
            overflow: hidden;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -2px rgba(0, 0, 0, 0.1);
            margin-bottom: 1.5rem; 
        }
        #stream-img {
            width: 100%;
            height: auto;
            display: block;
        }
        /* 搖桿圓盤樣式 */
        #joystick { 
            position: relative; 
            width: 100%;
            padding-top: 100%; /* 1:1 比例 */
            margin: 0 auto; 
            border-radius: 50%; 
            background: linear-gradient(145deg, #2d3748, #1a202c); 
            box-shadow: 10px 10px 20px #171d26, -10px -10px 20px #273142, inset 0 0 10px rgba(0,0,0,0.5);
            touch-action: none; 
        }
        /* 實際可拖曳區域 (內縮 5% 讓邊緣有陰影效果) */
        #joystick-inner {
            position: absolute;
            top: 5%; left: 5%; right: 5%; bottom: 5%;
            width: 90%;
            height: 90%;
        }
        /* 搖桿中心點 (Thumb) */
        #joystick-thumb {
            position: absolute;
            width: 70px; 
            height: 70px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            border-radius: 50%;
            background: #4f46e5;
            box-shadow: 0 0 15px #4f46e5, inset 0 0 10px #7c3aed;
            cursor: grab;
            transition: box-shadow 0.1s;
        }
        #joystick-thumb.active { cursor: grabbing; box-shadow: 0 0 25px #7c3aed, inset 0 0 15px #4f46e5; }
        /* 狀態文字 */
        #status { font-weight: 700; text-shadow: 0 0 5px rgba(79, 70, 229, 0.5); }
    </style>
</head>
<body class="p-4">
    <div class="main-content bg-gray-800 rounded-xl shadow-2xl">
        <h1 class="text-3xl font-extrabold text-center text-indigo-400 mb-2">Vibe Racer</h1>
        <p class="text-center text-sm mb-6 text-gray-400">
            裝置名稱: <span id="hostname">%HOSTNAME%</span><br>
            IP: <span id="ipaddress">%IPADDRESS%</span>
        </p>

        <div id="stream-container">
            <img id="stream-img" src="/stream" alt="Video Stream">
        </div>

        <div id="joystick" class="mb-6">
            <div id="joystick-inner">
                 <div id="joystick-thumb"></div>
            </div>
        </div>

        <div class="text-center space-y-2">
            <p class="text-xl">狀態: <span id="status" class="text-green-400">靜止</span></p>
            <p class="text-xs text-gray-500">
                X (轉向): <span id="val_x">0.00</span> | Y (速度): <span id="val_y">0.00</span>
            </p>
        </div>
    </div>

    <script>
        const joystickContainer = document.getElementById('joystick'); 
        const joystick = document.getElementById('joystick-inner'); 
        const thumb = document.getElementById('joystick-thumb');
        const statusEl = document.getElementById('status');
        const valXEl = document.getElementById('val_x');
        const valYEl = document.getElementById('val_y');
        
        const maxRadius = joystick.clientWidth / 2;
        let isDragging = false;
        let controlInterval;
        let lastMotorT = 0; 
        let lastMotorS = 0; 

        const currentIP = document.getElementById('ipaddress').textContent;
        const baseIp = currentIP.startsWith('192.168.4.1') ? 'http://192.168.4.1' : '';
        
        function updateMotorValues(rawX, rawY) {
            
            const distance = Math.sqrt(rawX*rawX + rawY*rawY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            const angle = Math.atan2(rawY, rawX);
            
            const normX = magnitude * Math.cos(angle); 
            const normY = magnitude * Math.sin(angle); 

            let motorT_float = normY; 
            let motorS_float = normX; 

            const speedT = Math.round(motorT_float * 255);
            const speedS = Math.round(motorS_float * 255);

            valYEl.textContent = speedT; 
            valXEl.textContent = speedS; 
            
            let currentStatus = "靜止";
            let statusColor = "text-green-400";
            if (Math.abs(speedT) > 5 || Math.abs(speedS) > 5) {
                 statusColor = "text-yellow-400";
                 if (speedT > 50 && Math.abs(speedS) < 50) currentStatus = "前進加速中";
                 else if (speedT < -50 && Math.abs(speedS) < 50) currentStatus = "後退減速中";
                 else if (speedS > 50) currentStatus = "右轉中";
                 else if (speedS < -50) currentStatus = "左轉中";
                 else currentStatus = "移動中";
            } else {
                 statusColor = "text-green-400";
            }
            statusEl.textContent = currentStatus;
            statusEl.className = statusColor;

            if (speedT !== lastMotorT || speedS !== lastMotorS) {
                lastMotorT = speedT;
                lastMotorS = speedS;
                sendControl(speedT, speedS); 
            }
        }

        function sendControl(T, S) {
            fetch(`${baseIp}/control?t=${T}&s=${S}`, { method: 'GET' })
                .then(response => {
                    if (!response.ok) {
                        console.error('Server responded with an error:', response.status);
                    }
                })
                .catch(error => {
                    // console.error('Control command failed:', error);
                });
        }

        function resetThumbPosition() {
            thumb.style.left = '50%';
            thumb.style.top = '50%';
            thumb.style.transform = 'translate(-50%, -50%)';
            thumb.classList.remove('active');
        }

        function stopMotors() {
            isDragging = false;
            if (controlInterval) clearInterval(controlInterval);
            resetThumbPosition();
            updateMotorValues(0, 0); 
        }

        function handleMove(e) {
            e.preventDefault();
            if (!isDragging) return;

            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;

            const rect = joystick.getBoundingClientRect();
            const centerX = rect.left + maxRadius;
            const centerY = rect.top + maxRadius;

            let offsetX = clientX - centerX;
            let offsetY = clientY - centerY; 
            
            const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            if (distance > maxRadius) {
                const angle = Math.atan2(offsetY, offsetX);
                offsetX = maxRadius * Math.cos(angle);
                offsetY = maxRadius * Math.sin(angle);
            }
            
            const thumbX = maxRadius + offsetX;
            const thumbY = maxRadius + offsetY; 

            thumb.style.left = `${thumbX}px`;
            thumb.style.top = `${thumbY}px`;
            thumb.style.transform = 'translate(-50%, -50%)';

            updateMotorValues(offsetX, -offsetY);
        }

        function handleStart(e) {
            isDragging = true;
            thumb.classList.add('active');
            handleMove(e); 

            if (controlInterval) clearInterval(controlInterval);
            controlInterval = setInterval(() => {
                sendControl(lastMotorT, lastMotorS);
            }, 100); 
        }

        function handleEnd() {
            stopMotors();
        }

        // --- 事件監聽 ---
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);

        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove);
        joystickContainer.addEventListener('touchend', handleEnd); 

        stopMotors(); 
    </script>
</body>
</html>
)rawliteral";

// --- 輔助函式 ---

// 產生基於 MAC 位址的 Hostname (修正為 S3)
void generateHostname() {    
    globalHostname = "esp32s3-" + WiFi.macAddress(); 
    globalHostname.replace(":", ""); 
    globalHostname.toLowerCase(); 
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}

// --- 馬達控制邏輯 (LEDC PWM) ---
void setMotorT(int speed) {
    speed = constrain(speed, -255, 255); 
    if (speed > 0) { 
        ledcWrite(LEDC_CH_A1, speed);
        ledcWrite(LEDC_CH_A2, 0);
    } else if (speed < 0) { 
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, -speed); 
    } else { 
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, 0);
    }
}

void setMotorS(int speed) {
    speed = constrain(speed, -255, 255); 
    if (speed > 0) { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, speed);
    } else if (speed < 0) { 
        ledcWrite(LEDC_CH_B1, -speed); 
        ledcWrite(LEDC_CH_B2, 0);
    } else { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, 0);
    }
}

#include "esp_log.h"
#include "driver/i2c.h"

#define CAM_ADDR_1 0x30
#define CAM_ADDR_2 0x21

void test_camera_i2c() {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  uint8_t val = 0;
  uint8_t reg_1C[1] = {0x1C};
esp_err_t err = i2c_master_write_read_device(
    I2C_NUM_0, CAM_ADDR_1, reg_1C, 1, &val, 1, 1000 / portTICK_PERIOD_MS);

  //esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR_1,
  //                                             (uint8_t[]){0x1C}, 1, &val, 1, 1000 / portTICK_PERIOD_MS);
  if (err == ESP_OK)
      ESP_LOGI("I2C", "Camera responded at 0x30 with val=0x%02X", val);
  else
      ESP_LOGW("I2C", "No response from 0x30 (err=0x%x)", err);

err = i2c_master_write_read_device(
    I2C_NUM_0, CAM_ADDR_2, reg_1C, 1, &val, 1, 1000 / portTICK_PERIOD_MS);
  //err = i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR_2,
  //                                   (uint8_t[]){0x1C}, 1, &val, 1, 1000 / portTICK_PERIOD_MS);
  if (err == ESP_OK)
      ESP_LOGI("I2C", "Camera responded at 0x21 with val=0x%02X", val);
  else
      ESP_LOGW("I2C", "No response from 0x21 (err=0x%x)", err);

  i2c_cmd_link_delete(cmd);
}

#include "esp_camera.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define CAM_SDA  13  // change to your real SDA pin
#define CAM_SCL  12  // change to your real SCL pin
#define CAM_ADDR 0x30 // most common camera address

void probeCameraID() {
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // temporarily change grab mode

  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = (gpio_num_t)CAM_SDA,
      .scl_io_num = (gpio_num_t)CAM_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {.clk_speed = 100000},
      .clk_flags = 0,
  };
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

  uint8_t high = 0, low = 0;
  //i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, (uint8_t[]){0x1C}, 1, &high, 1, 1000 / portTICK_PERIOD_MS);
  uint8_t reg;
  reg = 0x1C;
  i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, &reg, 1, &high, 1, 1000 / portTICK_PERIOD_MS);

reg = 0x1D;
i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, &reg, 1, &low, 1, 1000 / portTICK_PERIOD_MS);

reg = 0x0A;
i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, &reg, 1, &high, 1, 1000 / portTICK_PERIOD_MS);

reg = 0x0B;
i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, &reg, 1, &low, 1, 1000 / portTICK_PERIOD_MS);

  //i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, (uint8_t[]){0x1D}, 1, &low, 1, 1000 / portTICK_PERIOD_MS);
  ESP_LOGI("CAM_PROBE", "Manufacturer ID = 0x%02X%02X", high, low);

  //i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, (uint8_t[]){0x0A}, 1, &high, 1, 1000 / portTICK_PERIOD_MS);
  //i2c_master_write_read_device(I2C_NUM_0, CAM_ADDR, (uint8_t[]){0x0B}, 1, &low, 1, 1000 / portTICK_PERIOD_MS);
  ESP_LOGI("CAM_PROBE", "Product ID = 0x%02X%02X", high, low);

  i2c_driver_delete(I2C_NUM_0);
}

// --- 相機初始化 ---
bool initCamera() {

    // 由於 PSRAM 已確認工作，直接指定它
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.fb_count    = 2; // 或 1，先用 1 減少資源消耗
    
    probeCameraID();
    test_camera_i2c();
/*    
    if (psramFound()) {
        Serial.println("PSRAM Found. Using PSRAM for Frame Buffer.");
    } else {
        Serial.println("PSRAM Not Found. WARNING: Stream stability may be low.");
        camera_config.fb_count = 1; 
        camera_config.fb_location = CAMERA_FB_IN_DRAM;
        camera_config.frame_size = FRAMESIZE_QQVGA; // QQVGA (160x120) for safety
    }
*/    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("相機初始化失敗 (0x%x)\n", err);
        return false;
    }
    Serial.println("✅ 相機初始化成功。");
    
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1); 
        s->set_hmirror(s, 1); 
        s->set_quality(s, 15); // 質量 (10-30 為一般範圍)
    }
    return true;
}

// --- Web Server 處理函式 (Async 版本) ---

// 處理 MJPEG 影像串流
void handle_jpg_stream(AsyncWebServerRequest *request) {
    
    const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
    const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
    const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    AsyncResponseStream *response = request->beginResponseStream(_STREAM_CONTENT_TYPE);
    response->addHeader("Access-Control-Allow-Origin", "*");
    
    if (!request->client()->connected()) {
        //response->send();
        return;
    }

    while (request->client()->connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("相機取幀失敗");
            vTaskDelay(100);
            continue;
        }

        // 寫入邊界
        response->print(_STREAM_BOUNDARY);
        // 寫入 JPEG 內容頭
        response->printf(_STREAM_PART, fb->len);
        // 寫入 JPEG 資料
        if (request->client()->write((char*)fb->buf, fb->len) != fb->len) {
            esp_camera_fb_return(fb);
            break;
        }

        esp_camera_fb_return(fb);
        // 輕微延遲以讓出 CPU 給其他任務 (防止 WDT 超時，即使在串流任務中)
        vTaskDelay(1); 
    }
    //response->send();
}

// 處理根目錄請求 (搖桿頁面)
void handleRoot(AsyncWebServerRequest *request) {
    
    String html = HTML_CONTENT; 
    String ipAddress = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);

    request->send(200, "text/html", html);
}

// 處理馬達控制 API 請求
void handleControl(AsyncWebServerRequest *request) {
    if (request->hasParam("t") && request->hasParam("s")) {
        int speed_Throttle = request->arg("t").toInt(); 
        int speed_Steering = request->arg("s").toInt();
        
        setMotorT(speed_Throttle);      
        setMotorS(speed_Steering); 

        Serial.printf("WebControl: T馬達(速度)=%d, S馬達(轉向)=%d\n", speed_Throttle, speed_Steering);        
        request->send(200, "text/plain", "OK"); 
    } else {
        request->send(400, "text/plain", "Invalid arguments (Missing t or s)");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Async Web Server ---");

    server.on("/", HTTP_GET, handleRoot);
    server.on("/stream", HTTP_GET, handle_jpg_stream); // 新增串流路由
    server.on("/control", HTTP_GET, handleControl);

    server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404, "text/plain", "Not Found");
    });

    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80 (Async)。");
}

// --- mDNS/OTA 設定 ---
void setupMdnsOtaSta() {
    Serial.println("--- 設定 mDNS 和 OTA (STA 模式) ---");

    if (MDNS.begin(globalHostname.c_str())) {
        Serial.printf("mDNS (STA 模式) 啟動: %s.local -> %s\n", 
            globalHostname.c_str(), WiFi.localIP().toString().c_str());
    } else {
        Serial.println("mDNS (STA 模式) 啟動失敗。");
    }

    ArduinoOTA.setHostname(globalHostname.c_str());
    ArduinoOTA.setPassword("mysecurepassword");

    ArduinoOTA.onStart([]() { Serial.println("OTA 更新開始..."); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA 更新完成! 正在重啟..."); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA 錯誤碼 [%u]\n", error); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("進度: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.begin();
    
    Serial.println("-------------------------------------------------");
}

// --- 連線或啟動 Wi-Fi 配置入口網站 (Async 版) ---
void connectToWiFi() {

    WiFi.setHostname(globalHostname.c_str());

    wm->setAPCallback([](ESPAsync_WiFiManager *wm) {
        isConfigurationMode = true;
        Serial.println("進入配置 AP 模式 (ESP32-Setup)。");
        Serial.println("連線 AP: " + WiFi.softAPSSID());
        Serial.println("IP 位址: " + WiFi.softAPIP().toString());
    });
    
    wm->setSaveConfigCallback([](){
        isConfigurationMode = false;
        Serial.println("✅ Wi-Fi 成功配置並連線!");
    });

    Serial.println("正在啟動 AsyncWiFiManager autoConnect...");

    if (!wm->autoConnect("ESP32-Setup")) {        
        Serial.println("AutoConnect 失敗，設定為配置模式旗標。");
        isConfigurationMode = true;
    } else {
        isConfigurationMode = false;
        Serial.println("✅ Wi-Fi 連線成功!");
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 使用 vTaskDelay

    // --- 初始化馬達控制腳位 (DRV8833) ---
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH); 
    Serial.printf("馬達驅動 (nSLEEP) 已致能於 GPIO%d\n", NSLEEP_PIN);

    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);

    setMotorT(0);
    setMotorS(0);
    
    // --- 啟動相機 ---
    if (!initCamera()) {
        Serial.println("致命錯誤: 相機無法啟動。請檢查接線。");
        // 即使相機失敗，我們仍繼續啟動 Web 和 Wi-Fi
    }

    // --- 啟動器核心邏輯 ---
    wm = new ESPAsync_WiFiManager(&server, &dns, "ESP32-Setup");
    
    generateHostname();

    connectToWiFi();

    if (!isConfigurationMode) {
        setupMdnsOtaSta();
        setupWebServer();

        Serial.println("-------------------------------------------------------");
        Serial.println("裝置已連線。請瀏覽器打開 IP 或 mDNS 名稱。");
        Serial.println("-------------------------------------------------------");

    } else {
        Serial.println("⚠️ 進入 AP 配置模式。請連線 'ESP32-Setup' 進行配置。");
    }
}

// --- Loop ---
void loop() {
    if (!isConfigurationMode) {
        // 只有在 STA 模式下才處理 OTA
        ArduinoOTA.handle(); 
    }
    // 讓 app_main 中的 vTaskDelay(1) 處理 CPU 讓渡
}

// --- app_main ---
extern "C" void app_main()
{
    initArduino();   
    setup();         
    for (;;) {
        loop();      
        // 將 CPU 讓出給其他 FreeRTOS 任務，確保 Watchdog 被重置。
        vTaskDelay(1); 
    }
}