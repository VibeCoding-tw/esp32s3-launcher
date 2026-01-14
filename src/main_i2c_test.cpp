#include <Arduino.h>
#include <Wire.h>
#include <driver/i2c.h>
#include <esp_log.h>

// --- Configuration ---

// XCLK 設置 (Freenove ESP32-S3 WROOM 標準)
const int XCLK_PIN = 10;
const int XCLK_CHANNEL = 0;
const uint32_t XCLK_FREQ_HZ = 20000000; // OV3660 建議 20MHz

// 相機啟用/重置腳位 (Freenove ESP32-S3 WROOM 標準)
const int PWDN_PIN = 48; // Power Down: LOW 啟用相機
const int RESET_PIN = 47; // Reset: LOW 重置

// 測試腳位對 (SDA, SCL)
// 4,3 是 Freenove S3-CAM 的標準 SCCB 腳位
int pairs[][2] = {
  {4,3},      // 標準 SCCB 腳位
  {18,17},    // 備用 I2C 腳位
  {43,44},    // 額外測試
  {21,22},    // 額外測試
};

// I2C 掃描設置
static const char *TAG = "I2C_SCAN";
#define I2C_MASTER_FREQ_HZ 100000 
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_PORT_TO_USE I2C_NUM_0 // 使用 I2C Port 0 進行測試

// --- XCLK Functions ---

void startXCLK() {
  const int resolution = 4; // 4-bit duty resolution
  // 嘗試 20MHz XCLK
  if (!ledcSetup(XCLK_CHANNEL, XCLK_FREQ_HZ, resolution)) {
    Serial.println("⚠️ ledc setup failed, trying lower freq...");
    ledcSetup(XCLK_CHANNEL, 8000000, resolution); // 降級至 8MHz
  }
  ledcAttachPin(XCLK_PIN, XCLK_CHANNEL);
  ledcWrite(XCLK_CHANNEL, 8); // 50% duty (max=15 for 4-bit)
  Serial.printf("✅ XCLK running on pin %d @ %.1f MHz\n", XCLK_PIN, (float)ledcReadFreq(XCLK_CHANNEL)/1e6);
}

void stopXCLK() {
  ledcDetachPin(XCLK_PIN);
  Serial.println("XCLK stopped");
}

// --- Camera PWDN/RESET Control ---

void initCameraPowerAndReset() {
    // 設置 PWDN 和 RESET 腳位為輸出
    pinMode(PWDN_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);

    // 1. 啟用相機 (PWDN = LOW)
    digitalWrite(PWDN_PIN, LOW); // LOW 啟用相機 (HIGH 為 Power Down)
    delay(50); 

    // 2. 重置相機 (RESET = LOW -> HIGH)
    digitalWrite(RESET_PIN, LOW); // 設置為 LOW 進行重置
    delay(50);
    digitalWrite(RESET_PIN, HIGH); // 拉高解除重置
    delay(500); // 必須等待較長時間，確保相機啟動

    Serial.printf("\n*** Camera PWDN=%d (LOW), RESET=%d (PULSE) Applied ***\n", PWDN_PIN, RESET_PIN);
}


// --- ESP-IDF I2C Scan (Low-Level) ---

void esp_idf_i2c_scan(i2c_port_t i2c_num, int sda_pin, int scl_pin) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // 強制啟用內部上拉
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        },
        .clk_flags = 0,
    };

    Serial.printf("\n--- ESP-IDF Scanning I2C Port %d: SDA=%d, SCL=%d (100KHz) ---\n", i2c_num, sda_pin, scl_pin);

    // 1. 驅動程式配置
    esp_err_t err = i2c_param_config(i2c_num, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed (0x%x)", err);
        i2c_driver_delete(i2c_num);
        return;
    }

    // 2. 驅動程式安裝
    err = i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed (0x%x)", err);
        return;
    }

    // 3. 執行掃描
    int found_devices = 0;
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // I2C 地址格式: 7-bit address + 1-bit WRITE (0)
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        // 嘗試發送命令，等待 50ms
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // OV3660 地址通常是 0x3C (5EH) 或 0x3A (5CH) (7-bit)
            Serial.printf("  -> ✅ Found device at 0x%02X\n", address);
            found_devices++;
        }
    }

    if (found_devices == 0) {
        Serial.println("  ❌ No devices found");
    } else {
        Serial.printf("  🎉 Total %d device(s) found!\n", found_devices);
    }

    // 4. 清理驅動程式
    i2c_driver_delete(i2c_num);
}


// --- Arduino Setup & Loop ---

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("\n=== ESP32-S3 XCLK + ESP-IDF I2C Scanner ===");

  // 1. 啟用相機電源和重置
  initCameraPowerAndReset();

  // 2. 啟動 XCLK
  startXCLK(); 
  delay(200);

  // 3. 執行 I2C 掃描
  for (size_t i = 0; i < sizeof(pairs)/sizeof(pairs[0]); ++i) {
    //esp_idf_i2c_scan(I2C_PORT_TO_USE, pairs[i][0], pairs[i][1]);
  }

// 3. 執行 I2C 掃描：使用 I2C_NUM_1 測試標準 SCCB 腳位 (SDA=4, SCL=3)
  // 如果 I2C_NUM_0 有問題，I2C_NUM_1 可能會成功
  esp_idf_i2c_scan(I2C_PORT_TO_USE, 4, 3);
  esp_idf_i2c_scan(I2C_NUM_1, 4, 3);

  stopXCLK();
  Serial.println("\n--- Scan finished ---");
}

void loop() {
  // nothing
}

// --- app_main (PlatformIO/ESP-IDF Integration) ---
extern "C" void app_main()
{
    initArduino();   
    setup();         
    for (;;) {
        loop();      
        // 將 CPU 讓出給其他 FreeRTOS 任務
        vTaskDelay(1); 
    }
}