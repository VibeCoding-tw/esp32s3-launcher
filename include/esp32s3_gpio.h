#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

/*
#define CAM_PIN_PWDN  -1    // Power down not used
#define CAM_PIN_RESET 38    // Reset
#define CAM_PIN_XCLK   39   // XCLK (clock output)
#define CAM_PIN_SIOD   40   // SDA
#define CAM_PIN_SIOC   41   // SCL

#define CAM_PIN_D7     13
#define CAM_PIN_D6     14
#define CAM_PIN_D5     15
#define CAM_PIN_D4     16
#define CAM_PIN_D3     17
#define CAM_PIN_D2     18
#define CAM_PIN_D1     12
#define CAM_PIN_D0     11

#define CAM_PIN_VSYNC  42
#define CAM_PIN_HREF    2
#define CAM_PIN_PCLK    1

// ==== Freenove ESP32-S3 WROOM Camera Pins ====
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_PCLK  13
#define CAM_PIN_XCLK  15
#define CAM_PIN_D7    16
#define CAM_PIN_D6    17
#define CAM_PIN_D5    18
#define CAM_PIN_D4    12
#define CAM_PIN_D3    10
#define CAM_PIN_D2    8
#define CAM_PIN_D1    9
#define CAM_PIN_D0    11
#define CAM_PIN_RESET -1   // 未連接
#define CAM_PIN_PWDN  -1   // 未連接

// --- 相機腳位定義 (ESP32-S3 Board) ---
#define CAM_PIN_SIOD  4    // SDA
#define CAM_PIN_SIOC  3    // SCL
#define CAM_PIN_VSYNC 5
#define CAM_PIN_HREF  6
#define CAM_PIN_PCLK  14
#define CAM_PIN_XCLK  15
#define CAM_PIN_D7    16
#define CAM_PIN_D6    17
#define CAM_PIN_D5    13
#define CAM_PIN_D4    12
#define CAM_PIN_D3    10
#define CAM_PIN_D2    8
#define CAM_PIN_D1    9
#define CAM_PIN_D0    11
#define CAM_PIN_RESET -1   // 未連接
#define CAM_PIN_PWDN  -1   // 未連接
*/
// --- 馬達控制腳位與 PWM 設定 (已修正，避免與 Flash/PSRAM 衝突) ---
#define AIN1_PIN 46   // 馬達 T 輸入 1 (PWM)
#define AIN2_PIN 45   // 馬達 T 輸入 2 (PWM)
#define BIN1_PIN 34   // 馬達 S 輸入 1 (PWM)
#define BIN2_PIN 33   // 馬達 S 輸入 2 (PWM)
#define NSLEEP_PIN 38 // 高電位致能馬達驅動器 (使用 GPIO 38 或其他安全腳位)
