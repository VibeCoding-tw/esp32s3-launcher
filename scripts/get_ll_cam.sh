#!/usr/bin/env bash
# Download the esp32s3 low-level camera implementation and header into your project.
# Uses a specific commit that contains the files.
set -e
COMMIT=b817e9b81041980608d1a3d3e0b08d86e3666382

mkdir -p src include

echo "Downloading ll_cam.c (esp32s3) ..."
curl -fsSL -o src/ll_cam.c "https://raw.githubusercontent.com/espressif/esp32-camera/${COMMIT}/target/esp32s3/ll_cam.c"

echo "Downloading ll_cam.h (private include) ..."
curl -fsSL -o include/ll_cam.h "https://raw.githubusercontent.com/espressif/esp32-camera/${COMMIT}/target/private_include/ll_cam.h"

echo "Done. Now run: pio run -t clean && pio run -v"