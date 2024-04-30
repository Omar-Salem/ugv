#! /bin/bash


cd ../esp32dev-microros
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware