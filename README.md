# LiPo Charger Helper (ESP32-S3 Reverse TFT Feather)

This project turns the Adafruit ESP32-S3 Reverse TFT Feather into a battery/power status display with OTA updates over local Wi-Fi.

## Project layout (recommended)
- Sketch folder: `LiPo_Charger_Helper/`
- Sketch file: `LiPo_Charger_Helper/LiPo_Charger_Helper.ino`
- Partitions: `LiPo_Charger_Helper/partitions.csv`
- Secrets template: `LiPo_Charger_Helper/secrets.example.h`
- Local secrets: `LiPo_Charger_Helper/secrets.h` (not committed)

## Why this layout
The ESP32 Arduino CLI toolchain is most reliable when the sketch folder and `.ino` name do not contain spaces. This layout avoids build issues.

## Buttons
- `D0`: cycle screens.
- `D1` (hold ~2.2s): arm OTA window.
- `D2`: debug-visible button state.

## OTA model
- OTA is locked by default.
- Hold `D1` to arm OTA for 10 minutes.
- Endpoint: `http://<board_ip>/update`
- Auth from `secrets.h` (`OTA_HTTP_USER`, `OTA_HTTP_PASSWORD`).

## Power/charge state detection method
- Uses a confidence-based battery-present detector (avoids single-sample misclassification).
- Uses a short telemetry history window from MAX17048 (`SOC`, `VBAT`, `%/hr`) to smooth noisy readings.
- Uses USB presence from VBUS/USB signals first, with conservative inference only when battery trends clearly indicate charging.
- Adds `USB IDLE` state so USB-connected systems are not mislabeled as `DISCHARGING` when current is near-zero/noisy.
- Note: charge current limits/termination are handled by onboard charger hardware; firmware is telemetry/state logic only.

## Setup
1. Copy `secrets.example.h` to `secrets.h`.
2. Set Wi-Fi and OTA credentials.
3. Build + flash once over USB.
4. Use OTA for normal updates.

## CLI build
```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
SKETCH="LiPo_Charger_Helper"
FQBN="esp32:esp32:adafruit_feather_esp32s3_reversetft"

"$CLI" compile --clean --fqbn "$FQBN" --build-path /tmp/lipo-build --output-dir /tmp/lipo-out "$SKETCH"
```

## USB flash
```bash
PORT="/dev/cu.usbmodem101"
"$CLI" upload -p "$PORT" --fqbn "$FQBN" --input-dir /tmp/lipo-out "$SKETCH"
```

## OTA flash
1. Hold `D1` for ~2.2s to arm OTA.
2. Run:
```bash
BIN="/tmp/lipo-out/LiPo_Charger_Helper.ino.bin"
IP="<board_ip_from_screen>"

curl --fail -u ota:YOUR_OTA_PASSWORD "http://$IP/"
curl -v --fail -u ota:YOUR_OTA_PASSWORD \
  -F "firmware=@$BIN;type=application/octet-stream" \
  "http://$IP/update"
```

## Troubleshooting
- `403 OTA LOCKED`: hold `D1` again.
- `500 OTA FAIL`: reflash once by USB so partition layout is definitely applied.
- If Codex can’t reach LAN IPs, run final OTA `curl` locally on your Mac terminal.
