# LiPo Charger Helper (ESP32-S3 Reverse TFT Feather)

This project turns the Adafruit ESP32-S3 Reverse TFT Feather into a battery telemetry display with an on-demand Wi-Fi + OTA screen.

## Project layout (recommended)
- Sketch folder: `LiPo_Charger_Helper/`
- Sketch file: `LiPo_Charger_Helper/LiPo_Charger_Helper.ino`
- Partitions: `LiPo_Charger_Helper/partitions.csv`
- Secrets template: `LiPo_Charger_Helper/secrets.example.h`
- Local secrets: `LiPo_Charger_Helper/secrets.h` (not committed)

## Why this layout
The ESP32 Arduino CLI toolchain is most reliable when the sketch folder and `.ino` name do not contain spaces. This layout avoids build issues.

## Buttons
- `D0`: cycle between Battery screen and WiFi/OTA screen.
- `D1` short press: enable Wi-Fi (on-demand).
- `D1` hold ~2.2s: unlock OTA upload for 10 minutes.

## OTA model
- Wi-Fi is off by default to preserve battery.
- Wi-Fi is enabled only when `D1` is pressed.
- OTA is locked by default.
- Hold `D1` to arm OTA for 10 minutes.
- Endpoint: `http://<board_ip>/update`
- Auth from `secrets.h` (`OTA_HTTP_USER`, `OTA_HTTP_PASSWORD`).

## Telemetry model
- Main screen shows live MAX17048 telemetry:
  - `SOC`
  - `vBat`
  - `cRate`
- No battery/USB classification logic is used.

## Setup
1. Copy `secrets.example.h` to `secrets.h`.
2. Set Wi-Fi and OTA credentials.
3. Build + flash once over USB.
4. For OTA updates, press/hold `D1` to bring up Wi-Fi and unlock OTA.

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
1. Press `D1` once to enable Wi-Fi and connect.
2. Hold `D1` for ~2.2s to unlock OTA.
3. Run:
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
