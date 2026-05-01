# ENGR 122 Final Run

Arduino/ESP8266 project for the ENGR 122 final robot challenge using:
- MQTT camera-based localization (`x, y, heading`)
- Ultrasonic obstacle avoidance (front/left/right)
- Servo motor control for navigation
- OLED live debug output

The robot drives to a list of arena targets while avoiding walls and local obstacles.

## Project Files

- `ENGR122_Final.ino` - Main control loop, navigation, obstacle logic, OLED updates.
- `MQTT.h` - Wi-Fi + MQTT connection and message parsing callback.

## How It Works

1. Robot connects to Wi-Fi and MQTT broker.
2. Camera system publishes robot pose as `x,y,theta` to the configured topic.
3. `MQTT.h` callback parses pose into:
   - `x_robot`
   - `y_robot`
   - `z_ang_robot`
4. Main loop:
   - Reads three ultrasonic sensors.
   - Computes heading error to current target.
   - Chooses either:
     - obstacle-avoidance behavior, or
     - path-planning steering toward target.
5. When robot reaches a target radius, it advances to next target.
6. OLED displays target, error, sensor values, distance, and pose for debugging.

## Hardware

- ESP8266-based board (NodeMCU or similar)
- 2 continuous-rotation servos (left/right wheels)
- 3 ultrasonic sensors (front/left/right)
- I2C OLED display (SSD1306, address `0x3C`)
- External camera/localization system publishing pose over MQTT

## Pin Mapping (Current Sketch)

- Right motor: `D6`
- Left motor: `D3`
- Front ultrasonic: trigger `D5`, echo `D8`
- Right ultrasonic: trigger `D4`, echo `D7`
- Left ultrasonic: trigger `TX (pin 1)`, echo `D0`

Note: Left ultrasonic uses pin 1 (`TX`), which conflicts with Serial TX. The sketch includes a pin reclaim workaround (`reclaimPin1()`).

## Arduino Library Dependencies

Install these libraries in Arduino IDE:
- `Servo`
- `SSD1306Wire`
- `Ultrasonic`
- `ESP8266WiFi`
- `PubSubClient`
- `WiFiManager`

## Setup

### 1) Configure network/broker credentials

Edit `ENGR122_Final.ino` and replace placeholders:

- `REPLACE_WITH_WIFI_SSID`
- `REPLACE_WITH_WIFI_PASSWORD`
- `REPLACE_WITH_MQTT_PASSWORD`

Also verify:
- `mqtt_server`
- `mqtt_username`
- `mqtt_port`
- `topic`

### 2) Verify target coordinates

Targets are set in millimeters:
- `x_targets[]`
- `y_targets[]`

Update them to your arena map.

### 3) Calibrate motor values

Tune these constants so forward motion is straight and reverse is stable:
- `FW_L`, `FW_R`
- `BK_L`, `BK_R`

### 4) Upload and run

1. Select ESP8266 board and correct COM port in Arduino IDE.
2. Compile and upload.
3. Open serial monitor if needed.
4. Place robot in arena and confirm camera MQTT stream is active.

## Control/Behavior Summary

- **Obstacle first**: if obstacle thresholds are triggered, avoidance overrides path planning.
- **Stuck detection**: if little movement over time, robot reverse-spins to escape.
- **Wall following assist**: side sensor proximity applies steering bias.
- **Heading correction**: steering scales with angular error toward target.

## Security Notes

- Do not commit real Wi-Fi or MQTT secrets.
- Keep credentials local-only in your `.ino`.
- If credentials were ever exposed, rotate them in router/broker settings.

## Troubleshooting

- **No MQTT pose updates**
  - Confirm topic name and broker IP/port.
  - Verify publisher format is exactly `x,y,theta`.
- **Robot turns in place or drifts**
  - Recalibrate `FW_*` / `BK_*`.
  - Check wheel orientation and servo direction.
- **Bad ultrasonic readings**
  - Check wiring and power stability.
  - Ignore outliers via `readClean()` behavior.
- **Left sensor unreliable**
  - This may be due to TX pin conflicts; confirm `reclaimPin1()` is being called.

## Academic Use

This repository is intended for ENGR 122 educational use and final project development.
