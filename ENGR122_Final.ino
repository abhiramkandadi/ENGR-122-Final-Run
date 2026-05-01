/*========================================================================================================================
 * Ver.1 – 03/02/2026 SES IDEAs Program 
 * 2026S ENGR122 Sample Code for Week 7 Activity & Final Project 
 * Camera Sensor–Based Position Feedback using MQTT Network
 *
 * This sketch is provided as a baseline for the Week 7 lab activity and Final Project
 * Week7 Activity:
 *  -Based on this baseline code, students must complete the program to display the robot's position on the OLED display.
 * Final  Project:
 *  -Using the ultrasonic sensor, students must design an Obstacle Avoidance logic.
 *  -Using camera-sensor data received through the MQTT network, students must implement a Path-Planning algorithm.
 *  -Students must integrate these algorithms to control the two motors so that the robot can reach designated targets. 
 *  -The OLED display should be used for debugging purposes.
 *=========================================================================================================================*/

#include "MQTT.h"
#include <Servo.h>
#include <SSD1306Wire.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include <math.h>

Servo motorR, motorL;
#define MOTOR_R_PIN D6  // right wheel
#define MOTOR_L_PIN D3  // left wheel

Ultrasonic us_front(D5, D8);
Ultrasonic us_right(D4, D7);
Ultrasonic us_left(1, D0);  // trigger on Pin 1 (TX) - conflicts with Serial

SSD1306Wire display(0x3C, SDA, SCL);

// north arena targets (mm)
const int x_targets[] = {710, 1587, 2134, 125};
const int y_targets[] = {680,  139,  608, 125};
int target_index = 0;

const char* mqtt_server   = "192.168.0.2";
const char* mqtt_username = "user";
// Do not commit real credentials; set these locally before upload.
const char* mqtt_password = "REPLACE_WITH_MQTT_PASSWORD";
const int   mqtt_port     = 1883;
const String topic    = "EAS011_North";
const char*  ssid     = "REPLACE_WITH_WIFI_SSID";
const char*  password = "REPLACE_WITH_WIFI_PASSWORD";

float x_robot = 0, y_robot = 0, z_ang_robot = 0;

// calibrated by running both wheels and adjusting until it drove straight
const int FW_L = 113;
const int FW_R = 125;
const int BK_L = 67;
const int BK_R = 55;

// sensor thresholds (cm)
const int FRONT_THRESH = 12;
const int SIDE_THRESH  = 10;
const int WALL_FAR     = 25;

// stuck detection
float prev_x = 0, prev_y = 0;
int stuckCount = 0;

char pubString1[8]; // needed by MQTT.h extern, unused

/* MQTT.h calls Serial.print() in callback() and reconnect(),
   which re-enables Serial and takes over Pin 1. We need to grab it back
   every time MQTT does anything so the left ultrasonic sensor works.
    */
void reclaimPin1() {
  Serial.end();
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
}

// filter out garbage readings from the ultrasonic sensors
int readClean(Ultrasonic &sensor) {
  int d = sensor.read(CM);
  return (d <= 1 || d >= 350) ? -1 : d;
}

// keep angles in -180 to 180 range
float normAng(float deg) {
  while (deg >  180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

void stopMotors() { motorL.write(90);   motorR.write(90);   }
void forward()    { motorL.write(FW_L); motorR.write(FW_R); }
void backward()   { motorL.write(BK_L); motorR.write(BK_R); }

// turn toward target side, but go the other way if that side is blocked
bool shouldGoLeft(float err, int dL, int dR) {
  bool wantLeft = (err > 0);
  bool leftWall  = (dL > 0 && dL <= SIDE_THRESH);
  bool rightWall = (dR > 0 && dR <= SIDE_THRESH);
  return wantLeft ? !leftWall : rightWall;
}

// reverse and spin 
void escapeReverse(int backMs, int spinMs, float err, int dL, int dR) {
  backward();
  delay(backMs);
  stopMotors();
  delay(100);
  if (shouldGoLeft(err, dL, dR)) {
    motorL.write(BK_L); motorR.write(FW_R);
  } else {
    motorL.write(FW_L); motorR.write(BK_R);
  }
  delay(spinMs);
  stopMotors();
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(10);

  motorR.attach(MOTOR_R_PIN);
  motorL.attach(MOTOR_L_PIN);
  stopMotors();

  Wire.begin(SDA, SCL);
  delay(25);
  display.init();
  display.flipScreenVertically();

  wifi_mqtt_init();
  mqtt_clean();
  delay(500);
  reclaimPin1(); 
}

void loop() {
  unsigned long loopStart = millis();

  mqtt_rebound();
  client.loop();
  reclaimPin1();

  if (target_index >= 4) {
    stopMotors();
    display.clear();
    display.drawString(0, 20, "DONE: 4/4");
    display.display();
    mqtt_clean();
    while (millis() - loopStart < 100) { delay(5); }
    return;
  }

  // read sensors
  int dF = readClean(us_front); delay(12);
  int dL = readClean(us_left);  delay(12);
  int dR = readClean(us_right); delay(12);

  // vector to target
  float dx = (float)x_targets[target_index] - x_robot;
  float dy = (float)y_targets[target_index] - y_robot;
  float dist = sqrt(dx * dx + dy * dy);
  float ang = atan2(dy, dx) * 180.0 / PI;
  float err = normAng(ang - z_ang_robot);

  // reached target 
  if (dist < 100) {
    stopMotors();
    stuckCount = 0;
    unsigned long t = millis();
    while (millis() - t < 2000) { client.loop(); delay(10); }
    reclaimPin1();
    target_index++;
    mqtt_clean();
    reclaimPin1();
    while (millis() - loopStart < 100) { delay(5); }
    return;
  }

  // stuck detection 
  float moved = sqrt((x_robot - prev_x) * (x_robot - prev_x) +
                      (y_robot - prev_y) * (y_robot - prev_y));
  stuckCount = (moved < 15.0) ? stuckCount + 1 : 0;
  prev_x = x_robot;
  prev_y = y_robot;

  bool avoiding = false;
  bool fClose = (dF > 0 && dF <= FRONT_THRESH);
  bool lClose = (dL > 0 && dL <= SIDE_THRESH);
  bool rClose = (dR > 0 && dR <= SIDE_THRESH);

  // obstacle avoidance 
  // only back up if completely stuck or boxed in on all sides

  if (stuckCount >= 40) {
    // haven't moved in ~4 sec, something is wrong
    escapeReverse(600, 600, err, dL, dR);
    stuckCount = 0;
    avoiding = true;
  }
  else if (fClose && lClose && rClose) {
    // boxed in on all sides
    escapeReverse(500, 500, err, dL, dR);
    avoiding = true;
  }
  else if (fClose && lClose) {
    // front+left blocked, pivot right
    motorL.write(FW_L);
    motorR.write(80);
    avoiding = true;
  }
  else if (fClose && rClose) {
    // front+right blocked, pivot left
    motorL.write(80);
    motorR.write(FW_R);
    avoiding = true;
  }
  else if (fClose) {
    // front only 
    bool goL = shouldGoLeft(err, dL, dR);
    bool lNear = (dL > 0 && dL <= WALL_FAR);
    bool rNear = (dR > 0 && dR <= WALL_FAR);

    // if the side we want is also blocked, go the other way
    if (goL && lNear)       { motorL.write(FW_L); motorR.write(80); }
    else if (!goL && rNear) { motorL.write(80);   motorR.write(FW_R); }
    else if (goL)           { motorL.write(80);   motorR.write(FW_R); }
    else                    { motorL.write(FW_L); motorR.write(80); }
    avoiding = true;
  }
  else if (lClose) {
    // wall on left, veer right while still moving forward
    motorL.write(FW_L + 3);
    motorR.write(FW_R - 22);
    avoiding = true;
  }
  else if (rClose) {
    // wall on right, veer left
    motorL.write(FW_L - 22);
    motorR.write(FW_R + 3);
    avoiding = true;
  }

  // path planning 
  if (!avoiding) {
    float absErr = abs(err);
    int steerCorr = 0;
    if (absErr > 2.0) {
      steerCorr = (int)((absErr - 2.0) / 25.0 * 20.0);
      if (steerCorr > 20) steerCorr = 20;
    }

    // wall follow 
    int wallPush = 0;
    bool lWall = (dL > 0 && dL <= WALL_FAR);
    bool rWall = (dR > 0 && dR <= WALL_FAR);

    if (lWall && !rWall) {
      if (dL <= 8)       wallPush = 25;
      else if (dL <= 15) wallPush = 18;
      else               wallPush = 12;
    }
    else if (rWall && !lWall) {
      if (dR <= 8)       wallPush = -25;
      else if (dR <= 15) wallPush = -18;
      else               wallPush = -12;
    }

    // combine steering and wall corrections
    int spdL = FW_L;
    int spdR = FW_R;

    if (err > 2.0)       spdL -= steerCorr;
    else if (err < -2.0) spdR -= steerCorr;

    if (wallPush > 0)      spdR -= wallPush;
    else if (wallPush < 0) spdL -= (-wallPush);

    // clamp so we don't accidentally go backward
    if (spdL < 92) spdL = 92;
    if (spdR < 92) spdR = 92;

    motorL.write(spdL);
    motorR.write(spdR);
  }

  // debug display
  display.clear();
  display.drawString(0,  0, "T" + String(target_index + 1) + " Err:" + String(err, 0));
  display.drawString(0, 15, "F:" + String(dF) + " L:" + String(dL) + " R:" + String(dR));
  display.drawString(0, 30, "D:" + String(dist, 0) + " S:" + String(stuckCount));
  display.drawString(0, 45, "X:" + String(x_robot, 0) + " Y:" + String(y_robot, 0));
  display.display();

  mqtt_clean();
  reclaimPin1();
  while (millis() - loopStart < 100) { delay(5); }
}