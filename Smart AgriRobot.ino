#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// =====================================================
// WIFI DASHBOARD
// =====================================================

const char* WIFI_NAME = "AgriRobot";
const char* WIFI_PASSWORD = "AgriRobot123";

WebServer server(80);

// =====================================================
// PIN CONFIGURATION (UNCHANGED FROM BASE CODE)
// =====================================================

// L298N
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA 32
#define ENB 33

// Ultrasonic
#define TRIG_PIN 23
#define ECHO_PIN 22

// Servos
#define SCAN_SERVO_PIN 18
#define DIG_SERVO_PIN 21
#define SEED_SERVO_PIN 13
#define SOIL_SERVO_PIN 16

// Relay and moisture sensor
#define RELAY_PIN 19
#define SOIL_SENSOR_PIN 34

// =====================================================
// SERVO OBJECTS
// =====================================================

Servo scanServo;
Servo digServo;
Servo seedServo;
Servo soilServo;

// =====================================================
// ORIGINAL SETTINGS
// =====================================================

const int MOTOR_SPEED = 170;

const int OBSTACLE_DISTANCE = 30;
const int MINIMUM_CLEARANCE = 15;

const int CENTER_ANGLE = 90;
const int LEFT_SCAN = 160;
const int RIGHT_SCAN = 20;

const unsigned long FORWARD_30CM_MS = 1500;

const unsigned long REVERSE_TIME = 350;
const unsigned long REVERSE_EXTRA_TIME = 600;
const unsigned long TURN_TIME = 850;

const int LEFT_POSITION = 45;
const int RIGHT_POSITION = 135;

const int DIG_CYCLES = 10;
const unsigned long DIG_INTERVAL = 150;

const int SEED_CLOSED_ANGLE = 90;
const int SEED_OPEN_ANGLE = 160;

const unsigned long HOLD_OPEN_TIME = 1000;
const unsigned long HOLD_CLOSED_TIME = 2000;

// Soil calibration
const int SOIL_DRY_VALUE = 3834;
const int SOIL_WET_VALUE = 1278;

// Soil probe positions
const int SOIL_RAISED = 0;
const int SOIL_LOWERED = 90;
int soilServoCurrentAngle = SOIL_RAISED;
int soilServoTargetAngle = SOIL_RAISED;
unsigned long lastSoilServoStep = 0;

// =====================================================
// GLOBAL STATE
// =====================================================

enum RobotMode {
  MODE_MANUAL,
  MODE_AUTO
};

RobotMode robotMode = MODE_MANUAL;

bool scannerEnabled = false;
bool obstacleBlocked = false;
bool scannerTurning = false;

// Press-and-hold manual driving safety state.
bool manualDriveActive = false;
unsigned long lastManualDriveCommand = 0;
const unsigned long MANUAL_DRIVE_TIMEOUT = 600;

String scannerDirection = "Center";
String obstacleStatus = "Clear";
String currentMovement = "Stopped";

unsigned long forwardMovementTime = 0;

// Manual scanner state machine
enum ScannerState {
  SCAN_CENTER,
  SCAN_RIGHT,
  SCAN_LEFT,
  SCAN_TURN,
  SCAN_RETURN
};

ScannerState scannerState = SCAN_CENTER;

unsigned long scannerTimer = 0;
float rightDistance = -1;
float leftDistance = -1;

int selectedTurn = 0; // 1 = right, -1 = left

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void relayOn();
void relayOff();
void handleManualScanner();
void automaticCycle();
void handleModeChange();

// =====================================================
// RESPONSIVE WAIT
// Services dashboard while waiting.
// =====================================================

bool waitMode(unsigned long duration, RobotMode requiredMode) {
  unsigned long start = millis();

  while (millis() - start < duration) {
    server.handleClient();

    if (robotMode != requiredMode) {
      stopMotors();
      return false;
    }

    delay(2);
  }

  return true;
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  currentMovement = "Stopped";
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, MOTOR_SPEED);
  ledcWrite(ENB, MOTOR_SPEED);

  currentMovement = "Forward";
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 150);
  ledcWrite(ENB, 150);

  currentMovement = "Backward";
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, 180);
  ledcWrite(ENB, 180);

  currentMovement = "Turning Left";
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 180);
  ledcWrite(ENB, 180);

  currentMovement = "Turning Right";
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1;

  return (duration * 0.0343) / 2.0;
}

float stableDistance() {
  float readings[3];
  int valid = 0;

  for (int i = 0; i < 3; i++) {
    float distance = getDistance();

    if (distance > 0) {
      readings[valid++] = distance;
    }

    delay(40);
  }

  if (valid == 3) {
    if (readings[0] > readings[1]) {
      float t = readings[0];
      readings[0] = readings[1];
      readings[1] = t;
    }

    if (readings[1] > readings[2]) {
      float t = readings[1];
      readings[1] = readings[2];
      readings[2] = t;
    }

    if (readings[0] > readings[1]) {
      float t = readings[0];
      readings[0] = readings[1];
      readings[1] = t;
    }

    return readings[1];
  }

  if (valid == 2) {
    return (readings[0] + readings[1]) / 2.0;
  }

  return -1;
}

// =====================================================
// RELAY CONTROL
// Preserves your previously working relay behaviour.
// =====================================================

void relayOff() {
  pinMode(RELAY_PIN, INPUT);
}

void relayOn() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void operateWaterPump() {
  relayOn();

  if (!waitMode(500, MODE_AUTO)) return;

  relayOff();

  waitMode(500, MODE_AUTO);
}

// =====================================================
// DIGGING
// =====================================================

void performDigging(RobotMode requiredMode) {
  stopMotors();

  digServo.write(LEFT_POSITION);

  if (!waitMode(300, requiredMode)) return;

  for (int i = 0; i < DIG_CYCLES; i++) {
    digServo.write(RIGHT_POSITION);

    if (!waitMode(DIG_INTERVAL, requiredMode)) return;

    digServo.write(LEFT_POSITION);

    if (!waitMode(DIG_INTERVAL, requiredMode)) return;
  }

  digServo.write(LEFT_POSITION);
  waitMode(300, requiredMode);
}

// Manual digging avoids recursively calling server.handleClient()
// while the current /cmd request is already being processed.
void performManualDigging() {
  stopMotors();

  digServo.write(LEFT_POSITION);
  delay(300);

  for (int i = 0; i < DIG_CYCLES; i++) {
    digServo.write(RIGHT_POSITION);
    delay(DIG_INTERVAL);

    digServo.write(LEFT_POSITION);
    delay(DIG_INTERVAL);
  }

  digServo.write(LEFT_POSITION);
  delay(300);
}

// =====================================================
// SEED DISPENSER
// =====================================================

void dispenseSeeds(RobotMode requiredMode) {
  seedServo.write(SEED_OPEN_ANGLE);

  if (!waitMode(HOLD_OPEN_TIME, requiredMode)) return;

  seedServo.write(SEED_CLOSED_ANGLE);

  waitMode(HOLD_CLOSED_TIME, requiredMode);
}

// =====================================================
// ORIGINAL AUTOMATIC OBSTACLE AVOIDANCE
// =====================================================

void centerScannerAuto() {
  scanServo.write(CENTER_ANGLE);
  waitMode(500, MODE_AUTO);
}

void avoidObstacleAuto() {
  stopMotors();

  if (!waitMode(300, MODE_AUTO)) return;

  moveBackward();

  if (!waitMode(REVERSE_TIME, MODE_AUTO)) return;

  stopMotors();

  if (!waitMode(300, MODE_AUTO)) return;

  scanServo.write(RIGHT_SCAN);

  if (!waitMode(700, MODE_AUTO)) return;

  float right = stableDistance();

  scanServo.write(LEFT_SCAN);

  if (!waitMode(700, MODE_AUTO)) return;

  float left = stableDistance();

  centerScannerAuto();

  bool rightSafe = right >= MINIMUM_CLEARANCE;
  bool leftSafe = left >= MINIMUM_CLEARANCE;

  if (!rightSafe && !leftSafe) {
    moveBackward();

    if (!waitMode(REVERSE_EXTRA_TIME, MODE_AUTO)) return;

    stopMotors();
    waitMode(500, MODE_AUTO);
    return;
  }

  if (rightSafe && right > left) {
    turnRight();
  } else {
    turnLeft();
  }

  if (!waitMode(TURN_TIME, MODE_AUTO)) return;

  stopMotors();
  waitMode(300, MODE_AUTO);
}

// =====================================================
// AUTOMATIC FORWARD MOVEMENT
// =====================================================

void moveForward30cm() {
  forwardMovementTime = 0;

  moveForward();

  unsigned long forwardStart = millis();

  while (forwardMovementTime < FORWARD_30CM_MS) {
    server.handleClient();

    if (robotMode != MODE_AUTO) {
      stopMotors();
      return;
    }

    float distance = stableDistance();

    unsigned long currentTime = millis();

    forwardMovementTime += currentTime - forwardStart;
    forwardStart = currentTime;

    if (distance > 0 && distance <= OBSTACLE_DISTANCE) {
      stopMotors();

      avoidObstacleAuto();

      if (robotMode != MODE_AUTO) return;

      if (forwardMovementTime < FORWARD_30CM_MS) {
        moveForward();
        forwardStart = millis();
      }
    }
  }

  stopMotors();
}

// =====================================================
// AUTOMATIC CYCLE
// =====================================================

void automaticCycle() {
  if (robotMode != MODE_AUTO) return;

  moveForward30cm();

  if (robotMode != MODE_AUTO) return;

  stopMotors();

  if (!waitMode(500, MODE_AUTO)) return;

  performDigging(MODE_AUTO);

  if (robotMode != MODE_AUTO) return;

  stopMotors();

  if (!waitMode(500, MODE_AUTO)) return;

  dispenseSeeds(MODE_AUTO);

  if (robotMode != MODE_AUTO) return;

  stopMotors();

  if (!waitMode(500, MODE_AUTO)) return;

  operateWaterPump();

  if (robotMode != MODE_AUTO) return;

  forwardMovementTime = 0;

  waitMode(1000, MODE_AUTO);
}

// =====================================================
// MANUAL SCANNER
// It can turn the robot but never resumes forward motion.
// =====================================================

void resetManualScanner() {
  scannerState = SCAN_CENTER;
  scannerTimer = millis();

  scannerTurning = false;
  selectedTurn = 0;

  scanServo.write(CENTER_ANGLE);
}

void handleManualScanner() {
  if (!scannerEnabled || robotMode != MODE_MANUAL) return;

  unsigned long now = millis();

  // Complete the robot's turn, then return the scanner to centre.
  if (scannerState == SCAN_TURN) {
    if (now - scannerTimer >= TURN_TIME) {
      stopMotors();
      scannerTurning = false;
      scannerState = SCAN_RETURN;
      scannerTimer = now;
    }
    return;
  }

  if (scannerState == SCAN_RETURN) {
    scanServo.write(CENTER_ANGLE);

    if (now - scannerTimer >= 600) {
      scannerDirection = "Center";
      scannerState = SCAN_CENTER;
      scannerTimer = now;
    }
    return;
  }

  // Keep the scanner centred and check straight ahead.
  // Do not sweep or turn the robot when the path is clear.
  if (scannerState == SCAN_CENTER) {
    scanServo.write(CENTER_ANGLE);

    if (now - scannerTimer >= 250) {
      float front = stableDistance();

      if (front > 0 && front <= OBSTACLE_DISTANCE) {
        stopMotors();
        obstacleBlocked = true;
        obstacleStatus = "Obstacle detected";

        scanServo.write(RIGHT_SCAN);
        scannerDirection = "Scanning right";
        scannerState = SCAN_RIGHT;
        scannerTimer = millis();
      } else {
        obstacleBlocked = false;
        obstacleStatus = "Clear";
        scannerDirection = "Center";
        scannerTimer = millis();
      }
    }
    return;
  }

  if (scannerState == SCAN_RIGHT) {
    if (now - scannerTimer >= 650) {
      rightDistance = stableDistance();

      scanServo.write(LEFT_SCAN);
      scannerDirection = "Scanning left";
      scannerState = SCAN_LEFT;
      scannerTimer = millis();
    }
    return;
  }

  if (scannerState == SCAN_LEFT) {
    if (now - scannerTimer >= 650) {
      leftDistance = stableDistance();

      // Centre the scanner before turning the robot.
      scanServo.write(CENTER_ANGLE);

      bool rightSafe = rightDistance >= MINIMUM_CLEARANCE;
      bool leftSafe = leftDistance >= MINIMUM_CLEARANCE;

      if (!rightSafe && !leftSafe) {
        stopMotors();
        obstacleBlocked = true;
        obstacleStatus = "No safe direction";
        scannerState = SCAN_RETURN;
        scannerTimer = millis();
        return;
      }

      // Turn the ROBOT towards the side with more clearance.
      if (rightSafe && rightDistance > leftDistance) {
        selectedTurn = 1;
        scannerDirection = "Right";
        turnRight();
      } else {
        selectedTurn = -1;
        scannerDirection = "Left";
        turnLeft();
      }

      scannerTurning = true;
      scannerState = SCAN_TURN;
      scannerTimer = millis();
    }
  }
}

// Set a target for the soil probe servo; movement is stepped in loop().
void moveSoilServoSlowly(int targetAngle) {
  soilServoTargetAngle = constrain(targetAngle, SOIL_RAISED, SOIL_LOWERED);
}

// Move the soil probe one degree every 18 ms without blocking the web server.
void updateSoilServoMovement() {
  if (soilServoCurrentAngle == soilServoTargetAngle) return;
  if (millis() - lastSoilServoStep < 18) return;

  lastSoilServoStep = millis();

  if (soilServoCurrentAngle < soilServoTargetAngle) {
    soilServoCurrentAngle++;
  } else {
    soilServoCurrentAngle--;
  }

  soilServo.write(soilServoCurrentAngle);
}

// =====================================================
// SOIL MOISTURE
// =====================================================

int readSoilMoisture() {
  int rawValue = analogRead(SOIL_SENSOR_PIN);

  int percentage = map(
    rawValue,
    SOIL_DRY_VALUE,
    SOIL_WET_VALUE,
    0,
    100
  );

  return constrain(percentage, 0, 100);
}

// =====================================================
// MODE SWITCHING
// =====================================================

void handleModeChange() {
  stopMotors();
  manualDriveActive = false;
  lastManualDriveCommand = 0;

  relayOff();

  scannerEnabled = false;
  obstacleBlocked = false;
  scannerTurning = false;

  obstacleStatus = "Clear";
  currentMovement = "Stopped";

  resetManualScanner();
}

// =====================================================
// DASHBOARD HTML
// =====================================================

const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>AgriRobot Control</title>
<style>
:root {
  color-scheme: dark;
  --bg: #0b1511;
  --panel: #14231c;
  --panel2: #1b3026;
  --green: #72e0a0;
  --green-dark: #24784c;
  --text: #f2f7f3;
  --muted: #a8b9ae;
  --red: #b94343;
  --line: #294536;
}
* { box-sizing: border-box; }
body {
  font-family: Arial, sans-serif;
  background: radial-gradient(circle at top, #183326 0, var(--bg) 48%);
  color: var(--text);
  margin: 0;
  padding: 16px;
  max-width: 900px;
  margin-inline: auto;
}
header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  margin: 4px 2px 16px;
}
h1, h2, h3, p { margin-top: 0; }
header h1 { color: var(--green); font-size: 25px; margin: 0; }
.eyebrow { color: var(--muted); font-size: 12px; letter-spacing: 1.4px; text-transform: uppercase; }
.top-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
.card {
  background: linear-gradient(145deg, var(--panel2), var(--panel));
  border: 1px solid var(--line);
  padding: 16px;
  border-radius: 16px;
  margin-bottom: 13px;
  box-shadow: 0 8px 22px rgba(0,0,0,.16);
}
.card h3 { color: var(--green); font-size: 16px; margin-bottom: 12px; }
.moisture-value { font-size: 42px; line-height: 1; font-weight: 700; color: var(--green); margin: 10px 0; }
.moisture-unit { font-size: 18px; }
.subtle { color: var(--muted); font-size: 12px; margin-bottom: 0; }
.status { color: var(--green); font-weight: bold; }
.mode-buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; }
button {
  background: var(--green-dark);
  border: 1px solid rgba(255,255,255,.08);
  padding: 13px 10px;
  margin: 3px;
  border-radius: 10px;
  color: white;
  font-size: 14px;
  font-weight: 600;
  min-height: 46px;
  cursor: pointer;
  touch-action: none;
  user-select: none;
  -webkit-user-select: none;
  -webkit-tap-highlight-color: transparent;
}
button:active { filter: brightness(1.2); transform: scale(.98); }
.stop { background: var(--red); }
.off { background: #46534b; }
.grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 7px;
  max-width: 420px;
  margin: 0 auto;
}
.grid button { min-height: 58px; font-size: 15px; margin: 0; }
.grid .empty { visibility: hidden; }
.drive-card { text-align: center; }
.drive-card h3 { text-align: left; }
.drive-status { color: var(--muted); margin: 12px 0 0; font-size: 13px; }
.info-row { display: flex; justify-content: space-between; gap: 12px; padding: 7px 0; border-bottom: 1px solid var(--line); font-size: 14px; }
.info-row:last-child { border-bottom: 0; }
.action-buttons { display: flex; flex-wrap: wrap; gap: 6px; }
.action-buttons button { flex: 1 1 130px; }
@media (max-width: 560px) {
  body { padding: 12px; }
  .top-grid { grid-template-columns: 1fr 1fr; gap: 9px; }
  .card { padding: 13px; }
  .moisture-value { font-size: 35px; }
  .grid { gap: 6px; }
  .grid button { min-height: 54px; padding: 10px 4px; font-size: 13px; }
  .mode-buttons button { padding: 10px 5px; font-size: 12px; }
}
@media (max-width: 360px) {
  .top-grid { grid-template-columns: 1fr; }
}
</style>
</head>
<body>
<header>
  <div>
    <div class="eyebrow">Smart agricultural robot</div>
    <h1>AgriRobot</h1>
  </div>
  <div class="status" id="mode">MANUAL</div>
</header>

<div class="top-grid">
  <section class="card">
    <h3>Soil Moisture</h3>
    <div class="moisture-value"><span id="moisture">--</span><span class="moisture-unit">%</span></div>
    <p class="subtle">Raw value: <span id="raw">--</span></p>
  </section>

  <section class="card">
    <h3>Operating Mode</h3>
    <div class="mode-buttons">
      <button onclick="cmd('auto')">AUTO</button>
      <button onclick="cmd('manual')">MANUAL</button>
    </div>
    <p class="subtle" style="margin-top:12px">Current mode: <span id="modeLabel" class="status">MANUAL</span></p>
  </section>
</div>

<section class="card drive-card">
  <h3>Manual Driving Press and Hold</h3>
  <div class="grid">
    <div class="empty"></div>
    <button
      onpointerdown="startDrive(event,'forward')"
      onpointerup="stopDrive()"
      onpointerleave="stopDrive()"
      onpointercancel="stopDrive()">Forward</button>
    <div class="empty"></div>

    <button
      onpointerdown="startDrive(event,'left')"
      onpointerup="stopDrive()"
      onpointerleave="stopDrive()"
      onpointercancel="stopDrive()">Left</button>
    <button class="stop" onclick="stopDrive()">STOP</button>
    <button
      onpointerdown="startDrive(event,'right')"
      onpointerup="stopDrive()"
      onpointerleave="stopDrive()"
      onpointercancel="stopDrive()">Right</button>

    <div class="empty"></div>
    <button
      onpointerdown="startDrive(event,'backward')"
      onpointerup="stopDrive()"
      onpointerleave="stopDrive()"
      onpointercancel="stopDrive()">Backward</button>
    <div class="empty"></div>
  </div>
  <p class="drive-status">Movement: <span id="movement" class="status">Stopped</span></p>
</section>

<section class="card">
  <h3>Obstacle Scanner</h3>
  <div class="action-buttons">
    <button onclick="cmd('scanner_on')">Scanner ON</button>
    <button class="off" onclick="cmd('scanner_off')">Scanner OFF</button>
  </div>
  <div class="info-row"><span>Scanner</span><span id="scanner" class="status">OFF</span></div>
  <div class="info-row"><span>Obstacle</span><span id="obstacle">Clear</span></div>
  <div class="info-row"><span>Direction</span><span id="direction">Center</span></div>
</section>

<section class="card">
  <h3>Mechanisms</h3>
  <div class="action-buttons">
    <button onclick="cmd('dig')">Dig</button>
    <button onclick="cmd('seed')">Dispense Seeds</button>
    <button onclick="cmd('pump_on')">Pump ON</button>
    <button class="off" onclick="cmd('pump_off')">Pump OFF</button>
  </div>
</section>

<section class="card">
  <h3>Soil Probe</h3>
  <div class="action-buttons">
    <button onclick="cmd('soil_down')">Lower Probe</button>
    <button class="off" onclick="cmd('soil_up')">Raise Probe</button>
  </div>
</section>

<script>
function cmd(action) {
  fetch('/cmd?action=' + action)
    .catch(e => console.log(e));
}

let activeDrive = null;
let driveHeartbeat = null;

function sendDriveCommand() {
  if (activeDrive) {
    fetch('/cmd?action=drive_' + activeDrive)
      .catch(e => console.log(e));
  }
}

function startDrive(event, direction) {
  event.preventDefault();

  if (activeDrive) return;

  activeDrive = direction;

  if (event.currentTarget && event.pointerId !== undefined) {
    try {
      event.currentTarget.setPointerCapture(event.pointerId);
    } catch (e) {}
  }

  sendDriveCommand();
  driveHeartbeat = setInterval(sendDriveCommand, 180);
}

function stopDrive() {
  if (!activeDrive) return;

  activeDrive = null;

  if (driveHeartbeat) {
    clearInterval(driveHeartbeat);
    driveHeartbeat = null;
  }

  cmd('drive_stop');
}

document.addEventListener('pointerup', stopDrive);
document.addEventListener('pointercancel', stopDrive);
window.addEventListener('blur', stopDrive);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) stopDrive();
});

async function updateStatus() {
  try {
    const response = await fetch('/status');
    const s = await response.json();

    document.getElementById('mode').innerText = s.mode;
    document.getElementById('modeLabel').innerText = s.mode;
    document.getElementById('movement').innerText = s.movement;
    document.getElementById('scanner').innerText = s.scanner;
    document.getElementById('obstacle').innerText = s.obstacle;
    document.getElementById('direction').innerText = s.direction;
    document.getElementById('moisture').innerText = s.moisture;
    document.getElementById('raw').innerText = s.raw;
  } catch(e) {}
}

setInterval(updateStatus, 500);
updateStatus();
</script>
</body>
</html>
)rawliteral";

// =====================================================
// WEB COMMAND HANDLER
// =====================================================

void sendCommandResponse(String message) {
  server.send(200, "text/plain", message);
}

void handleCommand() {
  if (!server.hasArg("action")) {
    sendCommandResponse("Missing action");
    return;
  }

  String action = server.arg("action");

  // Mode selection
  if (action == "auto") {
    handleModeChange();
    robotMode = MODE_AUTO;

    sendCommandResponse("AUTO mode enabled");
    return;
  }

  if (action == "manual") {
    handleModeChange();
    robotMode = MODE_MANUAL;

    sendCommandResponse("MANUAL mode enabled");
    return;
  }

  if (robotMode != MODE_MANUAL) {
    sendCommandResponse("Switch to MANUAL mode first");
    return;
  }

  // Press-and-hold movement with a command timeout failsafe.
  if (action == "drive_stop") {
    manualDriveActive = false;
    stopMotors();
  }

  else if (action.startsWith("drive_")) {
    if (scannerTurning) {
      manualDriveActive = false;
      stopMotors();
      sendCommandResponse("Robot is turning");
      return;
    }

    String direction = action.substring(6);

    if (direction == "forward" && scannerEnabled && obstacleBlocked) {
      manualDriveActive = false;
      stopMotors();
      sendCommandResponse("Movement blocked by scanner");
      return;
    }

    if (direction == "forward") moveForward();
    else if (direction == "backward") moveBackward();
    else if (direction == "left") turnLeft();
    else if (direction == "right") turnRight();
    else {
      manualDriveActive = false;
      stopMotors();
      sendCommandResponse("Unknown drive command");
      return;
    }

    manualDriveActive = true;
    lastManualDriveCommand = millis();
  }

  // Legacy single-command movement actions retained for compatibility.
  else if (action == "stop") {
    manualDriveActive = false;
    stopMotors();
  }

  else if (action == "forward") {
    if (scannerTurning || (scannerEnabled && obstacleBlocked)) {
      stopMotors();
      sendCommandResponse("Movement blocked by scanner");
      return;
    }

    moveForward();
  }

  else if (action == "backward") {
    if (scannerTurning) {
      sendCommandResponse("Robot is turning");
      return;
    }

    moveBackward();
  }

  else if (action == "left") {
    if (scannerTurning) {
      sendCommandResponse("Robot is turning");
      return;
    }

    turnLeft();
  }

  else if (action == "right") {
    if (scannerTurning) {
      sendCommandResponse("Robot is turning");
      return;
    }

    turnRight();
  }

  // Scanner
  else if (action == "scanner_on") {
    scannerEnabled = true;
    obstacleBlocked = false;

    obstacleStatus = "Checking";
    scannerDirection = "Center";

    stopMotors();
    resetManualScanner();
  }

  else if (action == "scanner_off") {
    scannerEnabled = false;
    obstacleBlocked = false;
    scannerTurning = false;

    obstacleStatus = "Scanner disabled";
    scannerDirection = "Center";

    manualDriveActive = false;
    stopMotors();
    resetManualScanner();
  }

  // Mechanisms
  else if (action == "dig") {
    performManualDigging();
  }

  else if (action == "seed") {
    stopMotors();
    dispenseSeeds(MODE_MANUAL);
  }

  else if (action == "pump_on") {
    relayOn();
  }

  else if (action == "pump_off") {
    relayOff();
  }

  // Soil probe
  else if (action == "soil_down") {
    moveSoilServoSlowly(SOIL_LOWERED);
  }

  else if (action == "soil_up") {
    moveSoilServoSlowly(SOIL_RAISED);
  }

  sendCommandResponse("OK");
}

// =====================================================
// DASHBOARD STATUS
// =====================================================

void handleStatus() {
  int moisture = readSoilMoisture();
  int raw = analogRead(SOIL_SENSOR_PIN);

  String modeText =
    robotMode == MODE_AUTO ? "AUTO" : "MANUAL";

  String scannerText =
    scannerEnabled ? "ON" : "OFF";

  String json = "{";

  json += "\"mode\":\"" + modeText + "\",";
  json += "\"movement\":\"" + currentMovement + "\",";
  json += "\"scanner\":\"" + scannerText + "\",";
  json += "\"obstacle\":\"" + obstacleStatus + "\",";
  json += "\"direction\":\"" + scannerDirection + "\",";
  json += "\"moisture\":" + String(moisture) + ",";
  json += "\"raw\":" + String(raw);

  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Soil sensor
  analogReadResolution(12);
  pinMode(SOIL_SENSOR_PIN, INPUT);

  // Relay OFF
  relayOff();

 // PWM for ESP32 Core 3.x
// Reserve channels 14 and 15 for the motor driver.
// Keep these separate from ESP32Servo's PWM channels.
ledcAttachChannel(ENA, 1000, 8, 14);
ledcAttachChannel(ENB, 1000, 8, 15);
  stopMotors();

  // Servo timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Scanner servo
  scanServo.setPeriodHertz(50);
  scanServo.attach(SCAN_SERVO_PIN, 500, 2400);

  // Digging servo
  digServo.setPeriodHertz(50);
  digServo.attach(DIG_SERVO_PIN, 500, 2400);

  // Seed servo
  seedServo.setPeriodHertz(50);
  seedServo.attach(SEED_SERVO_PIN, 500, 2400);

  // Soil probe servo
  soilServo.setPeriodHertz(50);
  soilServo.attach(SOIL_SERVO_PIN, 500, 2400);

  // Initial positions
  scanServo.write(CENTER_ANGLE);
  digServo.write(LEFT_POSITION);
  seedServo.write(SEED_CLOSED_ANGLE);
  soilServo.write(SOIL_RAISED);
  soilServoCurrentAngle = SOIL_RAISED;
  soilServoTargetAngle = SOIL_RAISED;

  delay(1000);



  // Web routes
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", dashboardHTML);
  });

  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/status", HTTP_GET, handleStatus);

  server.begin();

  Serial.println("Web server started");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  server.handleClient();
  updateSoilServoMovement();

  // Stop if the phone stops sending hold-heartbeats or loses connection.
  if (robotMode == MODE_MANUAL &&
      manualDriveActive &&
      millis() - lastManualDriveCommand > MANUAL_DRIVE_TIMEOUT) {
    manualDriveActive = false;
    stopMotors();
  }

  if (robotMode == MODE_AUTO) {
    automaticCycle();
  }
  else {
    handleManualScanner();
  }
}