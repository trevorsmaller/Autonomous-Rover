#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ESP32 GPIO21 (TX) -> Uno D8 (RX); D11 TX is unused.
SoftwareSerial espSerial(8, 11);
Servo wheel1, wheel2, wheel3, wheel4, solarServo;

const byte WHEEL1_PIN = 4;
const byte WHEEL/Users/trevor/Downloads/green_following_rover_continuous_explore.ino2_PIN = 13;    // Unusual servo, previously on D5
const byte WHEEL3_PIN = 6;
const byte WHEEL4_PIN = 7;
const byte SOLAR_PIN = 9;
const byte BUZZER_PIN = 10;
const byte TRIG_PIN = 2;
const byte ECHO_PIN = 3;
const byte LDR_LEFT = A0;
const byte LDR_RIGHT = A1;

const int NORMAL_STOP_US = 1500;
const int WHEEL2_STOP_US = 1400;  // Calibrated neutral
const int SOLAR_TOLERANCE = 30;
const int SOLAR_STEP = 10;
const float WALL_CM = 10.0;
const float GREEN_FOUND_CM = 15.0;
const unsigned long CAMERA_TIMEOUT_MS = 400;
const unsigned long HAND_HOLD_MS = 2000;
const unsigned long START_DELAY_MS = 3000;
const unsigned long EXPLORE_FORWARD_MS = 2000;
const unsigned long REVERSE_MS = 1000;
const unsigned long AVOID_STOP_MS = 150;
const unsigned long GREEN_LOST_MS = 350;

// Camera sends S=no green, L=green left, R=green right, F=green centered.
char cameraCommand = 'S';
bool cameraEverSeen = false;
unsigned long lastCameraMs = 0;
unsigned long lastGreenMs = 0;
float distanceCm = -1;
unsigned long lastDistanceMs = 0;
unsigned long lastSolarMs = 0;
unsigned long lastStatusMs = 0;
int solarAngle = 90;

// EXPLORE and FOLLOW are the main movement modes; the others are phases.
enum RoverState {
  IDLE, START_DELAY, EXPLORE, FOLLOW, AVOID_STOP,
  AVOID_REVERSE, AVOID_TURN, FOUND, PAUSED
};
RoverState state = IDLE;
unsigned long stateSinceMs = 0;
unsigned long explorePhaseMs = 0;
unsigned long avoidTurnMs = 650;
bool exploreIsTurning = false;
bool turnLeftNext = true;
bool warningOn = false;
unsigned long warningToggleMs = 0;
bool handTiming = false;
unsigned long handStartMs = 0;
bool pauseReadyForNewGesture = false;
bool driveWasBlocked = false;
unsigned long noGreenSinceMs = 0;

void stopMotors() {
  wheel1.writeMicroseconds(NORMAL_STOP_US);
  wheel2.writeMicroseconds(WHEEL2_STOP_US);
  wheel3.writeMicroseconds(NORMAL_STOP_US);
  wheel4.writeMicroseconds(NORMAL_STOP_US);
}

void moveForward() {
  wheel1.write(180);
  wheel2.write(180);
  wheel3.write(180);
  wheel4.write(180);
}

void moveBackward() {
  // Verify these directions with the wheels lifted before driving on the floor.
  wheel1.write(0);
  wheel2.write(0);
  wheel3.write(0);
  wheel4.write(0);
}

void turnLeft() {
  // Forward-curving turn: left pair stops, right pair drives forward.
  wheel1.writeMicroseconds(NORMAL_STOP_US);
  wheel2.writeMicroseconds(WHEEL2_STOP_US);
  wheel3.write(180);
  wheel4.write(180);
}

void turnRight() {
  wheel1.write(180);
  wheel2.write(180);
  wheel3.writeMicroseconds(NORMAL_STOP_US);
  wheel4.writeMicroseconds(NORMAL_STOP_US);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

void readCamera() {
  while (espSerial.available()) {
    char incoming = (char)espSerial.read();
    if (incoming == 'S' || incoming == 'L' ||
        incoming == 'R' || incoming == 'F') {
      cameraCommand = incoming;
      lastCameraMs = millis();
      cameraEverSeen = true;
      if (incoming != 'S') lastGreenMs = lastCameraMs;
    }
  }
}

bool cameraFresh(unsigned long now) {
  return cameraEverSeen && now - lastCameraMs <= CAMERA_TIMEOUT_MS;
}

bool handClose() {
  return distanceCm >= 2.0f && distanceCm <= 5.5f;
}

void resetHandTimer() { handTiming = false; }

bool handHeldLongEnough(unsigned long now) {
  if (!handClose()) {
    resetHandTimer();
    return false;
  }
  if (!handTiming) {
    handTiming = true;
    handStartMs = now;
  }
  return now - handStartMs >= HAND_HOLD_MS;
}

bool avoiding() {
  return state == AVOID_STOP || state == AVOID_REVERSE ||
         state == AVOID_TURN;
}

void enterState(int next, unsigned long now) {
  RoverState target = static_cast<RoverState>(next);
  if (state == target) return;
  if (warningOn) {
    noTone(BUZZER_PIN);
    warningOn = false;
  }
  stopMotors();
  state = target;
  stateSinceMs = now;
  resetHandTimer();

  if (next == START_DELAY) {
    tone(BUZZER_PIN, 1700, 250);  // Start confirmation
  } else if (next == FOUND) {
    tone(BUZZER_PIN, 2300, 700);  // Distinct success sound
    noGreenSinceMs = 0;
    Serial.println(F("GREEN FOUND at about 15 cm!"));
  } else if (next == EXPLORE) {
    exploreIsTurning = false;
    explorePhaseMs = now;
  } else if (next == AVOID_TURN) {
    turnLeftNext = random(2) == 0;
    avoidTurnMs = random(500, 900); // Calibrate to your rover
  } else if (next == PAUSED) {
    pauseReadyForNewGesture = false;
    Serial.println(F("PAUSED: remove hand, then hold it near again to resume"));
  }
}

void updateSolar(unsigned long now) {
  if (now - lastSolarMs < 30) return;
  lastSolarMs = now;
  int difference = analogRead(LDR_LEFT) - analogRead(LDR_RIGHT);
  if (difference > SOLAR_TOLERANCE) solarAngle += SOLAR_STEP;
  else if (difference < -SOLAR_TOLERANCE) solarAngle -= SOLAR_STEP;
  solarAngle = constrain(solarAngle, 20, 160);
  solarServo.write(solarAngle);
}

void updateWarning(unsigned long now) {
  if (!avoiding()) {
    if (warningOn) {
      noTone(BUZZER_PIN);
      warningOn = false;
    }
    return;
  }
  if (now - warningToggleMs >= 250) {
    warningToggleMs = now;
    warningOn = !warningOn;
    if (warningOn) tone(BUZZER_PIN, 1500);
    else noTone(BUZZER_PIN);
  }
}

void explore(unsigned long now) {
  if (!exploreIsTurning && now - explorePhaseMs >= EXPLORE_FORWARD_MS) {
    exploreIsTurning = true;
    explorePhaseMs = now;
    turnLeftNext = random(2) == 0;
    avoidTurnMs = random(400, 900);
  } else if (exploreIsTurning && now - explorePhaseMs >= avoidTurnMs) {
    exploreIsTurning = false;
    explorePhaseMs = now;
  }

  if (!exploreIsTurning) moveForward();
  else if (turnLeftNext) turnLeft();
  else turnRight();
}

void printStatus(unsigned long now) {
  if (now - lastStatusMs < 1000) return;
  lastStatusMs = now;
  Serial.print(F("State="));
  switch (state) {
    case IDLE: Serial.print(F("IDLE")); break;
    case START_DELAY: Serial.print(F("START_DELAY")); break;
    case EXPLORE: Serial.print(F("EXPLORE")); break;
    case FOLLOW: Serial.print(F("FOLLOW_GREEN")); break;
    case AVOID_STOP: case AVOID_REVERSE: case AVOID_TURN:
      Serial.print(F("AVOID")); break;
    case FOUND: Serial.print(F("FOUND")); break;
    case PAUSED: Serial.print(F("PAUSED")); break;
  }
  Serial.print(F(" | Camera=")); Serial.print(cameraCommand);
  Serial.print(F(" | CamFresh=")); Serial.print(cameraFresh(now) ? F("YES") : F("NO"));
  Serial.print(F(" | Dist(cm)=")); Serial.print(distanceCm);
  Serial.print(F(" | Solar=")); Serial.println(solarAngle);
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  wheel1.attach(WHEEL1_PIN);
  wheel2.attach(WHEEL2_PIN);
  wheel3.attach(WHEEL3_PIN);
  wheel4.attach(WHEEL4_PIN);
  solarServo.attach(SOLAR_PIN);
  stopMotors();
  solarServo.write(solarAngle);
  randomSeed(analogRead(A2)); // A2 is unused in this wiring
  Serial.println(F("READY: hold hand about 4 cm away for 2 s to start"));
}

void loop() {
  unsigned long now = millis();
  readCamera();
  if (now - lastDistanceMs >= 80) {
    lastDistanceMs = now;
    distanceCm = readDistance();
  }
  updateSolar(now);
  updateWarning(now);
  printStatus(now);

  if (state == IDLE) {
    stopMotors();
    if (handHeldLongEnough(now)) enterState(START_DELAY, now);
    return;
  }

  if (state == START_DELAY) {
    stopMotors();
    // Wait 3 s and require the hand/obstacle to be removed.
    if (now - stateSinceMs >= START_DELAY_MS &&
        distanceCm > WALL_CM && cameraFresh(now)) {
      enterState(EXPLORE, now);
    }
    return;
  }

  if (state == PAUSED) {
    stopMotors();
    if (!pauseReadyForNewGesture) {
      if (distanceCm > 8.0f) pauseReadyForNewGesture = true;
    } else if (handHeldLongEnough(now)) {
      enterState(START_DELAY, now);
    }
    return;
  }

  if (state == FOUND) {
    stopMotors();
    // Stay parked while green is visible. Resume only after 1 s of S.
    if (cameraFresh(now) && cameraCommand == 'S') {
      if (noGreenSinceMs == 0) noGreenSinceMs = now;
      if (now - noGreenSinceMs >= 1000) enterState(EXPLORE, now);
    } else {
      noGreenSinceMs = 0;
    }
    return;
  }

  // Fail safe: no camera heartbeat or invalid distance means no motion.
  if (!cameraFresh(now) || distanceCm < 0) {
    stopMotors();
    driveWasBlocked = true;
    return;
  }
  if (driveWasBlocked) {
    driveWasBlocked = false;
    enterState(EXPLORE, now);
  }

  // A sustained close object is a hand-gesture candidate. Stop immediately.
  // One distance sensor cannot distinguish a hand from a wall.
  if (handClose()) {
    stopMotors();
    if (handHeldLongEnough(now)) enterState(PAUSED, now);
    return;
  }
  resetHandTimer();

  // Finish a wall-avoidance sequence before following new camera commands.
  if (state == AVOID_STOP) {
    stopMotors();
    if (now - stateSinceMs >= AVOID_STOP_MS)
      enterState(AVOID_REVERSE, now);
    return;
  }
  if (state == AVOID_REVERSE) {
    moveBackward();
    if (now - stateSinceMs >= REVERSE_MS)
      enterState(AVOID_TURN, now);
    return;
  }
  if (state == AVOID_TURN) {
    if (turnLeftNext) turnLeft();
    else turnRight();
    if (now - stateSinceMs >= avoidTurnMs)
      enterState(EXPLORE, now);
    return;
  }

  // Green centered and within 15 cm: stop and beep.
  // This assumes the ultrasonic echo came from the centered green object.
  if (cameraCommand == 'F' && distanceCm <= GREEN_FOUND_CM) {
    enterState(FOUND, now);
    return;
  }

  // Any other obstacle within 10 cm: avoid it.
  if (distanceCm <= WALL_CM) {
    enterState(AVOID_STOP, now);
    return;
  }

  if (cameraCommand == 'L' || cameraCommand == 'R' ||
      cameraCommand == 'F') {
    if (state != FOLLOW) enterState(FOLLOW, now);
    if (cameraCommand == 'L') turnLeft();
    else if (cameraCommand == 'R') turnRight();
    else moveForward();
  } else { // Camera S = no green, NOT a movement-stop command.
    if (state == FOLLOW) {
      stopMotors();
      if (now - lastGreenMs >= GREEN_LOST_MS)
        enterState(EXPLORE, now);
      return;
    }
    if (state != EXPLORE) enterState(EXPLORE, now);
    explore(now);
  }
}
