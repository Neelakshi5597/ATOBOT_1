// ============================================================
//  Position Control – Maxon Motor + ADS1261 (2x Load Cells)
//  Minimum Jerk Trajectory | Automated 0→80° Sweep (10° steps)
//  Records F1, F2, Tension at each hold position
//  Board: Teensy 4.1
//
//  PIN SUMMARY:
//  Motor:       PWM=8,   Enable=9,   Direction=10
//  Orange Enc:  A=25,    B=26
//  ADS1261:     CS=24,   DRDY=21,    RESET=20
//               MOSI=11, MISO=12,    SCK=13
//  Load Cell 1: AIN5(+) / AIN6(-)
//  Load Cell 2: AIN7(+) / AIN8(-)
// ============================================================

#include <SPI.h>
#include <Encoder.h>

// ============================================================
//  ADS1261 PIN DEFINITIONS
// ============================================================
#define CS0_PIN      24
#define DRDY0_PIN    21
#define RESET0_PIN   20
#define CAL_FACTOR   33255.01f

// Load cell channel definitions
#define LC1_AINP     5
#define LC1_AINN     6
#define LC2_AINP     7
#define LC2_AINN     8

// ============================================================
//  ORANGE ENCODER
// ============================================================
#define ENCODER_A 25
#define ENCODER_B 26

const int PPR            = 600;
const int COUNTS_PER_REV = PPR * 4;  // 2400 counts per revolution

Encoder orangeEncoder(ENCODER_A, ENCODER_B);

// ============================================================
//  MOTOR PINS
// ============================================================
const int PWMpin       = 8;
const int enablepin    = 9;
const int directionpin = 10;

// ============================================================
//  MOTOR CONTROL VARIABLES
// ============================================================
float currentJointAngle = 0.0f;
float Theta;
float Theta_prev;
float Theta_vel;
float PWM;
float error;
float t;
float t_prev;
float torque;

// ============================================================
//  MINIMUM JERK TRAJECTORY
// ============================================================
float traj_start_angle;
float traj_target_angle;
float traj_start_time;
float traj_duration;
bool  traj_active;
bool  traj_complete;
float traj_desired_pos;
float traj_desired_vel;
float traj_desired_acc;

// ============================================================
//  PD + FEEDFORWARD GAINS
// ============================================================
float Kp  = 4.0f;
float Kd  = 1.0f;
float Kff = 1.0f;

// ============================================================
//  PWM LIMITS
// ============================================================
const float PWM_MIN  = 410.0f;
const float PWM_MAX  = 800.0f;
const float DEADBAND = 0.005f;

// ============================================================
//  LOAD CELL VALUES
// ============================================================
float F1 = 0.0f;
float F2 = 0.0f;
float estimatedTension = 0.0f;

// ============================================================
//  AUTOMATED SEQUENCE CONFIGURATION
// ============================================================
const float HOLD_DURATION_SEC = 10.0f;   // Hold time at each position (seconds)
const float TRAJ_DURATION_SEC = 3.0f;   // Travel time between positions (seconds)
const float STEP_DEG          = 10.0f;  // Step size in degrees
const float MAX_DEG           = 80.0f;  // Maximum angle in degrees

// Waypoints: 0, 10, 20, ..., 80, 70, ..., 0  (17 waypoints)
const int NUM_WAYPOINTS = 17;
float waypoints[NUM_WAYPOINTS];          // filled in setup()

// ============================================================
//  STATE MACHINE
// ============================================================
enum State {
  WAITING_FOR_ZERO,
  RUNNING_SEQUENCE,
  SEQUENCE_DONE
};

State currentState = WAITING_FOR_ZERO;
unsigned long lastDisplayTime = 0;

// Sequence tracking
int           seqIndex     = 0;
bool          seqHolding   = false;
unsigned long holdStartTime = 0;

// ============================================================
//  RECORDED DATA PER WAYPOINT
// ============================================================
float recordedAngle[NUM_WAYPOINTS];
float recordedF1   [NUM_WAYPOINTS];
float recordedF2   [NUM_WAYPOINTS];
float recordedT    [NUM_WAYPOINTS];
float recordedPWM  [NUM_WAYPOINTS];

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void  ADS1261_init();
float ADS1261_readChannel(uint8_t ainp, uint8_t ainn, float lastVal);
void  writeReg(uint8_t reg, uint8_t val);
float joint_angle();
void  startMinimumJerkTrajectory(float start_pos, float target_pos, float duration);
void  calculateMinimumJerkTrajectory(float tau);
float computeTension(float f1, float f2);
void  printDataTable();

// ============================================================
//  ADS1261 HELPERS
// ============================================================
void writeReg(uint8_t reg, uint8_t val) {
  digitalWrite(CS0_PIN, LOW);
  SPI.transfer((uint8_t)(0x40 | reg));
  SPI.transfer(val);
  digitalWrite(CS0_PIN, HIGH);
}

void ADS1261_init() {
  pinMode(CS0_PIN,    OUTPUT);
  pinMode(DRDY0_PIN,  INPUT);
  pinMode(RESET0_PIN, OUTPUT);

  digitalWrite(CS0_PIN,    HIGH);
  digitalWrite(RESET0_PIN, HIGH);
  digitalWrite(RESET0_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(RESET0_PIN, HIGH);
  delay(50);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  writeReg(0x02, 0x48);  // CONFIG1 – gain / PGA
  writeReg(0x03, 0x01);  // CONFIG2 – data rate
  writeReg(0x05, 0x00);  // CONFIG4
  writeReg(0x06, 0x10);  // CONFIG5
  writeReg(0x10, 0x05);  // MUX – start with LC1
  writeReg(0x11, (uint8_t)(((LC1_AINP & 0x0F) << 4) | (LC1_AINN & 0x0F)));

  digitalWrite(CS0_PIN, LOW);
  SPI.transfer((uint8_t)0x08);  // START conversion
  digitalWrite(CS0_PIN, HIGH);

  SPI.endTransaction();
}

float ADS1261_readChannel(uint8_t ainp, uint8_t ainn, float lastVal) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  writeReg(0x11, (uint8_t)(((ainp & 0x0F) << 4) | (ainn & 0x0F)));

  unsigned long tStart = millis();
  while (digitalRead(DRDY0_PIN) == LOW) {
    if (millis() - tStart > 50) { SPI.endTransaction(); return lastVal; }
  }
  while (digitalRead(DRDY0_PIN) != LOW) {
    if (millis() - tStart > 50) { SPI.endTransaction(); return lastVal; }
  }

  digitalWrite(CS0_PIN, LOW);
  SPI.transfer((uint8_t)0x12);           // RDATA command
  (void)SPI.transfer((uint8_t)0x00);    // status byte (discard)
  uint8_t b2 = SPI.transfer((uint8_t)0x00);
  uint8_t b1 = SPI.transfer((uint8_t)0x00);
  uint8_t b0 = SPI.transfer((uint8_t)0x00);
  digitalWrite(CS0_PIN, HIGH);

  SPI.endTransaction();

  int32_t raw = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | (int32_t)b0;
  if (raw & 0x800000) raw |= 0xFF000000;

  return (float)raw / CAL_FACTOR;
}

// ============================================================
//  TENSION ESTIMATION
//  T = 1.75 * (F1 + F2)
// ============================================================
float computeTension(float f1, float f2) {
  return 1.75f * (f1 + f2);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Build waypoints: 0 → 80 in steps of 10, then 70 → 0
  for (int i = 0; i <= 8; i++) waypoints[i]      = i * STEP_DEG;         // 0..80
  for (int i = 1; i <= 8; i++) waypoints[8 + i]  = (8 - i) * STEP_DEG;  // 70..0

  // ADS1261 load cell amplifier
  ADS1261_init();

  // Motor
  pinMode(enablepin,    OUTPUT);
  pinMode(directionpin, OUTPUT);
  pinMode(PWMpin,       OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  digitalWrite(enablepin, LOW);

  analogReadResolution(12);
  analogWriteResolution(12);

  // Initialize trajectory variables
  traj_active   = false;
  traj_complete = false;
  traj_duration = TRAJ_DURATION_SEC;

  t_prev     = millis() / 1000.0f;
  Theta      = joint_angle() * 3.14159265359f / 180.0f;
  Theta_prev = Theta;
  Theta_vel  = 0.0f;

  traj_desired_pos = Theta;
  traj_desired_vel = 0.0f;
  traj_desired_acc = 0.0f;

  Serial.println("=== Automated Angle Sweep: 0 -> 80 -> 0 degrees ===");
  Serial.println("Load cell amplifier: ADS1261 (dual channel)");
  Serial.println();
  Serial.print("Current joint angle: ");
  Serial.print(joint_angle(), 2);
  Serial.println(" degrees");
  Serial.println();
  Serial.println("Step 1: Move shaft to desired zero position,");
  Serial.println("        then enter 'z' or 'Z' to set zero.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long currentTime = millis();

  switch (currentState) {

    // ========== STATE 1: WAITING FOR ZERO ==========
    case WAITING_FOR_ZERO:
      if (currentTime - lastDisplayTime >= 500) {
        Serial.print("Current joint angle: ");
        Serial.print(joint_angle(), 2);
        Serial.println(" degrees");
        lastDisplayTime = currentTime;
      }

      if (Serial.available() > 0) {
        char input = Serial.read();
        while (Serial.available() > 0) Serial.read();  // flush

        if (input == 'z' || input == 'Z') {
          orangeEncoder.write(0);

          Serial.println();
          Serial.println("Zero position set!");
          Serial.println("Starting automated sweep sequence...");
          Serial.println();
          Serial.println("Step | Target(deg) | Desired(deg) | Actual(deg) | PWM");
          Serial.println("----------------------------------------------------------");

          Theta = joint_angle() * 3.14159265359f / 180.0f;
          float target_rad = waypoints[0] * 3.14159265359f / 180.0f;
          startMinimumJerkTrajectory(Theta, target_rad, TRAJ_DURATION_SEC);

          seqIndex   = 0;
          seqHolding = false;
          currentState = RUNNING_SEQUENCE;
        }
      }
      break;

    // ========== STATE 2: RUNNING AUTOMATED SEQUENCE ==========
    case RUNNING_SEQUENCE:
    {
      t = millis() / 1000.0f;
      float dt = t - t_prev;
      if (dt < 0.001f) dt = 0.001f;

      // Read current angle
      Theta = joint_angle() * 3.14159265359f / 180.0f;

      // Velocity estimation with low-pass filter
      float vel_raw = (Theta - Theta_prev) / dt;
      Theta_vel = 0.7f * Theta_vel + 0.3f * vel_raw;

      // ---- Trajectory update ----
      if (traj_active) {
        float elapsed = t - traj_start_time;
        if (elapsed >= traj_duration) {
          traj_active      = false;
          traj_complete    = true;
          traj_desired_pos = traj_target_angle;
          traj_desired_vel = 0.0f;
          traj_desired_acc = 0.0f;
        } else {
          calculateMinimumJerkTrajectory(elapsed);
        }
      }

      // ---- Position error & PD control ----
      error = traj_desired_pos - Theta;
      float velocity_error  = traj_desired_vel - Theta_vel;
      float control_signal  = Kp * error + Kd * velocity_error + Kff * traj_desired_acc;

      // Suppress output when trajectory done and near target
      if (traj_complete && abs(error) < DEADBAND) {
        control_signal = 0.0f;
      }

      // Apply PWM
      if (abs(control_signal) < 0.01f) {
        digitalWrite(enablepin, LOW);
        PWM = 0.0f;
      } else {
        PWM = constrain(PWM_MIN + 311.607f * abs(control_signal), PWM_MIN, PWM_MAX);
        digitalWrite(enablepin, HIGH);
        analogWrite(PWMpin, (int)PWM);
        digitalWrite(directionpin, (control_signal > 0) ? LOW : HIGH);
      }

      // ---- Read load cells (always running) ----
      F1 = ADS1261_readChannel(LC1_AINP, LC1_AINN, F1);
      F2 = ADS1261_readChannel(LC2_AINP, LC2_AINN, F2);
      estimatedTension = computeTension(F1, F2);

      // ---- Sequence logic ----
      if (!seqHolding) {
        // Travelling to current waypoint – print live progress every 50 ms
        if (currentTime - lastDisplayTime >= 50) {
          Serial.print("  -->  ");
          Serial.print(seqIndex);
          Serial.print("   |  ");
          Serial.print(waypoints[seqIndex], 1);
          Serial.print("          |  ");
          Serial.print(traj_desired_pos * 180.0f / 3.14159265359f, 2);
          Serial.print("        |  ");
          Serial.print(Theta * 180.0f / 3.14159265359f, 2);
          Serial.print("       |  ");
          Serial.println((int)PWM);
          lastDisplayTime = currentTime;
        }

        // Check if we've arrived (trajectory complete AND settled, ~3 deg)
        if (traj_complete && abs(error) < 0.05f) {
          seqHolding    = true;
          holdStartTime = currentTime;

          // Record data at this waypoint
          recordedAngle[seqIndex] = Theta * 180.0f / 3.14159265359f;
          recordedF1   [seqIndex] = F1;
          recordedF2   [seqIndex] = F2;
          recordedT    [seqIndex] = estimatedTension;
          recordedPWM  [seqIndex] = PWM;

          Serial.println();
          Serial.print("[HOLD] Waypoint ");
          Serial.print(seqIndex);
          Serial.print("  Target: ");
          Serial.print(waypoints[seqIndex], 1);
          Serial.print(" deg  |  Actual: ");
          Serial.print(recordedAngle[seqIndex], 2);
          Serial.print(" deg  |  PWM: ");
          Serial.print((int)recordedPWM[seqIndex]);
          Serial.print("  |  F1: ");
          Serial.print(recordedF1[seqIndex], 4);
          Serial.print("  F2: ");
          Serial.print(recordedF2[seqIndex], 4);
          Serial.print("  T: ");
          Serial.print(recordedT[seqIndex], 3);
          Serial.println(" N");
        }

      } else {
        // Holding at current waypoint – keep sampling load cells every 100 ms
        unsigned long elapsed_hold = currentTime - holdStartTime;

        if (currentTime - lastDisplayTime >= 500) {
          // Continuously update recorded values with latest readings during hold
          recordedF1[seqIndex] = F1;
          recordedF2[seqIndex] = F2;
          recordedT [seqIndex] = estimatedTension;

          Serial.print("  Holding... ");
          Serial.print((HOLD_DURATION_SEC * 1000UL - elapsed_hold) / 1000.0f, 1);
          Serial.print(" s  |  F1: ");
          Serial.print(F1, 4);
          Serial.print("  F2: ");
          Serial.print(F2, 4);
          Serial.print("  T: ");
          Serial.print(estimatedTension, 3);
          Serial.println(" N");
          lastDisplayTime = currentTime;
        }

        // After hold duration, advance to next waypoint
        if (elapsed_hold >= (unsigned long)(HOLD_DURATION_SEC * 1000.0f)) {
          seqIndex++;

          if (seqIndex >= NUM_WAYPOINTS) {
            // All waypoints complete
            digitalWrite(enablepin, LOW);
            currentState = SEQUENCE_DONE;
            Serial.println();
            Serial.println("Sequence complete! Motor disabled.");
            printDataTable();
          } else {
            seqHolding = false;
            Theta = joint_angle() * 3.14159265359f / 180.0f;
            float next_rad = waypoints[seqIndex] * 3.14159265359f / 180.0f;
            startMinimumJerkTrajectory(Theta, next_rad, TRAJ_DURATION_SEC);

            Serial.println();
            Serial.print("Moving to waypoint ");
            Serial.print(seqIndex);
            Serial.print(": ");
            Serial.print(waypoints[seqIndex], 1);
            Serial.println(" degrees");
            Serial.println("Step | Target(deg) | Desired(deg) | Actual(deg) | PWM");
            Serial.println("----------------------------------------------------------");
          }
        }
      }

      t_prev     = t;
      Theta_prev = Theta;
      delay(10);  // ~100 Hz
      break;
    }

    // ========== STATE 3: SEQUENCE DONE ==========
    case SEQUENCE_DONE:
      if (Serial.available() > 0) {
        char c = Serial.read();
        while (Serial.available() > 0) Serial.read();
        if (c == 'r' || c == 'R') {
          currentState = WAITING_FOR_ZERO;
          Serial.println();
          Serial.println("=== RESTARTING ===");
          Serial.println("Step 1: Enter 'z' or 'Z' to set zero position.");
        }
      }
      break;
  }
}

// ============================================================
//  PRINT FINAL RECORDED DATA TABLE
// ============================================================
void printDataTable() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("              RECORDED DATA TABLE                          ");
  Serial.println("============================================================");
  Serial.println("Idx | Target(deg) | Actual(deg) | PWM  |   F1     |   F2     | Tension(N)");
  Serial.println("----+-------------+-------------+------+----------+----------+-----------");
  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    Serial.print("  ");
    if (i < 10) Serial.print(" ");
    Serial.print(i);
    Serial.print("  |  ");
    Serial.print(waypoints[i], 1);
    Serial.print("        |  ");
    Serial.print(recordedAngle[i], 2);
    Serial.print("       |  ");
    Serial.print((int)recordedPWM[i]);
    Serial.print("  |  ");
    Serial.print(recordedF1[i], 4);
    Serial.print("  |  ");
    Serial.print(recordedF2[i], 4);
    Serial.print("  |  ");
    Serial.println(recordedT[i], 3);
  }
  Serial.println("============================================================");
  Serial.println();
  Serial.println("Enter 'r' or 'R' to restart.");
}

// ============================================================
//  JOINT ANGLE FROM ENCODER (degrees, CW positive)
// ============================================================
float joint_angle() {
  long newCount   = -orangeEncoder.read();
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0f / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0f;
  return currentJointAngle;
}

// ============================================================
//  TRAJECTORY HELPERS
// ============================================================
void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle  = start_pos;
  traj_target_angle = target_pos;
  traj_start_time   = millis() / 1000.0f;
  traj_duration     = duration;
  traj_active       = true;
  traj_complete     = false;
}

void calculateMinimumJerkTrajectory(float tau) {
  float T  = traj_duration;
  float q0 = traj_start_angle;
  float qf = traj_target_angle;

  float s  = constrain(tau / T, 0.0f, 1.0f);
  float s2 = s * s;
  float s3 = s2 * s;
  float s4 = s3 * s;
  float s5 = s4 * s;

  traj_desired_pos = q0 + (qf - q0) * (10.0f*s3 - 15.0f*s4 + 6.0f*s5);
  traj_desired_vel = (qf - q0) / T       * (30.0f*s2 - 60.0f*s3 + 30.0f*s4);
  traj_desired_acc = (qf - q0) / (T * T) * (60.0f*s  - 180.0f*s2 + 120.0f*s3);
}
