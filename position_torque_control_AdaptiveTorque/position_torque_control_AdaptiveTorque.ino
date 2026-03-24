// ============================================================
//  Position Control – Maxon Motor + ADS1261 (2x Load Cells)
//  Minimum Jerk Trajectory + Tension-Responsive Angle Descent
//  Board: Teensy 4.1
//
//  PIN SUMMARY:
//  Motor:       PWM=8,   Enable=9,   Direction=10
//  Orange Enc:  A=25,    B=26
//  ADS1261:     CS=24,   DRDY=21,    RESET=20
//               MOSI=11, MISO=12,    SCK=13
//  Load Cell 1: AIN5(+) / AIN6(-)
//  Load Cell 2: AIN7(+) / AIN8(-)
//  Linear Act.: PWM=5,   DIR1=3,     DIR2=4,  Feedback=A0
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
const int COUNTS_PER_REV = PPR * 4;

Encoder orangeEncoder(ENCODER_A, ENCODER_B);

// ============================================================
//  MOTOR PINS
// ============================================================
const int MOTOR_PWM_PIN    = 8;
const int MOTOR_ENABLE_PIN = 9;
const int MOTOR_DIR_PIN    = 10;

// ============================================================
//  MOTOR CONTROL VARIABLES
// ============================================================
float currentJointAngle = 0.0;
float Theta;
float Theta_prev;
float Theta_vel;
float motor_PWM;
float motor_error;
float t;
float t_prev;

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
//  CABLE / DIRECTION TRACKING
// ============================================================
int  cable_pulling_direction = 0;
bool holding_mode            = false;

// ============================================================
//  SMOOTH PWM TRANSITION
// ============================================================
float       target_PWM         = 0;
float       current_PWM_smooth = 0;
const float PWM_RAMP_RATE      = 600.0f;

// ============================================================
//  PD + FEEDFORWARD GAINS
// ============================================================
float Kp  = 7.0f;
float Kd  = 1.8f;
float Kff = 1.5f;
float Ki  = 0.20f;
float error_integral = 0.0f;

// ============================================================
//  PWM LIMITS
// ============================================================
const float HOLDING_MIN_PWM = 550.0f;
const float HOLDING_MAX_PWM = 1000.0f;
const float PWM_MIN         = 410.0f;
const float PWM_MAX         = 1200.0f;

// ============================================================
//  LOAD CELL VALUES
// ============================================================
float F1 = 0.0f;
float F2 = 0.0f;
float estimatedTension     = 0.0f;
float prevEstimatedTension = 0.0f;

// ============================================================
//  TENSION-RESPONSIVE DESCENT PARAMETERS
// ============================================================


const float TENSION_RISE_THRESHOLD  = 0.5f;   // N – how much tension must rise before descent starts
const float ANGLE_STEP_DEG          = 0.3f;   // degrees to decrease per interval
const unsigned long DESCENT_INTERVAL_MS = 200; // ms between each step-down
const float DESCENT_MIN_ANGLE_DEG   = 0.0f;   // hard floor – never go below this

bool  tension_descent_active   = false;
unsigned long lastDescentTime  = 0;
float tension_at_hold_start    = 0.0f;       // baseline tension recorded when hold began

// ============================================================
//  LINEAR ACTUATOR
// ============================================================
const int LA_PWM_PIN      = 5;
const int LA_DIR_PIN1     = 3;
const int LA_DIR_PIN2     = 4;
const int LA_FEEDBACK_PIN = A0;

int       LA_TARGET_POSITION = 0;
const int LA_TOLERANCE       = 10;
const int LA_MOTOR_SPEED     = 3200;
int       LA_zeroOffset      = 0;

// ============================================================
//  STATE MACHINE
// ============================================================
enum State {
  WAITING_FOR_ZERO,
  WAITING_FOR_TARGETS,
  RUNNING_MOTION
};

State         currentState  = WAITING_FOR_ZERO;
float         target_angle_deg  = 0.0f;
int           target_position_LA = 0;
unsigned long lastDisplayTime   = 0;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void  ADS1261_init();
float ADS1261_readChannel(uint8_t ainp, uint8_t ainn, float lastVal);
void  writeReg(uint8_t reg, uint8_t val);
float joint_angle();
void  startMinimumJerkTrajectory(float start_pos, float target_pos, float duration);
void  calculateMinimumJerkTrajectory(float tau);
bool  parseTargets();
void  extendLA(int speed);
void  retractLA(int speed);
void  stopLA();
float computeTension(float f1, float f2);

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

  // Switch MUX to requested channel
  writeReg(0x11, (uint8_t)(((ainp & 0x0F) << 4) | (ainn & 0x0F)));

  // Wait for DRDY to pulse LOW then HIGH (new conversion ready)
  unsigned long tStart = millis();
  while (digitalRead(DRDY0_PIN) == LOW)  {
    if (millis() - tStart > 50) { SPI.endTransaction(); return lastVal; }
  }
  while (digitalRead(DRDY0_PIN) != LOW) {
    if (millis() - tStart > 50) { SPI.endTransaction(); return lastVal; }
  }

  // Read 24-bit result
  digitalWrite(CS0_PIN, LOW);
  SPI.transfer((uint8_t)0x12);           // RDATA command
  (void)SPI.transfer((uint8_t)0x00);    // status byte (discard)
  uint8_t b2 = SPI.transfer((uint8_t)0x00);
  uint8_t b1 = SPI.transfer((uint8_t)0x00);
  uint8_t b0 = SPI.transfer((uint8_t)0x00);
  digitalWrite(CS0_PIN, HIGH);

  SPI.endTransaction();

  // Sign-extend 24-bit to 32-bit
  int32_t raw = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | (int32_t)b0;
  if (raw & 0x800000) raw |= 0xFF000000;

  return (float)raw / CAL_FACTOR;
}

// ============================================================
//  TENSION ESTIMATION
//  T = 69.902 × F1^0.2005 × F2^0.4826 − 165.998  (N)
//  T = (F1 + F2) × 0.0175 / 0.01  =  1.75 × (F1 + F2)
// ============================================================
float computeTension(float f1, float f2) {
  // Guard against zero / negative values before pow()
  float absF1 = abs(f1);
  float absF2 = abs(f2);
  if (absF1 < 1e-6f) absF1 = 1e-6f;
  if (absF2 < 1e-6f) absF2 = 1e-6f;

  //float T = 69.902f * pow(absF1, 0.2005f) * pow(absF2, 0.4826f) - 165.998f;
  float T = 1.75 * (F1 + F2);
  return T;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // --- ADS1261 ---
  ADS1261_init();

  // --- Motor ---
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN,    OUTPUT);
  pinMode(MOTOR_PWM_PIN,    OUTPUT);
  pinMode(ENCODER_A,        INPUT_PULLUP);
  pinMode(ENCODER_B,        INPUT_PULLUP);

  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWriteResolution(12);

  traj_active   = false;
  traj_complete = false;
  traj_duration = 1.8f;

  t_prev     = millis() / 1000.0f;
  Theta      = joint_angle() * 3.14159265359f / 180.0f;
  Theta_prev = Theta;
  Theta_vel  = 0.0f;

  traj_desired_pos = Theta;
  traj_desired_vel = 0.0f;
  traj_desired_acc = 0.0f;

  // --- Linear Actuator ---
  pinMode(LA_PWM_PIN,      OUTPUT);
  pinMode(LA_DIR_PIN1,     OUTPUT);
  pinMode(LA_DIR_PIN2,     OUTPUT);
  pinMode(LA_FEEDBACK_PIN, INPUT);

  stopLA();

  Serial.println("=== System Ready ===");
  Serial.println("Enter 'z' to set encoder/LA zero position.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  unsigned long currentTime = millis();

  switch (currentState) {

    // ----------------------------------------------------------
    case WAITING_FOR_ZERO:
      if (Serial.available() > 0) {
        char input = Serial.read();
        while (Serial.available() > 0) Serial.read();

        if (input == 'z' || input == 'Z') {
          orangeEncoder.write(0);
          LA_zeroOffset = analogRead(LA_FEEDBACK_PIN);

          currentState = WAITING_FOR_TARGETS;
          Serial.println("Zero set. Enter targets as:  angle_deg , LA_position");
          Serial.println("(e.g.  45,200   or   N,300 to keep current angle)");
        }
      }
      break;

    // ----------------------------------------------------------
    case WAITING_FOR_TARGETS:
      if (Serial.available() > 0) {
        bool ok = parseTargets();
        if (ok) {
          currentState = RUNNING_MOTION;
          Serial.println("Target | Actual | Error | PWM | F1 | F2 | Tension | Status");
        } else {
          Serial.println("Invalid input. Format: angle,position");
        }
      }
      break;

    // ----------------------------------------------------------
    case RUNNING_MOTION:
    {
      // ---- TIME ----
      t = millis() / 1000.0f;
      float dt = t - t_prev;
      if (dt < 0.001f) dt = 0.001f;

      // ---- ENCODER ----
      Theta = joint_angle() * 3.14159265359f / 180.0f;
      float vel_raw = (Theta - Theta_prev) / dt;
      Theta_vel = 0.7f * Theta_vel + 0.3f * vel_raw;

      // ---- TRAJECTORY ----
      if (traj_active) {
        float elapsed_time = t - traj_start_time;

        if (elapsed_time >= traj_duration) {
          traj_active      = false;
          traj_complete    = true;
          holding_mode     = true;
          traj_desired_pos = traj_target_angle;
          traj_desired_vel = 0.0f;
          traj_desired_acc = 0.0f;

          // Record baseline tension at the moment hold starts
          tension_at_hold_start  = estimatedTension;
          prevEstimatedTension   = estimatedTension;
          tension_descent_active = false;
          lastDescentTime       = currentTime;

          Serial.println(">>> Trajectory complete – entering hold mode.");
          Serial.println(">>> Monitoring torque for gradual descent...");
        } else {
          calculateMinimumJerkTrajectory(elapsed_time);
        }
      }

      // ---- LOAD CELLS & TENSION ----
      F1 = ADS1261_readChannel(LC1_AINP, LC1_AINN, F1);
      F2 = ADS1261_readChannel(LC2_AINP, LC2_AINN, F2);
      estimatedTension = computeTension(F1, F2);

      // ---- TENSION-RESPONSIVE DESCENT (active only in hold mode) ----
      if (holding_mode && traj_complete) {
        float tensionRise = estimatedTension - tension_at_hold_start;

        if (tensionRise > TENSION_RISE_THRESHOLD) {
          tension_descent_active = true;
        }

        if (tension_descent_active &&
            (currentTime - lastDescentTime >= DESCENT_INTERVAL_MS)) {

          float currentTargetDeg = traj_target_angle * 180.0f / 3.14159265359f;

          if (currentTargetDeg > DESCENT_MIN_ANGLE_DEG) {
            float newTargetDeg = currentTargetDeg - ANGLE_STEP_DEG;
            if (newTargetDeg < DESCENT_MIN_ANGLE_DEG) {
              newTargetDeg = DESCENT_MIN_ANGLE_DEG;
            }

            float newTargetRad = newTargetDeg * 3.14159265359f / 180.0f;

            traj_desired_pos  = newTargetRad;
            traj_target_angle = newTargetRad;

            float holdError = traj_desired_pos - Theta;
            if (abs(holdError) > 0.02f) {
              cable_pulling_direction = (holdError > 0) ? 1 : -1;
            }
          }

          lastDescentTime = currentTime;
        }
      }

      // ---- MOTOR CONTROL ----
      motor_error = traj_desired_pos - Theta;
      float velocity_error = traj_desired_vel - Theta_vel;
      float error_deg      = motor_error * 180.0f / 3.14159265359f;
      float abs_error_deg  = abs(error_deg);

      // Direction locked per cable pull direction
      if (cable_pulling_direction > 0) {
        digitalWrite(MOTOR_DIR_PIN, LOW);
      } else {
        digitalWrite(MOTOR_DIR_PIN, HIGH);
      }

      // Integral with anti-windup
      if (abs_error_deg > 0.3f) {
        error_integral += motor_error * dt;
        error_integral  = constrain(error_integral, -1.0f, 1.0f);
      } else {
        error_integral *= 0.95f;
      }

      float p_term  = Kp  * motor_error;
      float i_term  = Ki  * error_integral;
      float d_term  = Kd  * velocity_error;
      float ff_term = Kff * traj_desired_acc;

      float control_signal = p_term + i_term + d_term + ff_term;

      // PWM target calculation
      if (motor_error > 0) {
        float additional = abs(control_signal) * 250.0f;
        target_PWM = HOLDING_MIN_PWM + additional;
        target_PWM = constrain(target_PWM, HOLDING_MIN_PWM, HOLDING_MAX_PWM);
      } else {
        float reduction_factor;
        if      (abs_error_deg > 5.0f)  reduction_factor = 0.45f;
        else if (abs_error_deg > 2.0f)  reduction_factor = 0.60f;
        else if (abs_error_deg > 1.0f)  reduction_factor = 0.75f;
        else                            reduction_factor = 0.88f;
        target_PWM = HOLDING_MIN_PWM * reduction_factor;
      }

      // Velocity damping
      float vel_deg_s = Theta_vel * 180.0f / 3.14159265359f;
      if (abs(vel_deg_s) > 1.0f) {
        if ((motor_error > 0 && Theta_vel < 0) || (motor_error < 0 && Theta_vel > 0)) {
          target_PWM += abs(vel_deg_s) * 20.0f;
        }
      }

      target_PWM = constrain(target_PWM, HOLDING_MIN_PWM * 0.4f, HOLDING_MAX_PWM);

      // Smooth ramping with anti-jerk at trajectory start
      float pwm_error   = target_PWM - current_PWM_smooth;
      float max_change  = PWM_RAMP_RATE * dt;

      if (traj_active) {
        float elapsed_time = t - traj_start_time;
        if (elapsed_time < 0.15f) max_change *= 0.6f;
      }

      if (abs(pwm_error) > max_change) {
        current_PWM_smooth += (pwm_error > 0) ? max_change : -max_change;
      } else {
        current_PWM_smooth = target_PWM;
      }

      motor_PWM = current_PWM_smooth;

      // Minimal hold PWM when very close and nearly stationary
      if (abs_error_deg < 0.2f && abs(vel_deg_s) < 0.5f && holding_mode) {
        motor_PWM = HOLDING_MIN_PWM * 0.85f;
      }

      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
      analogWrite(MOTOR_PWM_PIN, (int)motor_PWM);

      // ---- LINEAR ACTUATOR ----
      int currentPosLA = analogRead(LA_FEEDBACK_PIN);
      int LA_error     = LA_TARGET_POSITION - currentPosLA;
      bool LA_at_target = false;

      if      (LA_error >  LA_TOLERANCE) extendLA(LA_MOTOR_SPEED);
      else if (LA_error < -LA_TOLERANCE) retractLA(LA_MOTOR_SPEED);
      else { stopLA(); LA_at_target = true; }

      // ---- DIAGNOSTIC PRINT ----
      float target_deg_print = traj_desired_pos * 180.0f / 3.14159265359f;
      float actual_deg_print = Theta            * 180.0f / 3.14159265359f;

      Serial.print(target_deg_print, 2);
      Serial.print(" | ");
      Serial.print(actual_deg_print, 2);
      Serial.print(" | ");
      Serial.print(error_deg, 2);
      Serial.print(" | ");
      Serial.print((int)motor_PWM);
      Serial.print(" | F1:");
      Serial.print(F1, 4);
      Serial.print(" | F2:");
      Serial.print(F2, 4);
      Serial.print(" | T:");
      Serial.print(estimatedTension, 3);
      Serial.print(" N | ");

      if (traj_active) {
        Serial.println("TRAJ");
      } else if (tension_descent_active) {
        Serial.println("DESCENDING");
      } else if (holding_mode) {
        Serial.println("HOLD");
      } else {
        Serial.println("IDLE");
      }

      // ---- COMPLETED – allow new command or reset ----
      if (traj_complete && LA_at_target) {
        static unsigned long lastPromptTime = 0;
        if (currentTime - lastPromptTime >= 5000) {
          Serial.println("Ready for next command. Enter: angle,position  |  'r' to reset");
          lastPromptTime = currentTime;
        }

        if (Serial.available() > 0) {
          char firstChar = Serial.peek();

          if (firstChar == 'r' || firstChar == 'R') {
            Serial.read();
            while (Serial.available() > 0) Serial.read();

            currentState            = WAITING_FOR_ZERO;
            holding_mode          = false;
            tension_descent_active = false;
            cable_pulling_direction = 0;
            current_PWM_smooth      = 0.0f;
            error_integral          = 0.0f;

            Serial.println("Reset. Enter 'z' to set zero.");
          } else {
            bool ok = parseTargets();
            if (ok) {
              holding_mode           = false;
              tension_descent_active = false;
              error_integral         = 0.0f;
              Serial.println("Target | Actual | Error | PWM | F1 | F2 | Tension | Status");
            }
          }
        }
      }

      t_prev     = t;
      Theta_prev = Theta;

      delay(10);
      break;
    }
  }
}

// ============================================================
//  PARSE "angle , LA_position" FROM SERIAL
// ============================================================
bool parseTargets() {
  String inputStr = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      inputStr += c;
    }
  }
  while (Serial.available() > 0) Serial.read();

  inputStr.trim();
  int commaIndex = inputStr.indexOf(',');
  if (commaIndex == -1) return false;

  String anglePart    = inputStr.substring(0, commaIndex);
  String positionPart = inputStr.substring(commaIndex + 1);
  anglePart.trim();
  positionPart.trim();

  float newAngle = target_angle_deg;
  bool  updateAngle = true;

  if (anglePart.equalsIgnoreCase("N")) {
    updateAngle = false;
    newAngle    = joint_angle();
  } else {
    newAngle = anglePart.toFloat();
    if (newAngle < -360 || newAngle > 360) return false;
  }

  int  newPosition    = target_position_LA;
  bool updatePosition = true;
  int  currentPosLA   = analogRead(LA_FEEDBACK_PIN);
  int  currentRelLA   = currentPosLA - LA_zeroOffset;

  if (positionPart.equalsIgnoreCase("N")) {
    updatePosition = false;
    newPosition    = currentRelLA;
  } else {
    newPosition = positionPart.toInt();
    if (newPosition < 0 || newPosition > 1023) return false;
  }

  if (!updateAngle && !updatePosition) return false;

  target_angle_deg    = newAngle;
  target_position_LA  = newPosition;
  LA_TARGET_POSITION  = LA_zeroOffset + target_position_LA;
  LA_TARGET_POSITION  = constrain(LA_TARGET_POSITION, 0, 1023);

  Serial.print("New target: ");
  Serial.print(target_angle_deg, 1);
  Serial.print(" deg, LA:");
  Serial.print(target_position_LA);

  Theta = joint_angle() * 3.14159265359f / 180.0f;
  float target_angle_rad = target_angle_deg * 3.14159265359f / 180.0f;

  float initial_error = target_angle_rad - Theta;
  if (abs(initial_error) > 0.1f) {
    cable_pulling_direction = (initial_error > 0) ? 1 : -1;
    Serial.print(" | Dir: ");
    Serial.println(cable_pulling_direction > 0 ? "+" : "-");
  } else {
    Serial.println(" | No angular change needed.");
  }

  startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
  return true;
}

// ============================================================
//  ORANGE ENCODER → JOINT ANGLE (degrees)
// ============================================================
float joint_angle() {
  long newCount  = -orangeEncoder.read();
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0f / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0f;
  return currentJointAngle;
}

// ============================================================
//  TRAJECTORY HELPERS
// ============================================================
void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle = start_pos;
  traj_target_angle = target_pos;
  traj_start_time  = millis() / 1000.0f;
  traj_duration    = duration;
  traj_active      = true;
  traj_complete    = false;
  // Do NOT reset current_PWM_smooth – maintain continuity to avoid jerk
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
  traj_desired_vel = (qf - q0) / T  * (30.0f*s2 - 60.0f*s3 + 30.0f*s4);
  traj_desired_acc = (qf - q0) / (T*T) * (60.0f*s - 180.0f*s2 + 120.0f*s3);
}

// ============================================================
//  LINEAR ACTUATOR HELPERS
// ============================================================
void extendLA(int speed) {
  digitalWrite(LA_DIR_PIN1, HIGH);
  digitalWrite(LA_DIR_PIN2, LOW);
  analogWrite(LA_PWM_PIN, speed);
}

void retractLA(int speed) {
  digitalWrite(LA_DIR_PIN1, LOW);
  digitalWrite(LA_DIR_PIN2, HIGH);
  analogWrite(LA_PWM_PIN, speed);
}

void stopLA() {
  digitalWrite(LA_DIR_PIN1, LOW);
  digitalWrite(LA_DIR_PIN2, LOW);
  analogWrite(LA_PWM_PIN, 0);
}
