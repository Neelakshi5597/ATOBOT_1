#include "HX711_ADC.h"
#include <Encoder.h>

// ============================================
// MAXON MOTOR CONFIGURATION
// ============================================

#define calibration_factor  398046.19

// HX711 pin definitions
#define LOADCELL1_DOUT_PIN 34
#define LOADCELL1_SCK_PIN 35

// Orange encoder definitions
#define ENCODER_A 25
#define ENCODER_B 26

// Encoder specifications
const int PPR = 600;
const int COUNTS_PER_REV = PPR * 4;

// Encoder object
Encoder orangeEncoder(ENCODER_A, ENCODER_B);

// Motor PWM pins
const int MOTOR_PWM_PIN = 8;
const int MOTOR_ENABLE_PIN = 9;
const int MOTOR_DIR_PIN = 10;

// Motor control variables
float currentJointAngle = 0.0;
float Theta;
float Theta_prev;
float Theta_vel;
float motor_PWM;
float motor_error;
float t;
float t_prev;
float torque;

// Minimum Jerk Trajectory variables
float traj_start_angle;
float traj_target_angle;
float traj_start_time;
float traj_duration;
bool traj_active;
bool traj_complete;
float traj_desired_pos;
float traj_desired_vel;
float traj_desired_acc;

// Direction tracking
// +1 means MOTOR_DIR_PIN = LOW drives joint positive
// -1 means MOTOR_DIR_PIN = HIGH drives joint positive
// Adjust this to match your hardware wiring
const int POSITIVE_DIR_PIN_STATE = LOW;   // <-- TUNE IF NEEDED
const int NEGATIVE_DIR_PIN_STATE = HIGH;  // <-- TUNE IF NEEDED

bool holding_mode = false;

// Smooth transition variables
float target_PWM = 0;
float current_PWM_smooth = 0;
const float PWM_RAMP_RATE = 300.0;  // Reduced from 600 to prevent jerk

// PID gains with feedforward
float Kp = 6.0;
float Kd = 2.0;
float Kff = 1.2;
float Ki = 0.15;
float error_integral = 0.0;

// PWM limits
const float HOLDING_MIN_PWM  = 450;   // Reduced baseline to reduce jerk in hold
const float HOLDING_MAX_PWM  = 950;
const float REVERSE_MIN_PWM  = 450;   // PWM floor when reversing
const float DEADBAND_DEG     = 0.3;   // Degrees within which we just hold gently

// Load cell instance
HX711_ADC LOADCELL(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

// ============================================
// LINEAR ACTUATOR CONFIGURATION
// ============================================
const int LA_PWM_PIN = 5;
const int LA_DIR_PIN1 = 3;
const int LA_DIR_PIN2 = 4;
const int LA_FEEDBACK_PIN = A0;

// LA control parameters
int LA_TARGET_POSITION = 0;
const int LA_TOLERANCE = 10;
const int LA_MOTOR_SPEED = 3200;
int LA_zeroOffset = 0;

// ============================================
// COMBINED STATE MACHINE
// ============================================
enum State {
  WAITING_FOR_ZERO,
  WAITING_FOR_TARGETS,
  RUNNING_MOTION
};

State currentState = WAITING_FOR_ZERO;
float target_angle_deg = 0.0;
int target_position_LA = 0;

void setup() {
  Serial.begin(115200);

  // ===== MOTOR SETUP =====
  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWriteResolution(12);

  traj_active   = false;
  traj_complete = false;
  traj_duration = 1.8;

  t_prev    = millis() / 1000.0;
  Theta     = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel  = 0;

  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;

  // ===== LINEAR ACTUATOR SETUP =====
  pinMode(LA_PWM_PIN, OUTPUT);
  pinMode(LA_DIR_PIN1, OUTPUT);
  pinMode(LA_DIR_PIN2, OUTPUT);
  pinMode(LA_FEEDBACK_PIN, INPUT);

  stopLA();

  Serial.println("Ready. Enter 'z' to set zero");
}

// ============================================
// SET MOTOR DIRECTION BASED ON ERROR SIGN
// This is the single source of truth for direction.
// If error > 0 → joint needs to move positive → drive forward
// If error < 0 → joint needs to move negative → drive reverse
// ============================================
void setMotorDirection(float error) {
  if (error >= 0) {
    digitalWrite(MOTOR_DIR_PIN, POSITIVE_DIR_PIN_STATE);
  } else {
    digitalWrite(MOTOR_DIR_PIN, NEGATIVE_DIR_PIN_STATE);
  }
}

void loop() {
  LOADCELL.update();
  unsigned long currentTime = millis();

  switch (currentState) {

    case WAITING_FOR_ZERO:
      if (Serial.available() > 0) {
        char input = Serial.read();
        while (Serial.available() > 0) Serial.read();

        if (input == 'z' || input == 'Z') {
          orangeEncoder.write(0);
          LA_zeroOffset = analogRead(LA_FEEDBACK_PIN);
          currentState = WAITING_FOR_TARGETS;
          Serial.println("Zero set. Enter targets: angle,position");
        }
      }
      break;

    case WAITING_FOR_TARGETS:
      if (Serial.available() > 0) {
        bool parseSuccess = parseTargets();
        if (parseSuccess) {
          currentState = RUNNING_MOTION;
          Serial.println("Target | Actual | Error | PWM | Dir | Status");
        } else {
          Serial.println("Invalid. Format: angle,position");
        }
      }
      break;

    case RUNNING_MOTION: {
      // ===== TIME & ANGLE UPDATE =====
      t = millis() / 1000.0;
      float dt = t - t_prev;
      if (dt < 0.001) dt = 0.001;

      Theta = joint_angle() * 3.14159265359 / 180.0;
      float vel_raw = (Theta - Theta_prev) / dt;
      Theta_vel = 0.7 * Theta_vel + 0.3 * vel_raw;  // low-pass filter

      // ===== TRAJECTORY =====
      if (traj_active) {
        float elapsed_time = t - traj_start_time;

        if (elapsed_time >= traj_duration) {
          traj_active      = false;
          traj_complete    = true;
          holding_mode     = true;
          traj_desired_pos = traj_target_angle;
          traj_desired_vel = 0;
          traj_desired_acc = 0;
        } else {
          calculateMinimumJerkTrajectory(elapsed_time);
        }
      }

      // ===== ERRORS =====
      motor_error        = traj_desired_pos - Theta;
      float velocity_error = traj_desired_vel - Theta_vel;
      float error_deg    = motor_error * 180.0 / 3.14159265359;
      float abs_error_deg = abs(error_deg);
      float vel_deg_s    = Theta_vel * 180.0 / 3.14159265359;

      // ===== DIRECTION PIN — set from live error, not a cached direction =====
      // This ensures direction always reflects which way motor must turn NOW
      setMotorDirection(motor_error);

      // ===== INTEGRAL with anti-windup =====
      if (abs_error_deg > DEADBAND_DEG) {
        error_integral += motor_error * dt;
        error_integral = constrain(error_integral, -1.0, 1.0);
      } else {
        error_integral *= 0.90;  // Faster decay near target to reduce jerk
      }

      // ===== PID + FEEDFORWARD =====
      float p_term = Kp  * motor_error;
      float i_term = Ki  * error_integral;
      float d_term = Kd  * velocity_error;
      float ff_term = Kff * traj_desired_acc;

      float control_signal = p_term + i_term + d_term + ff_term;

      // ===== PWM CALCULATION =====
      // Use abs(control_signal) because direction is handled by the DIR pin.
      // Apply a smooth scaling relative to error magnitude.
      float abs_control = abs(control_signal);

      if (abs_error_deg < DEADBAND_DEG && abs(vel_deg_s) < 0.5 && holding_mode) {
        // ---- DEADBAND: very close to target, minimal holding PWM ----
        // This is the main fix for jerk after reaching position
        target_PWM = HOLDING_MIN_PWM * 0.7;

      } else {
        // ---- ACTIVE CONTROL: drive toward target in either direction ----
        float base_pwm    = (motor_error >= 0) ? HOLDING_MIN_PWM : REVERSE_MIN_PWM;
        float additional  = abs_control * 220.0;

        // Velocity damping: add extra PWM if moving away from target
        if ((motor_error > 0 && Theta_vel < 0) || (motor_error < 0 && Theta_vel > 0)) {
          additional += abs(vel_deg_s) * 15.0;
        }

        target_PWM = base_pwm + additional;
        target_PWM = constrain(target_PWM, base_pwm * 0.4, HOLDING_MAX_PWM);
      }

      // ===== SMOOTH RAMPING =====
      // Ramp rate is reduced near trajectory start to prevent initial jerk
      float ramp_rate = PWM_RAMP_RATE;
      if (traj_active) {
        float elapsed = t - traj_start_time;
        if (elapsed < 0.20) {
          ramp_rate *= 0.4;  // Very gentle in first 200ms
        } else if (elapsed < 0.50) {
          ramp_rate *= 0.7;  // Moderate ramp 200-500ms
        }
      }

      // Also slow ramp when switching between forward and reverse
      // to prevent direction-change jerk
      float pwm_error_val = target_PWM - current_PWM_smooth;
      float max_change    = ramp_rate * dt;

      if (abs(pwm_error_val) > max_change) {
        current_PWM_smooth += (pwm_error_val > 0) ? max_change : -max_change;
      } else {
        current_PWM_smooth = target_PWM;
      }

      motor_PWM = constrain(current_PWM_smooth, 0, HOLDING_MAX_PWM);

      // ===== APPLY TO MOTOR =====
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
      analogWrite(MOTOR_PWM_PIN, (int)motor_PWM);

      torque = LOADCELL.getData();

      // ===== LINEAR ACTUATOR =====
      int currentPosLA = analogRead(LA_FEEDBACK_PIN);
      int LA_error_val  = LA_TARGET_POSITION - currentPosLA;

      bool LA_at_target = false;
      if (LA_error_val > LA_TOLERANCE) {
        extendLA(LA_MOTOR_SPEED);
      } else if (LA_error_val < -LA_TOLERANCE) {
        retractLA(LA_MOTOR_SPEED);
      } else {
        stopLA();
        LA_at_target = true;
      }

      // ===== DIAGNOSTICS =====
      float target_deg = traj_desired_pos * 180.0 / 3.14159265359;
      float actual_deg = Theta * 180.0 / 3.14159265359;

      Serial.print(target_deg, 2);
      Serial.print(" | ");
      Serial.print(actual_deg, 2);
      Serial.print(" | ");
      Serial.print(error_deg, 2);
      Serial.print(" | ");
      Serial.print((int)motor_PWM);
      Serial.print(" | ");
      Serial.print(motor_error >= 0 ? "FWD" : "REV");
      Serial.print(" | ");

      if (traj_active) {
        Serial.println("TRAJ");
      } else if (holding_mode) {
        Serial.println("HOLD");
      } else {
        Serial.println("IDLE");
      }

      // ===== HANDLE NEW INPUT =====
      if (traj_complete && LA_at_target) {
        static unsigned long lastPromptTime = 0;
        if (currentTime - lastPromptTime >= 3000) {
          Serial.println("Complete! Enter: angle,position | 'r' to reset");
          lastPromptTime = currentTime;
        }

        if (Serial.available() > 0) {
          char firstChar = Serial.peek();

          if (firstChar == 'r' || firstChar == 'R') {
            Serial.read();
            while (Serial.available() > 0) Serial.read();

            currentState       = WAITING_FOR_ZERO;
            holding_mode       = false;
            current_PWM_smooth = 0;
            error_integral     = 0.0;

            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            analogWrite(MOTOR_PWM_PIN, 0);

            Serial.println("Reset. Enter 'z' to set zero");
          } else {
            bool parseSuccess = parseTargets();
            if (parseSuccess) {
              holding_mode   = false;
              error_integral = 0.0;
              // NOTE: Do NOT reset current_PWM_smooth here.
              // Keeping it avoids a sudden PWM jump at trajectory start.
              Serial.println("Target | Actual | Error | PWM | Dir | Status");
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

  float newAngle      = target_angle_deg;
  bool  updateAngle   = true;

  if (anglePart.equalsIgnoreCase("N")) {
    updateAngle = false;
    newAngle    = joint_angle();
  } else {
    newAngle = anglePart.toFloat();
    if (newAngle < -360 || newAngle > 360) return false;
  }

  int  newPosition     = target_position_LA;
  bool updatePosition  = true;
  int  currentPosLA    = analogRead(LA_FEEDBACK_PIN);
  int  currentRelative = currentPosLA - LA_zeroOffset;

  if (positionPart.equalsIgnoreCase("N")) {
    updatePosition = false;
    newPosition    = currentRelative;
  } else {
    newPosition = positionPart.toInt();
    if (newPosition < 0 || newPosition > 1023) return false;
  }

  if (!updateAngle && !updatePosition) return false;

  target_angle_deg    = newAngle;
  target_position_LA  = newPosition;
  LA_TARGET_POSITION  = constrain(LA_zeroOffset + target_position_LA, 0, 1023);

  Serial.print("Target: ");
  Serial.print(target_angle_deg, 1);
  Serial.print(" deg, LA: ");
  Serial.println(target_position_LA);

  // Start trajectory from current position
  Theta = joint_angle() * 3.14159265359 / 180.0;
  float target_rad = target_angle_deg * 3.14159265359 / 180.0;

  startMinimumJerkTrajectory(Theta, target_rad, traj_duration);

  return true;
}

float joint_angle() {
  long newCount  = -orangeEncoder.read();
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0;
  return currentJointAngle;
}

void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle = start_pos;
  traj_target_angle = target_pos;
  traj_start_time  = millis() / 1000.0;
  traj_duration    = duration;
  traj_active      = true;
  traj_complete    = false;
  // current_PWM_smooth intentionally NOT reset — prevents jerk at start
}

void calculateMinimumJerkTrajectory(float tau) {
  float T  = traj_duration;
  float q0 = traj_start_angle;
  float qf = traj_target_angle;

  float s  = constrain(tau / T, 0.0, 1.0);
  float s2 = s * s;
  float s3 = s2 * s;
  float s4 = s3 * s;
  float s5 = s4 * s;

  traj_desired_pos = q0 + (qf - q0) * (10.0*s3 - 15.0*s4 + 6.0*s5);
  traj_desired_vel = (qf - q0) / T * (30.0*s2 - 60.0*s3 + 30.0*s4);
  traj_desired_acc = (qf - q0) / (T*T) * (60.0*s - 180.0*s2 + 120.0*s3);
}

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
