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
#define ENCODER_A 25//21
#define ENCODER_B 26//22

// Encoder specifications
const int PPR = 600;
const int COUNTS_PER_REV = PPR * 4;

// Encoder object
Encoder orangeEncoder(ENCODER_A, ENCODER_B);

// Motor PWM pins
const int MOTOR_PWM_PIN    = 8;
const int MOTOR_ENABLE_PIN = 9;
const int MOTOR_DIR_PIN    = 10;

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
bool  traj_active;
bool  traj_complete;
float traj_desired_pos;
float traj_desired_vel;
float traj_desired_acc;

// Direction — set from live error, not cached
// Adjust these two constants if motor spins the wrong way
const int POSITIVE_DIR_PIN_STATE = LOW;   // <-- TUNE IF NEEDED
const int NEGATIVE_DIR_PIN_STATE = HIGH;  // <-- TUNE IF NEEDED

bool holding_mode = false;

// Smooth transition variables
float target_PWM = 0;
float current_PWM_smooth = 0;
const float PWM_RAMP_RATE = 300.0;  // Reduced to prevent jerk

// PID gains with feedforward
float Kp  = 5.8;
float Kd  = 2.0;
float Kff = 1.2;
float Ki  = 0.15;
float error_integral = 0.0;

// PWM limits
const float HOLDING_MIN_PWM = 450;
const float HOLDING_MAX_PWM = 950;
const float REVERSE_MIN_PWM = 450;
const float DEADBAND_DEG    = 0.3;

// Load cell instance
HX711_ADC LOADCELL(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

// ============================================
// LINEAR ACTUATOR CONFIGURATION
// ============================================
const int LA_PWM_PIN      = 5;
const int LA_DIR_PIN1     = 3;
const int LA_DIR_PIN2     = 4;
const int LA_FEEDBACK_PIN = A0;

int LA_TARGET_POSITION = 0;
const int LA_TOLERANCE   = 10;
const int LA_MOTOR_SPEED = 3200;
int LA_zeroOffset        = 0;

// ============================================
// COMBINED STATE MACHINE
// ============================================
enum State {
  WAITING_FOR_ZERO,
  WAITING_FOR_TARGETS,
  RUNNING_MOTION
};

State currentState     = WAITING_FOR_ZERO;
float target_angle_deg = 0.0;
int   target_position_LA = 0;

// Global done flag — NOT a static local so it can be properly reset
bool doneSent = false;

// Telemetry timer
unsigned long lastStatusTime = 0;

// ============================================
// GUI PROTOCOL HELPERS
// ============================================
void sendMsg(const String& msg) {
  Serial.print("MSG:");
  Serial.println(msg);
}

void sendData(float target, float actual, float err,
              int pwm, float torq, int la_pos, const String& status) {
  Serial.print("DATA:");
  Serial.print(target, 2);  Serial.print(",");
  Serial.print(actual, 2);  Serial.print(",");
  Serial.print(err, 2);     Serial.print(",");
  Serial.print(pwm);        Serial.print(",");
  Serial.print(torq, 3);    Serial.print(",");
  Serial.print(la_pos);     Serial.print(",");
  Serial.println(status);
}

// ============================================
// DIRECTION HELPER — single source of truth
// ============================================
void setMotorDirection(float error) {
  if (error >= 0) {
    digitalWrite(MOTOR_DIR_PIN, POSITIVE_DIR_PIN_STATE);
  } else {
    digitalWrite(MOTOR_DIR_PIN, NEGATIVE_DIR_PIN_STATE);
  }
}

// ============================================
void setup() {
  Serial.begin(115200);

  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN,    OUTPUT);
  pinMode(MOTOR_PWM_PIN,    OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWriteResolution(12);

  traj_active   = false;
  traj_complete = false;
  traj_duration = 1.8;

  t_prev     = millis() / 1000.0;
  Theta      = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel  = 0;

  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;

  pinMode(LA_PWM_PIN,      OUTPUT);
  pinMode(LA_DIR_PIN1,     OUTPUT);
  pinMode(LA_DIR_PIN2,     OUTPUT);
  pinMode(LA_FEEDBACK_PIN, INPUT);

  stopLA();

  sendMsg("SYSTEM_READY");
  sendMsg("Send CMD:ZERO to set zero");
}

// ============================================
void loop() {
  LOADCELL.update();
  unsigned long currentTime = millis();

  // ── INCOMING SERIAL COMMANDS ──────────────────────────────
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("CMD:")) {
      String cmd = line.substring(4);
      cmd.trim();

      // ── ZERO ──────────────────────────────────────────────
      if (cmd.equalsIgnoreCase("ZERO")) {
        if (currentState == WAITING_FOR_ZERO ||
            currentState == WAITING_FOR_TARGETS) {
          orangeEncoder.write(0);
          LA_zeroOffset = analogRead(LA_FEEDBACK_PIN);
          currentState  = WAITING_FOR_TARGETS;
          Serial.println("ZERO_OK");
          sendMsg("Zero set. Send CMD:TARGET,angle,position");
        } else {
          sendMsg("Cannot zero while motion is running. Send CMD:RESET first.");
        }
      }

      // ── TARGET ────────────────────────────────────────────
      else if (cmd.startsWith("TARGET,")) {
        if (currentState == WAITING_FOR_TARGETS ||
            (currentState == RUNNING_MOTION && traj_complete)) {
          String params = cmd.substring(7);
          bool ok = parseTargetsFromString(params);
          if (ok) {
            currentState   = RUNNING_MOTION;
            holding_mode   = false;
            error_integral = 0.0;
            doneSent       = false;  // allow DONE to fire for this new move
            sendMsg("Target accepted. Motion starting.");
          } else {
            sendMsg("ERR:Invalid format. Use CMD:TARGET,angle,position");
          }
        } else if (currentState == WAITING_FOR_ZERO) {
          sendMsg("ERR:Set zero first. Send CMD:ZERO");
        } else {
          sendMsg("ERR:Motion already running.");
        }
      }

      // ── RESET ─────────────────────────────────────────────
      else if (cmd.equalsIgnoreCase("RESET")) {
        currentState       = WAITING_FOR_ZERO;
        holding_mode       = false;
        current_PWM_smooth = 0;
        error_integral     = 0.0;
        traj_active        = false;
        traj_complete      = false;
        doneSent           = false;
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        analogWrite(MOTOR_PWM_PIN, 0);
        stopLA();
        Serial.println("RESET_OK");
        sendMsg("Reset done. Send CMD:ZERO to set zero.");
      }

      // ── STATUS (one-shot snapshot) ─────────────────────────
      else if (cmd.equalsIgnoreCase("STATUS")) {
        float actual_deg = joint_angle();
        int   la_pos     = analogRead(LA_FEEDBACK_PIN) - LA_zeroOffset;
        String st = "IDLE";
        if      (currentState == WAITING_FOR_ZERO)    st = "WAIT_ZERO";
        else if (currentState == WAITING_FOR_TARGETS) st = "WAIT_TARGET";
        else if (traj_active)                         st = "TRAJ";
        else if (holding_mode)                        st = "HOLD";
        sendData(target_angle_deg, actual_deg,
                 target_angle_deg - actual_deg,
                 (int)motor_PWM, torque, la_pos, st);
      }

      else {
        sendMsg("ERR:Unknown command: " + cmd);
      }
    }
    // Silently ignore lines that don't start with CMD: 
    // (prevents garbage from crashing the parser)
  }

  // ── MOTION CONTROL ────────────────────────────────────────
  if (currentState != RUNNING_MOTION) return;

  // ── TIME & ANGLE UPDATE ───────────────────────────────────
  t = millis() / 1000.0;
  float dt = t - t_prev;
  if (dt < 0.001) dt = 0.001;

  Theta = joint_angle() * 3.14159265359 / 180.0;
  float vel_raw = (Theta - Theta_prev) / dt;
  Theta_vel = 0.7 * Theta_vel + 0.3 * vel_raw;

  // ── TRAJECTORY ────────────────────────────────────────────
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

  // ── ERRORS ────────────────────────────────────────────────
  motor_error          = traj_desired_pos - Theta;
  float velocity_error = traj_desired_vel - Theta_vel;
  float error_deg      = motor_error * 180.0 / 3.14159265359;
  float abs_error_deg  = abs(error_deg);
  float vel_deg_s      = Theta_vel * 180.0 / 3.14159265359;

  // ── DIRECTION — derived from live error, never cached ─────
  setMotorDirection(motor_error);

  // ── INTEGRAL with anti-windup ─────────────────────────────
  if (abs_error_deg > DEADBAND_DEG) {
    error_integral += motor_error * dt;
    error_integral  = constrain(error_integral, -1.0, 1.0);
  } else {
    error_integral *= 0.90;  // faster decay near target
  }

  // ── PID + FEEDFORWARD ─────────────────────────────────────
  float p_term  = Kp  * motor_error;
  float i_term  = Ki  * error_integral;
  float d_term  = Kd  * velocity_error;
  float ff_term = Kff * traj_desired_acc;
  float control_signal = p_term + i_term + d_term + ff_term;
  float abs_control    = abs(control_signal);

  // ── PWM CALCULATION ───────────────────────────────────────
  if (abs_error_deg < DEADBAND_DEG && abs(vel_deg_s) < 0.5 && holding_mode) {
    // Very close to target — gentle hold, main anti-jerk measure
    target_PWM = HOLDING_MIN_PWM * 0.7;

  } else {
    float base_pwm   = (motor_error >= 0) ? HOLDING_MIN_PWM : REVERSE_MIN_PWM;
    float additional = abs_control * 220.0;

    // Extra damping if moving away from target
    if ((motor_error > 0 && Theta_vel < 0) ||
        (motor_error < 0 && Theta_vel > 0)) {
      additional += abs(vel_deg_s) * 15.0;
    }

    target_PWM = base_pwm + additional;
    target_PWM = constrain(target_PWM, base_pwm * 0.4, HOLDING_MAX_PWM);
  }

  // ── SMOOTH RAMPING ────────────────────────────────────────
  float ramp_rate = PWM_RAMP_RATE;
  if (traj_active) {
    float elapsed = t - traj_start_time;
    if      (elapsed < 0.20) ramp_rate *= 0.4;  // very gentle first 200 ms
    else if (elapsed < 0.50) ramp_rate *= 0.7;  // moderate 200–500 ms
  }

  float pwm_err    = target_PWM - current_PWM_smooth;
  float max_change = ramp_rate * dt;

  if (abs(pwm_err) > max_change) {
    current_PWM_smooth += (pwm_err > 0) ? max_change : -max_change;
  } else {
    current_PWM_smooth = target_PWM;
  }

  motor_PWM = constrain(current_PWM_smooth, 0, HOLDING_MAX_PWM);

  // ── APPLY TO MOTOR ────────────────────────────────────────
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, (int)motor_PWM);

  torque = LOADCELL.getData();

  // ── LINEAR ACTUATOR ───────────────────────────────────────
  int  currentPosLA  = analogRead(LA_FEEDBACK_PIN);
  int  LA_error_val  = LA_TARGET_POSITION - currentPosLA;
  bool LA_at_target  = false;

  if      (LA_error_val >  LA_TOLERANCE) extendLA(LA_MOTOR_SPEED);
  else if (LA_error_val < -LA_TOLERANCE) retractLA(LA_MOTOR_SPEED);
  else { stopLA(); LA_at_target = true; }

  // ── SEND TELEMETRY TO GUI (~20 Hz) ────────────────────────
  if (currentTime - lastStatusTime >= 50) {
    lastStatusTime = currentTime;
    float target_deg = traj_desired_pos * 180.0 / 3.14159265359;
    float actual_deg = Theta            * 180.0 / 3.14159265359;
    int   la_pos_rel = currentPosLA - LA_zeroOffset;

    String status;
    if      (traj_active)   status = "TRAJ";
    else if (holding_mode)  status = "HOLD";
    else                    status = "IDLE";

    sendData(target_deg, actual_deg, error_deg,
             (int)motor_PWM, torque, la_pos_rel, status);
  }

  // ── NOTIFY GUI WHEN MOTION IS COMPLETE (fires once per move) ─
  if (traj_complete && LA_at_target && !doneSent) {
    Serial.println("DONE");
    sendMsg("Motion complete. Send CMD:TARGET,angle,position or CMD:RESET");
    doneSent = true;
  }

  t_prev     = t;
  Theta_prev = Theta;

  delay(10);
}

// ============================================
// PARSE TARGET STRING  "angle,position"
// Both fields accept "N" for no change.
// ============================================
bool parseTargetsFromString(String params) {
  params.trim();
  int commaIndex = params.indexOf(',');
  if (commaIndex == -1) return false;

  String anglePart    = params.substring(0, commaIndex);
  String positionPart = params.substring(commaIndex + 1);
  anglePart.trim();
  positionPart.trim();

  float newAngle    = target_angle_deg;
  int   newPosition = target_position_LA;

  int currentPosLA    = analogRead(LA_FEEDBACK_PIN);
  int currentRelative = currentPosLA - LA_zeroOffset;

  bool updateAngle    = !anglePart.equalsIgnoreCase("N");
  bool updatePosition = !positionPart.equalsIgnoreCase("N");

  if (!updateAngle && !updatePosition) return false;

  if (updateAngle) {
    newAngle = anglePart.toFloat();
    if (newAngle < -360 || newAngle > 360) return false;
  } else {
    newAngle = joint_angle();   // hold current angle
  }

  if (updatePosition) {
    newPosition = positionPart.toInt();
    if (newPosition < 0 || newPosition > 1023) return false;
  } else {
    newPosition = currentRelative;   // hold current LA position
  }

  target_angle_deg   = newAngle;
  target_position_LA = newPosition;
  LA_TARGET_POSITION = constrain(LA_zeroOffset + target_position_LA, 0, 1023);

  Theta = joint_angle() * 3.14159265359 / 180.0;
  float target_rad = target_angle_deg * 3.14159265359 / 180.0;

  String info = "Target=" + String(target_angle_deg, 1) +
                "deg  LA=" + String(target_position_LA);
  sendMsg(info);

  startMinimumJerkTrajectory(Theta, target_rad, traj_duration);
  return true;
}

// ============================================
float joint_angle() {
  long newCount   = -orangeEncoder.read();
  long angleCount =  newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0;
  return currentJointAngle;
}

void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle  = start_pos;
  traj_target_angle = target_pos;
  traj_start_time   = millis() / 1000.0;
  traj_duration     = duration;
  traj_active       = true;
  traj_complete     = false;
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
  traj_desired_vel = (qf - q0) / T       * (30.0*s2 - 60.0*s3 + 30.0*s4);
  traj_desired_acc = (qf - q0) / (T * T) * (60.0*s  - 180.0*s2 + 120.0*s3);
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
