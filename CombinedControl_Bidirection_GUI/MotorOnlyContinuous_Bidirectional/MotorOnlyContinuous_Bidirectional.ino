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

const int PPR            = 600;
const int COUNTS_PER_REV = PPR * 4;

Encoder orangeEncoder(ENCODER_A, ENCODER_B);

// Motor PWM pins
const int MOTOR_PWM_PIN    = 8;
const int MOTOR_ENABLE_PIN = 9;
const int MOTOR_DIR_PIN    = 10;

const int POSITIVE_DIR_PIN_STATE = LOW;
const int NEGATIVE_DIR_PIN_STATE = HIGH;

// ============================================
// MOTOR CONTROL VARIABLES
// ============================================
float currentJointAngle  = 0.0;
float Theta              = 0.0;
float Theta_prev         = 0.0;
float Theta_vel          = 0.0;
float motor_PWM          = 0.0;
float motor_error        = 0.0;
float t                  = 0.0;
float t_prev             = 0.0;
float torque             = 0.0;

bool  holding_mode       = false;

float target_PWM         = 0.0;
float current_PWM_smooth = 0.0;

// ── RAMP RATE ─────────────────────────────────────────────────
// Reduced from 300 → 180 so PWM bleeds off faster on direction
// change, preventing the current spike that causes entry jerk.
const float PWM_RAMP_RATE = 180.0;

// ── PID GAINS ─────────────────────────────────────────────────
// Kp raised: stronger proportional pull toward target.
// Kd kept high: damps arrival shake.
// Ki raised significantly: eliminates the 4° steady-state error.
//   The integral is what corrects the remaining offset once the
//   trajectory is complete and the motor is holding position.
// Kff unchanged.
float Kp  = 5.8;   // was 5.0 — more proportional authority
float Kd  = 3.0;
float Kff = 1.2;
float Ki  = 0.35;  // was 0.08 — must be high enough to correct 4° offset
float error_integral = 0.0;

// ── PWM LIMITS ────────────────────────────────────────────────
const float HOLDING_MIN_PWM = 380.0;
const float HOLDING_MAX_PWM = 950.0;
const float REVERSE_MIN_PWM = 350.0;

// Deadband tightened: was 0.5° which was hiding the 4° error.
// At 0.5° the integral stops accumulating before it can correct
// the offset. Tighten to 0.2° so integral keeps working until
// the motor is truly at the target.
const float DEADBAND_DEG    = 0.2;    // was 0.5

// ============================================
// TRAJECTORY VARIABLES
// ============================================
float traj_start_angle  = 0.0;
float traj_target_angle = 0.0;
float traj_start_time   = 0.0;
float traj_duration     = 2.0;   // was 1.8 — slightly longer, smoother decel
bool  traj_active       = false;
bool  traj_complete     = false;
float traj_desired_pos  = 0.0;
float traj_desired_vel  = 0.0;
float traj_desired_acc  = 0.0;

// ============================================
// LOAD CELL
// ============================================
HX711_ADC LOADCELL(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

// ============================================
// STATE MACHINE
// ============================================
enum State {
  WAITING_FOR_ZERO,
  WAITING_FOR_TARGETS,
  RUNNING_MOTION
};

State currentState     = WAITING_FOR_ZERO;
float target_angle_deg = 0.0;

bool  doneSent             = false;
bool  continuousMode       = false;
bool  waitingForNextTarget = false;

unsigned long lastStatusTime = 0;

// ============================================
// SERIAL BUFFER
// ============================================
#define CMD_BUF_SIZE 64
char          cmdBuf[CMD_BUF_SIZE];
int           cmdLen       = 0;
unsigned long lastCharTime = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
bool  parseTargetFromString(String params);
float joint_angle();
void  startMinimumJerkTrajectory(float start_pos, float target_pos, float duration);
void  calculateMinimumJerkTrajectory(float tau);
void  sendMsg(const String& msg);
void  sendData(float target, float actual, float err,
               int pwm, float torq, const String& status);
void  haltMotion();
void  processCommand(String cmd);
bool  readSerial();

// ============================================
// DIRECTION HELPER
// Called every loop with live motor_error.
// DIR pin flips instantly when error changes sign.
// ============================================
void setMotorDirection(float error) {
  if (error >= 0) {
    digitalWrite(MOTOR_DIR_PIN, POSITIVE_DIR_PIN_STATE);
  } else {
    digitalWrite(MOTOR_DIR_PIN, NEGATIVE_DIR_PIN_STATE);
  }
}

// ============================================
// SERIAL READER
// ============================================
bool readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastCharTime = millis();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        cmdLen = 0;
        return true;
      }
    } else if (cmdLen < CMD_BUF_SIZE - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
  if (cmdLen > 0 && (millis() - lastCharTime) > 120) {
    cmdBuf[cmdLen] = '\0';
    cmdLen = 0;
    return true;
  }
  return false;
}

// ============================================
// PROTOCOL HELPERS
// ============================================
void sendMsg(const String& msg) {
  Serial.print("MSG:");
  Serial.println(msg);
}

void sendData(float target, float actual, float err,
              int pwm, float torq, const String& status) {
  Serial.print("DATA:");
  Serial.print(target, 2);  Serial.print(",");
  Serial.print(actual, 2);  Serial.print(",");
  Serial.print(err, 2);     Serial.print(",");
  Serial.print(pwm);        Serial.print(",");
  Serial.print(torq, 3);    Serial.print(",");
  Serial.println(status);
}

// ============================================
// HALT
// ============================================
void haltMotion() {
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  traj_active          = false;
  traj_complete        = false;
  holding_mode         = false;
  current_PWM_smooth   = 0.0;
  error_integral       = 0.0;
  doneSent             = false;
  continuousMode       = false;
  waitingForNextTarget = false;
}

// ============================================
// COMMAND PROCESSOR
// ============================================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("CMD:") || cmd.startsWith("cmd:")) {
    cmd = cmd.substring(4);
    cmd.trim();
  }

  if (cmd.equalsIgnoreCase("ZERO") || cmd.equalsIgnoreCase("Z")) {
    if (currentState == WAITING_FOR_ZERO ||
        currentState == WAITING_FOR_TARGETS) {
      orangeEncoder.write(0);
      currentState = WAITING_FOR_TARGETS;
      Serial.println("ZERO_OK");
      sendMsg("Zero set. Send START for continuous or TARGET,angle for single move.");
    } else {
      sendMsg("Cannot zero while running. Send STOP first.");
    }
  }

  else if (cmd.equalsIgnoreCase("STOP") || cmd.equalsIgnoreCase("S")) {
    if (continuousMode || currentState == RUNNING_MOTION) {
      haltMotion();
      currentState = WAITING_FOR_TARGETS;
      Serial.println("STOP_OK");
      sendMsg("Motion stopped. Send TARGET,angle or ZERO to re-zero.");
    } else {
      sendMsg("Nothing running.");
    }
  }

  else if (cmd.equalsIgnoreCase("START")) {
    if (currentState == WAITING_FOR_TARGETS) {
      continuousMode       = true;
      waitingForNextTarget = true;
      Serial.println("START_OK");
      sendMsg("Continuous mode ON. Send TARGET,angle  |  STOP to quit.");
    } else if (currentState == WAITING_FOR_ZERO) {
      sendMsg("ERR: Set zero first. Send ZERO.");
    } else {
      sendMsg("ERR: Already running. Send STOP first.");
    }
  }

  else if (cmd.startsWith("TARGET,") || cmd.startsWith("target,") ||
           cmd.startsWith("T,")      || cmd.startsWith("t,")) {
    int    commaIdx = cmd.indexOf(',');
    String params   = cmd.substring(commaIdx + 1);

    bool canAccept =
      (currentState == WAITING_FOR_TARGETS) ||
      (currentState == RUNNING_MOTION && traj_complete && doneSent) ||
      (continuousMode && waitingForNextTarget);

    if (canAccept) {
      bool ok = parseTargetFromString(params);
      if (ok) {
        currentState         = RUNNING_MOTION;
        holding_mode         = false;
        doneSent             = false;
        waitingForNextTarget = false;
        sendMsg("Target accepted. Motion starting.");
      } else {
        sendMsg("ERR: Invalid format. Use TARGET,angle  e.g. TARGET,45");
      }
    } else if (currentState == WAITING_FOR_ZERO) {
      sendMsg("ERR: Set zero first.");
    } else {
      sendMsg("ERR: Motion already running. Wait for DONE or send STOP.");
    }
  }

  else if (cmd.equalsIgnoreCase("RESET") || cmd.equalsIgnoreCase("R")) {
    haltMotion();
    currentState = WAITING_FOR_ZERO;
    Serial.println("RESET_OK");
    sendMsg("Reset done. Send ZERO to set zero.");
  }

  else if (cmd.equalsIgnoreCase("STATUS")) {
    float  actual_deg = joint_angle();
    String st;
    if      (currentState == WAITING_FOR_ZERO)    st = "WAIT_ZERO";
    else if (currentState == WAITING_FOR_TARGETS) st = "WAIT_TARGET";
    else if (traj_active)                         st = "TRAJ";
    else if (holding_mode)                        st = "HOLD";
    else                                          st = "IDLE";
    sendData(target_angle_deg, actual_deg,
             target_angle_deg - actual_deg,
             (int)motor_PWM, torque, st);
  }

  else if (cmd.equalsIgnoreCase("HELP") || cmd.equalsIgnoreCase("?")) {
    Serial.println("ZERO / Z        Set zero reference");
    Serial.println("START           Continuous mode");
    Serial.println("TARGET,angle    Move to angle  e.g. TARGET,45");
    Serial.println("STOP / S        Stop motion");
    Serial.println("RESET / R       Full reset");
    Serial.println("STATUS          One data snapshot");
    Serial.println("HELP            This list");
  }

  else {
    sendMsg("ERR: Unknown command '" + cmd + "'. Send HELP.");
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);

  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN,    OUTPUT);
  pinMode(MOTOR_PWM_PIN,    OUTPUT);
  pinMode(ENCODER_A,        INPUT_PULLUP);
  pinMode(ENCODER_B,        INPUT_PULLUP);

  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWriteResolution(12);

  traj_active   = false;
  traj_complete = false;

  t_prev     = millis() / 1000.0;
  Theta      = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel  = 0.0;

  traj_desired_pos = Theta;
  traj_desired_vel = 0.0;
  traj_desired_acc = 0.0;

  Serial.println("MSG:SYSTEM_READY");
  Serial.println("MSG:Send ZERO to set zero reference");
  Serial.println("MSG:Send HELP for command list");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  LOADCELL.update();
  unsigned long currentTime = millis();

  if (readSerial()) {
    processCommand(String(cmdBuf));
  }

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
      traj_desired_vel = 0.0;
      traj_desired_acc = 0.0;
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

  // ── DIRECTION ─────────────────────────────────────────────
  setMotorDirection(motor_error);

  // ── INTEGRAL ──────────────────────────────────────────────
  // Reset integral completely on direction change to prevent
  // wound-up integral from previous move causing entry jerk.
  static float prev_error_sign = 0.0;
  float cur_error_sign = (motor_error >= 0) ? 1.0 : -1.0;
  if (prev_error_sign != 0.0 && cur_error_sign != prev_error_sign) {
    error_integral = 0.0;   // direction just flipped — wipe integral
  }
  prev_error_sign = cur_error_sign;

  if (abs_error_deg > DEADBAND_DEG) {
    error_integral += motor_error * dt;
    error_integral  = constrain(error_integral, -1.0, 1.0);
  } else {
    // Gentle decay inside deadband — lets integral hold the correction
    // without winding up further
    error_integral *= 0.92;
  }

  // ── PID + FEEDFORWARD ─────────────────────────────────────
  float p_term         = Kp  * motor_error;
  float i_term         = Ki  * error_integral;
  float d_term         = Kd  * velocity_error;
  float ff_term        = Kff * traj_desired_acc;
  float control_signal = p_term + i_term + d_term + ff_term;
  float abs_control    = abs(control_signal);

  // ── PWM CALCULATION ───────────────────────────────────────
  if (abs_error_deg < DEADBAND_DEG && abs(vel_deg_s) < 0.4 && holding_mode) {
    // Settled at target — minimal hold, prevents shake
    target_PWM = HOLDING_MIN_PWM * 0.65;  // was 0.7

  } else {
    float base_pwm   = (motor_error >= 0) ? HOLDING_MIN_PWM : REVERSE_MIN_PWM;
    float additional = abs_control * 200.0;  // was 220 — slightly less aggressive

    // Velocity damping when overshooting
    if ((motor_error > 0 && Theta_vel < 0) ||
        (motor_error < 0 && Theta_vel > 0)) {
      additional += abs(vel_deg_s) * 18.0;  // was 15 — more damping
    }

    target_PWM = base_pwm + additional;
    target_PWM = constrain(target_PWM, base_pwm * 0.4, HOLDING_MAX_PWM);
  }

  // ── SMOOTH RAMPING ────────────────────────────────────────
  float ramp_rate = PWM_RAMP_RATE;
  if (traj_active) {
    float elapsed = t - traj_start_time;
    if      (elapsed < 0.20) ramp_rate *= 0.35;  // was 0.4 — gentler start
    else if (elapsed < 0.50) ramp_rate *= 0.65;  // was 0.7
  }
  // Slow ramp-down into hold to prevent landing jerk
  if (holding_mode) ramp_rate *= 0.5;

  float pwm_err    = target_PWM - current_PWM_smooth;
  float max_change = ramp_rate * dt;
  if (abs(pwm_err) > max_change)
    current_PWM_smooth += (pwm_err > 0) ? max_change : -max_change;
  else
    current_PWM_smooth = target_PWM;

  motor_PWM = constrain(current_PWM_smooth, 0, HOLDING_MAX_PWM);

  // ── APPLY TO MOTOR ────────────────────────────────────────
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, (int)motor_PWM);

  torque = LOADCELL.getData();

  // ── TELEMETRY (~20 Hz) ────────────────────────────────────
  if (currentTime - lastStatusTime >= 50) {
    lastStatusTime = currentTime;
    float target_deg = traj_desired_pos * 180.0 / 3.14159265359;
    float actual_deg = Theta            * 180.0 / 3.14159265359;

    String status;
    if      (traj_active)  status = "TRAJ";
    else if (holding_mode) status = "HOLD";
    else                   status = "IDLE";

    sendData(target_deg, actual_deg, error_deg,
             (int)motor_PWM, torque, status);
  }

  // ── MOTION COMPLETE ───────────────────────────────────────
  if (traj_complete && !doneSent) {
    Serial.println("DONE");
    doneSent = true;
    if (continuousMode) {
      waitingForNextTarget = true;
      Serial.println("NEXT_TARGET");
      sendMsg("Reached target. Send TARGET,angle  |  STOP to finish.");
    } else {
      sendMsg("Motion complete. Send TARGET,angle or RESET.");
    }
  }

  t_prev     = t;
  Theta_prev = Theta;
  delay(10);
}

// ============================================
// PARSE TARGET
// ============================================
bool parseTargetFromString(String params) {
  params.trim();

  float newAngle = target_angle_deg;
  if (params.equalsIgnoreCase("N")) {
    newAngle = joint_angle();
  } else {
    newAngle = params.toFloat();
    if (newAngle < -360 || newAngle > 360) return false;
  }

  // Reset integral on every new target — prevents windup
  // from previous move carrying into the new one
  error_integral = 0.0;

  target_angle_deg = newAngle;

  Theta = joint_angle() * 3.14159265359 / 180.0;
  float target_rad = target_angle_deg * 3.14159265359 / 180.0;

  sendMsg("Target=" + String(target_angle_deg, 1) + "deg");

  startMinimumJerkTrajectory(Theta, target_rad, traj_duration);
  return true;
}

// ============================================
// JOINT ANGLE
// ============================================
float joint_angle() {
  long newCount   = -orangeEncoder.read();
  long angleCount =  newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0;
  return currentJointAngle;
}

// ============================================
// TRAJECTORY
// ============================================
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
