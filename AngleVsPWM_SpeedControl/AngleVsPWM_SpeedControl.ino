#include "HX711_ADC.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include <Encoder.h>

// Calibration factor for load cell
#define calibration_factor  398046.19

// HX711 pin definitions
#define LOADCELL1_DOUT_PIN 34
#define LOADCELL1_SCK_PIN 35

// Orange encoder definitions (for joint angle)
#define ENCODER_A 25  // Green wire
#define ENCODER_B 26  // White wire

// Encoder specifications for orange encoder
const int PPR = 600;
const int COUNTS_PER_REV = PPR * 4;  // 2400 counts per revolution

// Encoder object
Encoder orangeEncoder(ENCODER_A, ENCODER_B);  // Orange joint encoder

// Variables for orange encoder
float currentJointAngle = 0.0;

// Load cell instance
HX711_ADC LOADCELL(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

// PWM initialization
int PWMpin = 8;
int enablepin = 9;
int directionpin = 10;

// Control variables
float Theta;        // Current angle (radians)
float Theta_prev;   // Previous angle for velocity estimation
float Theta_vel;    // Estimated velocity (rad/s)
float PWM;
float error;
float t;
float t_prev;
float torque;

// Minimum Jerk Trajectory variables
float traj_start_angle;     // Starting angle for trajectory (radians)
float traj_target_angle;    // Target angle for trajectory (radians)
float traj_start_time;      // Time when trajectory started
float traj_duration;        // Duration of trajectory (seconds)
bool traj_active;           // Is trajectory currently active?
bool traj_complete;         // Has trajectory been completed?
float traj_desired_pos;     // Current desired position from trajectory (radians)
float traj_desired_vel;     // Current desired velocity from trajectory (rad/s)
float traj_desired_acc;     // Current desired acceleration from trajectory (rad/s²)

// PD gains with feedforward
float Kp = 4.0;      // Proportional gain
float Kd = 1.0;      // Derivative gain
float Kff = 1.0;     // Feedforward gain for acceleration

// Control signal limits
const float PWM_MIN = 410;
const float PWM_MAX = 800;
const float DEADBAND = 0.005;  // Small deadband to prevent oscillation when at target

// -------------------------------------------------------
// Automated sequence configuration
// -------------------------------------------------------
const float HOLD_DURATION_SEC = 2.0;   // Hold time at each position (seconds)
const float TRAJ_DURATION_SEC  = 3.0;  // Travel time between positions (seconds)
const float STEP_DEG           = 10.0; // Step size in degrees
const float MAX_DEG            = 100.0;// Maximum angle in degrees

// Build the full waypoint list: 0,10,20,...,100,90,...,0  (21 waypoints)
const int NUM_WAYPOINTS = 21;
float waypoints[NUM_WAYPOINTS]; // filled in setup()

// State machine variables
enum State {
  WAITING_FOR_ZERO,      // Step 1: user sets zero
  RUNNING_SEQUENCE,      // Automated sweep
  SEQUENCE_DONE          // All waypoints visited
};

State currentState = WAITING_FOR_ZERO;
float zeroAngle = 0.0;
unsigned long lastDisplayTime  = 0;

// Sequence tracking
int   seqIndex     = 0;          // Current waypoint index
bool  seqHolding   = false;      // True while holding at a waypoint
unsigned long holdStartTime = 0; // millis() when hold began

// Recorded data per waypoint (angle deg, PWM value)
float recordedAngle[NUM_WAYPOINTS];
float recordedPWM  [NUM_WAYPOINTS];

// -------------------------------------------------------
// Forward declarations
// -------------------------------------------------------
float joint_angle();
void  startMinimumJerkTrajectory(float start_pos, float target_pos, float duration);
void  calculateMinimumJerkTrajectory(float tau);
void  printDataTable();

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Build waypoints: 0 → 100 in steps of 10, then 90 → 0
  for (int i = 0; i <= 10; i++) waypoints[i]      = i * STEP_DEG;       // 0..100
  for (int i = 1; i <= 10; i++) waypoints[10 + i] = (10 - i) * STEP_DEG; // 90..0

  // Torque Sensor
  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);

  // Motor setup
  pinMode(enablepin, OUTPUT);
  pinMode(directionpin, OUTPUT);
  pinMode(PWMpin, OUTPUT);

  // Orange joint encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  digitalWrite(enablepin, LOW);

  analogReadResolution(12);
  analogWriteResolution(12);

  // Initialize trajectory variables
  traj_active   = false;
  traj_complete = false;
  traj_duration = TRAJ_DURATION_SEC;

  // Initialize state variables
  t_prev    = millis() / 1000.0;
  Theta     = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel  = 0;

  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;


  Serial.println("  Automated Angle Sweep: 0→100→0 degrees");
  Serial.print("Current joint angle: ");
  Serial.print(joint_angle(), 2);
  Serial.println(" degrees");
  Serial.println();
  Serial.println("Step 1: Move shaft to desired zero position,");
  Serial.println("        then enter 'z' or 'Z' to set zero.");
}

// -------------------------------------------------------
void loop() {
  LOADCELL.update();

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
        while (Serial.available() > 0) Serial.read(); // flush

        if (input == 'z' || input == 'Z') {
          orangeEncoder.write(0);
          zeroAngle = 0.0;

          Serial.println();
          Serial.println("Zero position set!");
        
          Serial.println("Starting automated sweep sequence...");
          Serial.println();
          Serial.println("Step | Target(deg) | Desired(deg) | Actual(deg) | PWM");
        

          // Start first waypoint trajectory from current position
          Theta = joint_angle() * 3.14159265359 / 180.0;
          float target_rad = waypoints[0] * 3.14159265359 / 180.0;
          startMinimumJerkTrajectory(Theta, target_rad, TRAJ_DURATION_SEC);

          seqIndex   = 0;
          seqHolding = false;
          currentState = RUNNING_SEQUENCE;
        }
      }
      break;

    // ========== STATE 2: RUNNING AUTOMATED SEQUENCE ==========
    case RUNNING_SEQUENCE: {
      t = millis() / 1000.0;
      float dt = t - t_prev;
      if (dt < 0.001) dt = 0.001;

      // Read current angle
      Theta = joint_angle() * 3.14159265359 / 180.0;

      // Velocity estimation with low-pass filter
      float vel_raw = (Theta - Theta_prev) / dt;
      Theta_vel = 0.7 * Theta_vel + 0.3 * vel_raw;

      // ---- Trajectory update ----
      if (traj_active) {
        float elapsed = t - traj_start_time;
        if (elapsed >= traj_duration) {
          traj_active      = false;
          traj_complete    = true;
          traj_desired_pos = traj_target_angle;
          traj_desired_vel = 0;
          traj_desired_acc = 0;
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
        control_signal = 0;
      }

      // Apply PWM
      if (abs(control_signal) < 0.01) {
        digitalWrite(enablepin, LOW);
        PWM = 0;
      } else {
        PWM = constrain(PWM_MIN + 311.607 * abs(control_signal), PWM_MIN, PWM_MAX);
        digitalWrite(enablepin, HIGH);
        analogWrite(PWMpin, (int)PWM);
        if (control_signal > 0) {
          digitalWrite(directionpin, LOW);
        } else {
          digitalWrite(directionpin, HIGH);
        }
      }

      // Read torque
      torque = LOADCELL.getData();

      // ---- Sequence logic ----
      if (!seqHolding) {
        // Travelling to current waypoint — print live progress every 50 ms
        if (currentTime - lastDisplayTime >= 50) {
          Serial.print("  -->  ");
          Serial.print(seqIndex);
          Serial.print("   |  ");
          Serial.print(waypoints[seqIndex], 1);
          Serial.print("          |  ");
          Serial.print(traj_desired_pos * 180.0 / 3.14159265359, 2);
          Serial.print("        |  ");
          Serial.print(Theta * 180.0 / 3.14159265359, 2);
          Serial.print("       |  ");
          Serial.println((int)PWM);
          lastDisplayTime = currentTime;
        }

        // Check if we've arrived (trajectory complete AND settled)
        if (traj_complete && abs(error) < 0.05) { // ~0.05 rad ≈ 3°
          seqHolding    = true;
          holdStartTime = currentTime;

          // Record data at this waypoint
          recordedAngle[seqIndex] = Theta * 180.0 / 3.14159265359;
          recordedPWM[seqIndex]   = PWM;

          Serial.println();
          Serial.print("[HOLD] Waypoint ");
          Serial.print(seqIndex);
          Serial.print("  Target: ");
          Serial.print(waypoints[seqIndex], 1);
          Serial.print(" deg  |  Actual: ");
          Serial.print(recordedAngle[seqIndex], 2);
          Serial.print(" deg  |  PWM: ");
          Serial.println((int)recordedPWM[seqIndex]);
        }

      } else {
        // Holding at current waypoint — keep motor energised to hold position
        // (control loop still runs above, just seqHolding flag is set)

        unsigned long elapsed_hold = currentTime - holdStartTime;

        // Print countdown every second
        if (currentTime - lastDisplayTime >= 500) {
          Serial.print("  Holding... ");
          Serial.print((HOLD_DURATION_SEC * 1000 - elapsed_hold) / 1000.0, 1);
          Serial.println(" s remaining");
          lastDisplayTime = currentTime;
        }

        // After hold duration, advance to next waypoint
        if (elapsed_hold >= (unsigned long)(HOLD_DURATION_SEC * 1000)) {
          seqIndex++;

          if (seqIndex >= NUM_WAYPOINTS) {
            // All waypoints complete
            digitalWrite(enablepin, LOW); // disable motor
            currentState = SEQUENCE_DONE;
            Serial.println();
           
            Serial.println("  Sequence complete! Motor disabled.");
           
            printDataTable();
          } else {
            // Start trajectory to next waypoint
            seqHolding = false;
            Theta = joint_angle() * 3.14159265359 / 180.0;
            float next_rad = waypoints[seqIndex] * 3.14159265359 / 180.0;
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
      delay(10); // 100 Hz
      break;
    }

    // ========== STATE 3: SEQUENCE DONE ==========
    case SEQUENCE_DONE:
      // Motor is off. Optionally re-run by entering 'r'
      if (Serial.available() > 0) {
        char c = Serial.read();
        while (Serial.available() > 0) Serial.read();
        if (c == 'r' || c == 'R') {
          currentState = WAITING_FOR_ZERO;
          Serial.println();
          Serial.println("=== RESTARTING ===");
          Serial.println("Step 1: Enter 'z' or 'Z' to set zero position");
        }
      }
      break;
  }
}

// -------------------------------------------------------
// Print final recorded data table
// -------------------------------------------------------
void printDataTable() {
  Serial.println();
  Serial.println("========== RECORDED DATA TABLE ==========");
  Serial.println("Index | Target (deg) | Actual (deg) | PWM");
  Serial.println("-----------------------------------------");
  for (int i = 0; i < NUM_WAYPOINTS; i++) {
    Serial.print("  ");
    Serial.print(i);
    Serial.print("   |  ");
    Serial.print(waypoints[i], 1);
    Serial.print("          |  ");
    Serial.print(recordedAngle[i], 2);
    Serial.print("         |  ");
    Serial.println((int)recordedPWM[i]);
  }

  Serial.println();
  Serial.println("Enter 'r' or 'R' to restart from zero.");
}

// -------------------------------------------------------
// Returns joint angle in degrees (clockwise positive)
// -------------------------------------------------------
float joint_angle() {
  long newCount  = -orangeEncoder.read();
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  if (currentJointAngle < 0) currentJointAngle += 360.0;
  return currentJointAngle;
}

// -------------------------------------------------------
void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle  = start_pos;
  traj_target_angle = target_pos;
  traj_start_time   = millis() / 1000.0;
  traj_duration     = duration;
  traj_active       = true;
  traj_complete     = false;
}

// -------------------------------------------------------
void calculateMinimumJerkTrajectory(float tau) {
  float T  = traj_duration;
  float q0 = traj_start_angle;
  float qf = traj_target_angle;
  float s  = tau / T;
  if (s < 0) s = 0;
  if (s > 1) s = 1;

  float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;

  float position_polynomial     = 10.0*s3 - 15.0*s4 + 6.0*s5;
  traj_desired_pos = q0 + (qf - q0) * position_polynomial;

  float velocity_polynomial     = 30.0*s2 - 60.0*s3 + 30.0*s4;
  traj_desired_vel = (qf - q0) / T * velocity_polynomial;

  float acceleration_polynomial = 60.0*s - 180.0*s2 + 120.0*s3;
  traj_desired_acc = (qf - q0) / (T * T) * acceleration_polynomial;
}
