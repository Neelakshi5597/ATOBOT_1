#include "HX711_ADC.h" //This library can be obtained here http://librarymanager/All#Avia_HX711

// Calibration factors for load cells
#define calibration_factor1 125339.52
#define calibration_factor2 125339.52

// HX711 pin definitions
#define LOADCELL1_DOUT_PIN 32
#define LOADCELL1_SCK_PIN 33
#define LOADCELL2_DOUT_PIN 34
#define LOADCELL2_SCK_PIN 35

#include <Encoder.h>

#define ENC1A          3
#define ENC1B          2
#define ENC1MAXCOUNT   4*2048*43
#define ENC1COUNT2DEG  0.25f*0.004087

// Encoder objects
Encoder angle(ENC1A, ENC1B);
long _enccount;

// Load cell instances
HX711_ADC loadcell1(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
HX711_ADC loadcell2(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

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
float error_prev;
float t;
float t_prev;

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

// Improved PD gains - TUNE THESE FOR YOUR SYSTEM
float Kp = 8.0;      // Proportional gain (increased for better tracking)
float Kd = 2.5;      // Derivative gain (increased for damping)
float Kff = 1.0;     // Feedforward gain for acceleration

// Target angle setting - now received from serial input
float target_angle_deg = 0.0;  // Will be set by user input
bool target_set = false;  // Flag to indicate target has been set
const float position_tolerance = 0.01;  // Tolerance in radians (~0.57 degrees)

// Control signal limits
const float PWM_MIN = 410;
const float PWM_MAX = 700;
const float DEADBAND = 0.005;  // Small deadband to prevent oscillation when at target

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection
  while (!Serial) {
    delay(10);
  }

  // Load cell setup
  loadcell1.begin(); 
  loadcell2.begin();
  loadcell1.setCalFactor(calibration_factor1);
  loadcell2.setCalFactor(calibration_factor2);
  loadcell1.tare(); 
  loadcell2.tare();
  
  // Motor setup
  pinMode(enablepin, OUTPUT);
  pinMode(directionpin, OUTPUT);
  pinMode(PWMpin, OUTPUT);

  // Encoder reading pins
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);

  digitalWrite(enablepin, LOW);
  
  analogReadResolution(12);
  analogWriteResolution(12);

  // Initialize trajectory variables
  traj_active = false;
  traj_complete = false;
  traj_duration = 10.0;  // 10 seconds for smooth trajectory
  
  // Initialize state variables
  t_prev = millis()/1000.0;
  Theta = angle_motor() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel = 0;
  error_prev = 0;
  
  // Initialize desired trajectory values
  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;
  
  Serial.println("Motor Control System");
  Serial.print("Current angle: ");
  Serial.print(Theta * 180.0 / 3.14159265359, 2);
  Serial.println(" degrees");
  Serial.println("Enter target angle in degrees:");
  
  // Wait for user input
  while (!target_set) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.length() > 0) {
        float new_angle = input.toFloat();
        
        // Validate input
        if (new_angle >= -360.0 && new_angle <= 360.0) {
          target_angle_deg = new_angle;
          target_set = true;
          
          Serial.print("Target set to: ");
          Serial.print(target_angle_deg, 2);
          Serial.println(" degrees");
          Serial.println("Starting motion...");
          Serial.println("Desired | Actual");
          
          // Start trajectory
          float target_angle_rad = target_angle_deg * 3.14159265359 / 180.0;
          startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
        } else {
          Serial.println("Invalid angle! Enter value between -360 and 360:");
        }
      }
    }
    delay(10);
  }
  
  delay(100);
}

void loop() {
  // Update load cells (but don't print)
  loadcell1.update(); 
  loadcell2.update();

  // Current time in seconds
  t = millis()/1000.0;
  float dt = t - t_prev;
  
  // Ensure reasonable dt
  if (dt < 0.001) dt = 0.001;
  
  // Read current angle from encoder
  Theta = angle_motor() * 3.14159265359 / 180.0;
  
  // Estimate velocity using finite difference (with smoothing)
  float vel_raw = (Theta - Theta_prev) / dt;
  Theta_vel = 0.7 * Theta_vel + 0.3 * vel_raw;  // Low-pass filter
  
  // Calculate minimum jerk trajectory
  if (traj_active) {
    float elapsed_time = t - traj_start_time;
    
    if (elapsed_time >= traj_duration) {
      // Trajectory complete - hold final position
      traj_active = false;
      traj_complete = true;
      traj_desired_pos = traj_target_angle;
      traj_desired_vel = 0;
      traj_desired_acc = 0;
    } else {
      // Calculate trajectory using minimum jerk polynomial
      calculateMinimumJerkTrajectory(elapsed_time);
    }
  }
  
  // Position error (desired - actual)
  error = traj_desired_pos - Theta;
  
  // Velocity error (desired - actual)
  float velocity_error = traj_desired_vel - Theta_vel;
  
  // PD control law with feedforward acceleration
  float control_signal = Kp * error + Kd * velocity_error + Kff * traj_desired_acc;
  
  // Apply deadband when trajectory is complete and error is small
  if (traj_complete && abs(error) < DEADBAND) {
    control_signal = 0;
  }
  
  // Convert control signal to PWM with saturation
  if (abs(control_signal) < 0.01) {
    // Very small control signal - turn off motor
    digitalWrite(enablepin, LOW);
    PWM = 0;
  } else {
    // Map control signal to PWM range
    PWM = constrain(PWM_MIN + 311.607 * abs(control_signal), PWM_MIN, PWM_MAX);
    
    // Set motor direction based on control signal sign
    digitalWrite(enablepin, HIGH);
    analogWrite(PWMpin, PWM);
    
    if (control_signal > 0) {
      digitalWrite(directionpin, HIGH);
    } else {
      digitalWrite(directionpin, LOW);
    }
  }
  
  // Print only desired and actual angles
  Serial.print(traj_desired_pos * 180.0 / 3.14159265359, 2);
  Serial.print(" | ");
  Serial.println(Theta * 180.0 / 3.14159265359, 2);
  
  // Update previous values
  t_prev = t;
  Theta_prev = Theta;
  error_prev = error;
  
  delay(10);  // 100 Hz control loop
}

void startMinimumJerkTrajectory(float start_pos, float target_pos, float duration) {
  traj_start_angle = start_pos;
  traj_target_angle = target_pos;
  traj_start_time = millis() / 1000.0;
  traj_duration = duration;
  traj_active = true;
  traj_complete = false;
}

void calculateMinimumJerkTrajectory(float tau) {
  // tau is elapsed time since trajectory start
  float T = traj_duration;
  float q0 = traj_start_angle;
  float qf = traj_target_angle;
  
  // Normalized time (0 to 1)
  float s = tau / T;
  
  // Clamp s to [0, 1] for safety
  if (s < 0) s = 0;
  if (s > 1) s = 1;
  
  // Pre-calculate powers of s
  float s2 = s * s;
  float s3 = s2 * s;
  float s4 = s3 * s;
  float s5 = s4 * s;
  
  // Minimum jerk trajectory polynomial (5th order)
  // Position: q(s) = q0 + (qf - q0) * (10s³ - 15s⁴ + 6s⁵)
  float position_polynomial = 10.0*s3 - 15.0*s4 + 6.0*s5;
  traj_desired_pos = q0 + (qf - q0) * position_polynomial;
  
  // Velocity: dq/dt = (qf - q0)/T * d/ds[10s³ - 15s⁴ + 6s⁵]
  //                 = (qf - q0)/T * (30s² - 60s³ + 30s⁴)
  float velocity_polynomial = 30.0*s2 - 60.0*s3 + 30.0*s4;
  traj_desired_vel = (qf - q0) / T * velocity_polynomial;
  
  // Acceleration: d²q/dt² = (qf - q0)/T² * d²/ds²[10s³ - 15s⁴ + 6s⁵]
  //                       = (qf - q0)/T² * (60s - 180s² + 120s³)
  float acceleration_polynomial = 60.0*s - 180.0*s2 + 120.0*s3;
  traj_desired_acc = (qf - q0) / (T * T) * acceleration_polynomial;
}

float angle_motor() {  
  _enccount = angle.read();
  if (_enccount >= ENC1MAXCOUNT) {
    angle.write(_enccount - ENC1MAXCOUNT);
  } else if (_enccount <= -ENC1MAXCOUNT) {
    angle.write(_enccount + ENC1MAXCOUNT);
  }
  return ENC1COUNT2DEG * _enccount;
}
