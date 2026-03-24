#include "HX711_ADC.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include <Encoder.h>

// Calibration factor for load cell
#define calibration_factor  398046.19

// HX711 pin definitions
#define LOADCELL1_DOUT_PIN 34
#define LOADCELL1_SCK_PIN 35

// Orange encoder definitions (for joint angle)
#define ENCODER_A 22  // Green wire
#define ENCODER_B 21  // White wire

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

// State machine variables
enum State {
  WAITING_FOR_ZERO,
  WAITING_FOR_TARGET,
  RUNNING_TRAJECTORY
};

State currentState = WAITING_FOR_ZERO;
float zeroAngle = 0.0;           // Zero position in degrees
float target_angle_deg = 0.0;    // Target angle in degrees (from user input)
unsigned long lastDisplayTime = 0;

void setup() {
  Serial.begin(115200);
  
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
  traj_active = false;
  traj_complete = false;
  traj_duration = 3.0;  // 10 seconds for smooth trajectory
  
  // Initialize state variables using joint angle
  t_prev = millis() / 1000.0;
  Theta = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel = 0;
  
  // Initialize desired trajectory values
  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;
  
  Serial.println("Motor Control System");
  Serial.print("Current joint angle: ");
  Serial.print(joint_angle(), 2);
  Serial.println(" degrees");
  Serial.println("---------------------------------------");
  Serial.println();
  
  // Start with Step 1
  Serial.println("Step 1: Enter 'z' or 'Z' to set zero position");
  
  delay(100);
}

void loop() {
  LOADCELL.update();
  
  unsigned long currentTime = millis();
  
  switch (currentState) {
    
    // ========== STATE 1: WAITING FOR ZERO ==========
    case WAITING_FOR_ZERO:
      // Display current angle every 500ms
      if (currentTime - lastDisplayTime >= 500) {
        Serial.print("Current joint angle: ");
        Serial.print(joint_angle(), 2);
        Serial.println(" degrees");
        lastDisplayTime = currentTime;
      }
      
      // Check for 'z' or 'Z' input
      if (Serial.available() > 0) {
        char input = Serial.read();
        
        // Clear any extra characters in buffer
        while (Serial.available() > 0) {
          Serial.read();
        }
        
        if (input == 'z' || input == 'Z') {
          // Reset encoder to zero
          orangeEncoder.write(0);
          zeroAngle = 0.0;
          
          Serial.print("[User types: ");
          Serial.print(input);
          Serial.println("]");
          Serial.println("Zero position has been set!");
          Serial.println("---------------------------------------");
          Serial.println();
          delay(500);
          
          // Move to next state
          currentState = WAITING_FOR_TARGET;
          Serial.println("Step 2: Enter target angle in degrees:");
        }
      }
      break;
    
    // ========== STATE 2: WAITING FOR TARGET ==========
    case WAITING_FOR_TARGET:
      // Check for target angle input
      if (Serial.available() > 0) {
        target_angle_deg = Serial.parseFloat();
        
        // Clear any extra characters in buffer
        while (Serial.available() > 0) {
          Serial.read();
        }
        
        // Validate input (reasonable range)
        if (target_angle_deg >= -360 && target_angle_deg <= 360) {
          Serial.print("[User types: ");
          Serial.print(target_angle_deg, 2);
          Serial.println("]");
          Serial.print("Target set to: ");
          Serial.print(target_angle_deg, 2);
          Serial.println(" degrees");
          Serial.println("Starting motion...");
          Serial.println();
          
          // Get current angle and start trajectory
          Theta = joint_angle() * 3.14159265359 / 180.0;
          float target_angle_rad = target_angle_deg * 3.14159265359 / 180.0;
          startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
          
          // Move to next state
          currentState = RUNNING_TRAJECTORY;
          
          Serial.println("Desired | Actual");
        } else {
          Serial.println("Invalid angle! Please enter a value between -360 and 360 degrees");
          Serial.println();
          Serial.println("Step 2: Enter target angle in degrees:");
        }
      }
      break;
    
    // ========== STATE 3: RUNNING TRAJECTORY ==========
    case RUNNING_TRAJECTORY:
      // Current time in seconds
      t = millis() / 1000.0;
      float dt = t - t_prev;
      
      // Ensure reasonable dt
      if (dt < 0.001) dt = 0.001;
      
      // Read current angle from joint encoder (orange encoder)
      Theta = joint_angle() * 3.14159265359 / 180.0;
      
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
        
        // *** REVERSED DIRECTION ***
        if (control_signal > 0) {
          digitalWrite(directionpin, LOW);   // Changed from HIGH to LOW
        } else {
          digitalWrite(directionpin, HIGH);  // Changed from LOW to HIGH
        }
      }
      
      // Read torque from load cell
      torque = LOADCELL.getData();
      
      // Print desired and actual angles side by side
      Serial.print(traj_desired_pos * 180.0 / 3.14159265359, 2);
      Serial.print(" | ");
      Serial.println(Theta * 180.0 / 3.14159265359, 2);
      
      // Check if trajectory is complete and prompt for new input
      if (traj_complete) {
        static unsigned long lastPromptTime = 0;
        if (currentTime - lastPromptTime >= 3000) {
          Serial.println();
          Serial.println("Trajectory complete!");
          Serial.println("Options:");
          Serial.println("  - Enter new target angle in degrees");
          Serial.println("  - Enter 'r' or 'R' to reset zero position");
          Serial.println();
          lastPromptTime = currentTime;
        }
        
        // Check for new input
        if (Serial.available() > 0) {
          char firstChar = Serial.peek();
          
          if (firstChar == 'r' || firstChar == 'R') {
            Serial.read();  // Consume the 'r'
            
            // Clear buffer
            while (Serial.available() > 0) {
              Serial.read();
            }
            
            // Reset to beginning
            currentState = WAITING_FOR_ZERO;
            Serial.println();
            Serial.println("=== RECALIBRATING ===");
            Serial.println();
            Serial.println("Step 1: Enter 'z' or 'Z' to set zero position");
          }
          else if ((firstChar >= '0' && firstChar <= '9') || firstChar == '-' || firstChar == '.') {
            // New target angle
            float newTargetAngle = Serial.parseFloat();
            
            // Clear buffer
            while (Serial.available() > 0) {
              Serial.read();
            }
            
            if (newTargetAngle >= -360 && newTargetAngle <= 360) {
              target_angle_deg = newTargetAngle;
              
              Serial.println();
              Serial.print("New target set to: ");
              Serial.print(target_angle_deg, 2);
              Serial.println(" degrees");
              Serial.println("Starting motion...");
              Serial.println();
              
              // Start new trajectory
              Theta = joint_angle() * 3.14159265359 / 180.0;
              float target_angle_rad = target_angle_deg * 3.14159265359 / 180.0;
              startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
              
              Serial.println("Desired | Actual");
            }
          }
          else {
            // Invalid input, clear buffer
            while (Serial.available() > 0) {
              Serial.read();
            }
          }
        }
      }
      
      // Update previous values
      t_prev = t;
      Theta_prev = Theta;
      
      delay(10);  // 100 Hz control loop
      break;
  }
}

// Function: joint_angle() - reads the orange encoder
// Returns angle in degrees (clockwise positive)
float joint_angle() {
  // Read encoder count (negate to make clockwise positive)
  long newCount = -orangeEncoder.read();
  
  // Calculate angle (0-360 degrees)
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  
  // Handle negative angles
  if (currentJointAngle < 0) {
    currentJointAngle += 360.0;
  }
  
  return currentJointAngle;
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
  float position_polynomial = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  traj_desired_pos = q0 + (qf - q0) * position_polynomial;
  
  // Velocity: dq/dt = (qf - q0)/T * d/ds[10s³ - 15s⁴ + 6s⁵]
  //                 = (qf - q0)/T * (30s² - 60s³ + 30s⁴)
  float velocity_polynomial = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  traj_desired_vel = (qf - q0) / T * velocity_polynomial;
  
  // Acceleration: d²q/dt² = (qf - q0)/T² * d²/ds²[10s³ - 15s⁴ + 6s⁵]
  //                       = (qf - q0)/T² * (60s - 180s² + 120s³)
  float acceleration_polynomial = 60.0 * s - 180.0 * s2 + 120.0 * s3;
  traj_desired_acc = (qf - q0) / (T * T) * acceleration_polynomial;
}
