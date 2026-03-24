#include "HX711_ADC.h"
#include <Encoder.h>

// ============================================
// MAXON MOTOR CONFIGURATION
// ============================================
// Calibration factor for load cell
#define calibration_factor  398046.19

// HX711 pin definitions
#define LOADCELL1_DOUT_PIN 34
#define LOADCELL1_SCK_PIN 35

// Orange encoder definitions (for joint angle)
#define ENCODER_A 22  // Green wire
#define ENCODER_B 21  // White wire

// Encoder specifications
const int PPR = 600;
const int COUNTS_PER_REV = PPR * 4;  // 2400 counts per revolution

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

// PD gains with feedforward
float Kp = 4.0;
float Kd = 1.0;
float Kff = 1.0;

// Control signal limits
const float PWM_MIN = 410;
const float PWM_MAX = 800;
const float DEADBAND = 0.005;

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
const int LA_MOTOR_SPEED = 3200;  // 12-bit PWM (0-4095), was 200 for 8-bit
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
unsigned long lastDisplayTime = 0;

void setup() {
  Serial.begin(115200);
  
  // ===== MOTOR SETUP =====
  // Torque Sensor
  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);
  
  // Motor pins
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  
  // Encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  
  // Motor uses 12-bit resolution
  analogWriteResolution(12);
  
  // Initialize trajectory variables
  traj_active = false;
  traj_complete = false;
  traj_duration = 3.0;
  
  // Initialize motor state
  t_prev = millis() / 1000.0;
  Theta = joint_angle() * 3.14159265359 / 180.0;
  Theta_prev = Theta;
  Theta_vel = 0;
  
  traj_desired_pos = Theta;
  traj_desired_vel = 0;
  traj_desired_acc = 0;
  
  // ===== LINEAR ACTUATOR SETUP =====
  pinMode(LA_PWM_PIN, OUTPUT);
  pinMode(LA_DIR_PIN1, OUTPUT);
  pinMode(LA_DIR_PIN2, OUTPUT);
  pinMode(LA_FEEDBACK_PIN, INPUT);
  
  stopLA();
  
  // ===== STARTUP MESSAGE =====
  Serial.println("=== COMBINED MOTOR & LINEAR ACTUATOR CONTROL ===");
  Serial.println();
  delay(500);
  
  Serial.println("Step 1: Position both devices, then enter 'z' or 'Z' to set zero");
  Serial.println();
}

void loop() {
  LOADCELL.update();
  unsigned long currentTime = millis();
  
  switch (currentState) {
    
    // ========== STATE 1: WAITING FOR ZERO ==========
    case WAITING_FOR_ZERO:
      // Display current positions every 500ms
      if (currentTime - lastDisplayTime >= 500) {
        int currentPosLA = analogRead(LA_FEEDBACK_PIN);
        float currentAngleMotor = joint_angle();
        
        Serial.print("Motor angle: ");
        Serial.print(currentAngleMotor, 2);
        Serial.print(" deg  |  LA position: ");
        Serial.println(currentPosLA);
        
        lastDisplayTime = currentTime;
      }
      
      // Check for 'z' or 'Z' input
      if (Serial.available() > 0) {
        char input = Serial.read();
        
        // Clear buffer
        while (Serial.available() > 0) {
          Serial.read();
        }
        
        if (input == 'z' || input == 'Z') {
          // Reset both to zero
          orangeEncoder.write(0);
          LA_zeroOffset = analogRead(LA_FEEDBACK_PIN);
          
          Serial.println();
          Serial.println("✓ Zero positions have been set!");
          Serial.print("  Motor zero: 0.00 deg");
          Serial.print("  |  LA zero reference: ");
          Serial.println(LA_zeroOffset);
          Serial.println();
          delay(500);
          
          // Move to next state
          currentState = WAITING_FOR_TARGETS;
          Serial.println("Step 2: Enter targets in format: angle,position");
          Serial.println("         Example: 80,400");
          Serial.println("         (Motor angle in degrees, LA position 0-1023)");
          Serial.println();
        }
      }
      break;
    
    // ========== STATE 2: WAITING FOR TARGETS ==========
    case WAITING_FOR_TARGETS:
      // Display current positions every 500ms
      if (currentTime - lastDisplayTime >= 500) {
        int currentPosLA = analogRead(LA_FEEDBACK_PIN);
        int relativePosLA = currentPosLA - LA_zeroOffset;
        float currentAngleMotor = joint_angle();
        
        Serial.print("Motor: ");
        Serial.print(currentAngleMotor, 2);
        Serial.print(" deg  |  LA: ");
        Serial.println(relativePosLA);
        
        lastDisplayTime = currentTime;
      }
      
      // Check for target input
      if (Serial.available() > 0) {
        // Parse comma-separated values
        target_angle_deg = Serial.parseFloat();
        
        // Look for comma
        if (Serial.available() > 0 && Serial.read() == ',') {
          target_position_LA = Serial.parseInt();
          
          // Clear buffer
          while (Serial.available() > 0) {
            Serial.read();
          }
          
          // Validate inputs
          bool valid = true;
          if (target_angle_deg < -360 || target_angle_deg > 360) {
            Serial.println("✗ Invalid motor angle! Must be between -360 and 360 degrees");
            valid = false;
          }
          if (target_position_LA < 0 || target_position_LA > 1023) {
            Serial.println("✗ Invalid LA position! Must be between 0 and 1023");
            valid = false;
          }
          
          if (valid) {
            // Calculate absolute LA position
            LA_TARGET_POSITION = LA_zeroOffset + target_position_LA;
            LA_TARGET_POSITION = constrain(LA_TARGET_POSITION, 0, 1023);
            
            Serial.println();
            Serial.print("✓ Targets set - Motor: ");
            Serial.print(target_angle_deg, 2);
            Serial.print(" deg  |  LA: ");
            Serial.print(target_position_LA);
            Serial.print(" (absolute: ");
            Serial.print(LA_TARGET_POSITION);
            Serial.println(")");
            Serial.println();
            Serial.println("Starting synchronized motion...");
            Serial.println();
            
            // Initialize motor trajectory
            Theta = joint_angle() * 3.14159265359 / 180.0;
            float target_angle_rad = target_angle_deg * 3.14159265359 / 180.0;
            startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
            
            // Move to running state
            currentState = RUNNING_MOTION;
            
            Serial.println("Motor Desired | Motor Actual | LA Current | LA Target");
          } else {
            Serial.println();
            Serial.println("Please enter valid targets in format: angle,position");
            Serial.println();
          }
        } else {
          // Clear buffer on format error
          while (Serial.available() > 0) {
            Serial.read();
          }
          Serial.println();
          Serial.println("✗ Invalid format! Use: angle,position (e.g., 80,400)");
          Serial.println();
        }
      }
      break;
    
    // ========== STATE 3: RUNNING MOTION ==========
    case RUNNING_MOTION:
      // ===== UPDATE MOTOR =====
      t = millis() / 1000.0;
      float dt = t - t_prev;
      if (dt < 0.001) dt = 0.001;
      
      Theta = joint_angle() * 3.14159265359 / 180.0;
      float vel_raw = (Theta - Theta_prev) / dt;
      Theta_vel = 0.7 * Theta_vel + 0.3 * vel_raw;
      
      // Calculate trajectory
      if (traj_active) {
        float elapsed_time = t - traj_start_time;
        
        if (elapsed_time >= traj_duration) {
          traj_active = false;
          traj_complete = true;
          traj_desired_pos = traj_target_angle;
          traj_desired_vel = 0;
          traj_desired_acc = 0;
        } else {
          calculateMinimumJerkTrajectory(elapsed_time);
        }
      }
      
      // Motor control
      motor_error = traj_desired_pos - Theta;
      float velocity_error = traj_desired_vel - Theta_vel;
      float control_signal = Kp * motor_error + Kd * velocity_error + Kff * traj_desired_acc;
      
      if (traj_complete && abs(motor_error) < DEADBAND) {
        control_signal = 0;
      }
      
      if (abs(control_signal) < 0.01) {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        motor_PWM = 0;
      } else {
        motor_PWM = constrain(PWM_MIN + 311.607 * abs(control_signal), PWM_MIN, PWM_MAX);
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
        analogWrite(MOTOR_PWM_PIN, motor_PWM);
        
        // *** REVERSED DIRECTION ***
        if (control_signal > 0) {
          digitalWrite(MOTOR_DIR_PIN, LOW);   // Changed from HIGH to LOW
        } else {
          digitalWrite(MOTOR_DIR_PIN, HIGH);  // Changed from LOW to HIGH
        }
      }
      
      torque = LOADCELL.getData();
      
      // ===== UPDATE LINEAR ACTUATOR =====
      int currentPosLA = analogRead(LA_FEEDBACK_PIN);
      int relativePosLA = currentPosLA - LA_zeroOffset;
      int relativeTargetLA = LA_TARGET_POSITION - LA_zeroOffset;
      int LA_error = LA_TARGET_POSITION - currentPosLA;
      
      bool LA_at_target = false;
      if (LA_error > LA_TOLERANCE) {
        extendLA(LA_MOTOR_SPEED);
      } else if (LA_error < -LA_TOLERANCE) {
        retractLA(LA_MOTOR_SPEED);
      } else {
        stopLA();
        LA_at_target = true;
      }
      
      // ===== PRINT STATUS =====
      Serial.print(traj_desired_pos * 180.0 / 3.14159265359, 2);
      Serial.print(" | ");
      Serial.print(Theta * 180.0 / 3.14159265359, 2);
      Serial.print(" | ");
      Serial.print(relativePosLA);
      Serial.print(" | ");
      Serial.print(relativeTargetLA);
      
      if (traj_complete && LA_at_target) {
        Serial.println("  ✓ BOTH COMPLETE");
        
        // Prompt for new input every 3 seconds
        static unsigned long lastPromptTime = 0;
        if (currentTime - lastPromptTime >= 3000) {
          Serial.println();
          Serial.println("Motion complete! Options:");
          Serial.println("  - Enter new targets: angle,position (e.g., 120,600)");
          Serial.println("  - Enter 'r' or 'R' to reset zero positions");
          Serial.println();
          lastPromptTime = currentTime;
        }
        
        // Check for new input
        if (Serial.available() > 0) {
          char firstChar = Serial.peek();
          
          if (firstChar == 'r' || firstChar == 'R') {
            Serial.read();
            while (Serial.available() > 0) {
              Serial.read();
            }
            
            currentState = WAITING_FOR_ZERO;
            Serial.println();
            Serial.println("=== RECALIBRATING ===");
            Serial.println();
            Serial.println("Step 1: Position both devices, then enter 'z' or 'Z' to set zero");
            Serial.println();
          }
          else if ((firstChar >= '0' && firstChar <= '9') || firstChar == '-') {
            // Parse new targets
            float newAngle = Serial.parseFloat();
            
            if (Serial.available() > 0 && Serial.read() == ',') {
              int newPos = Serial.parseInt();
              
              while (Serial.available() > 0) {
                Serial.read();
              }
              
              if (newAngle >= -360 && newAngle <= 360 && newPos >= 0 && newPos <= 1023) {
                target_angle_deg = newAngle;
                target_position_LA = newPos;
                LA_TARGET_POSITION = LA_zeroOffset + target_position_LA;
                LA_TARGET_POSITION = constrain(LA_TARGET_POSITION, 0, 1023);
                
                Serial.println();
                Serial.print("✓ New targets - Motor: ");
                Serial.print(target_angle_deg, 2);
                Serial.print(" deg  |  LA: ");
                Serial.println(target_position_LA);
                Serial.println("Starting motion...");
                Serial.println();
                
                Theta = joint_angle() * 3.14159265359 / 180.0;
                float target_angle_rad = target_angle_deg * 3.14159265359 / 180.0;
                startMinimumJerkTrajectory(Theta, target_angle_rad, traj_duration);
                
                Serial.println("Motor Desired | Motor Actual | LA Current | LA Target");
              }
            } else {
              while (Serial.available() > 0) {
                Serial.read();
              }
            }
          }
          else {
            while (Serial.available() > 0) {
              Serial.read();
            }
          }
        }
      } else {
        Serial.println();
      }
      
      t_prev = t;
      Theta_prev = Theta;
      
      delay(10);
      break;
  }
}

// ============================================
// MOTOR FUNCTIONS
// ============================================
float joint_angle() {
  long newCount = -orangeEncoder.read();
  long angleCount = newCount % COUNTS_PER_REV;
  currentJointAngle = (float)angleCount * 360.0 / COUNTS_PER_REV;
  
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
  float T = traj_duration;
  float q0 = traj_start_angle;
  float qf = traj_target_angle;
  
  float s = tau / T;
  if (s < 0) s = 0;
  if (s > 1) s = 1;
  
  float s2 = s * s;
  float s3 = s2 * s;
  float s4 = s3 * s;
  float s5 = s4 * s;
  
  float position_polynomial = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  traj_desired_pos = q0 + (qf - q0) * position_polynomial;
  
  float velocity_polynomial = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  traj_desired_vel = (qf - q0) / T * velocity_polynomial;
  
  float acceleration_polynomial = 60.0 * s - 180.0 * s2 + 120.0 * s3;
  traj_desired_acc = (qf - q0) / (T * T) * acceleration_polynomial;
}

// ============================================
// LINEAR ACTUATOR FUNCTIONS
// ============================================
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
