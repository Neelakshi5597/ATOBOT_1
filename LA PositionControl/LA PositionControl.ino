

// Pin definitions (matching your wiring)
const int PWM_PIN = 5;        // PWM/Enable to motor driver EN-b
const int DIR_PIN1 = 3;       // Direction control IN3
const int DIR_PIN2 = 4;       // Direction control IN4
const int FEEDBACK_PIN = A0;  // P16 Purple wire (pot wiper)

// Control parameters
const int TARGET_POSITION = 400;  // Hardcoded target (0-1023, 512 = mid-stroke)
const int TOLERANCE =10;          // Deadband to prevent oscillation
const int MOTOR_SPEED = 200;       // PWM value (0-255), adjust for speed

void setup() {
  Serial.begin(115200);
  
  // Configure motor control pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  
  // Configure feedback input
  pinMode(FEEDBACK_PIN, INPUT);
  
  // Stop motor initially
  stopMotor();
  Serial.print("Target Position: ");
  Serial.println(TARGET_POSITION);
  Serial.println();
  
  delay(1000);  // Give system time to stabilize
}

void loop() {
  // Read current position from feedback potentiometer
  int currentPos = analogRead(FEEDBACK_PIN);
  
  // Calculate error
  int error = TARGET_POSITION - currentPos;
  
  // Print status (every loop for monitoring)
  Serial.print("Current: ");
  Serial.print(currentPos);
  Serial.print(" | Target: ");
  Serial.print(TARGET_POSITION);
  Serial.print(" | Error: ");
  Serial.print(error);
  
  // Control logic with deadband
  if (error > TOLERANCE) {
    // Need to extend (move forward)
    Serial.println(" → EXTENDING");
    extendMotor(MOTOR_SPEED);
  }
  else if (error < -TOLERANCE) {
    // Need to retract (move backward)
    Serial.println(" → RETRACTING");
    retractMotor(MOTOR_SPEED);
  }
  else {
    // Within tolerance - stop motor
    stopMotor();
    Serial.println(" ✓ POSITION REACHED");
  }
  
  delay(20);  // Control loop delay (50Hz)
}

// Motor control functions for H-bridge with separate DIR pins
void extendMotor(int speed) {
  digitalWrite(DIR_PIN1, HIGH);  // IN3 = HIGH
  digitalWrite(DIR_PIN2, LOW);   // IN4 = LOW
  analogWrite(PWM_PIN, speed);   // Enable with PWM speed
}

void retractMotor(int speed) {
  digitalWrite(DIR_PIN1, LOW);   // IN3 = LOW
  digitalWrite(DIR_PIN2, HIGH);  // IN4 = HIGH
  analogWrite(PWM_PIN, speed);   // Enable with PWM speed
}

void stopMotor() {
  digitalWrite(DIR_PIN1, LOW);   // IN3 = LOW
  digitalWrite(DIR_PIN2, LOW);   // IN4 = LOW
  analogWrite(PWM_PIN, 0);       // Disable PWM
}
