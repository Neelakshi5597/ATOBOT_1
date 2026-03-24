

// -------------------- Motor Driver Pins --------------------
const int PIN_PWM    = 8;
const int PIN_ENABLE = 9;
const int PIN_DIR    = 10;

// -------------------- Timing --------------------
const uint32_t RUN_TIME_MS = 86400000UL;  // 24 hours in milliseconds

// -------------------- PWM Value --------------------
const int CONSTANT_PWM = 500;  // Constant PWM value to apply for 24 hours

// -------------------- Current State --------------------
uint32_t startTime = 0;
uint16_t pwmDuty = CONSTANT_PWM;

// ======================================================
// PWM FUNCTIONS
// ======================================================

void setPWM(uint16_t pwm) {
  // keep previous safe bounds (adjust if needed)
  pwmDuty = constrain(pwm, 410, 3686);
  analogWrite(PIN_PWM, pwmDuty);
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  analogWriteResolution(12);
  analogReadResolution(12);

  // Motor OFF during initialization
  digitalWrite(PIN_ENABLE, LOW);
  digitalWrite(PIN_DIR, LOW);

  // set baseline before enabling
  analogWrite(PIN_PWM, pwmDuty);

  // Print configuration
  Serial.print("Constant PWM: ");
  Serial.print(CONSTANT_PWM);


  // Start motor
  digitalWrite(PIN_ENABLE, HIGH);
  setPWM(CONSTANT_PWM);

  startTime = millis();
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
  uint32_t elapsed = millis() - startTime;

  // Check if 24 hours have elapsed
  if (elapsed >= RUN_TIME_MS) {
    // Stop motor after 24 hours
    analogWrite(PIN_PWM, 410); // safe idle
    digitalWrite(PIN_ENABLE, LOW);
    Serial.println("\n=== Complete. 24 hours elapsed. Motor stopped. ===");
    while (1);
  }

  // -------------------- DATA LOGGING --------------------
  Serial.println(pwmDuty);

  delay(100);  // 10 Hz sampling rate (100ms delay)
}
