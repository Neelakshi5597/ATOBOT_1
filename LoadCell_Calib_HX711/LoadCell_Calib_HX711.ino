#include <HX711.h>
 
#define LOADCELL_DOUT_PIN 14
#define LOADCELL_SCK_PIN 15
 
HX711 scale;
 
// Calibration factor (counts per kg)
// const float CALIBRATION_FACTOR = -64668.16;
 //const float CALIBRATION_FACTOR = -64445.61;
//const float CALIBRATION_FACTOR = 132560 ;
const float CALIBRATION_FACTOR = 125339.52; 
//const float CALIBRATION_FACTOR = 1 ;
long tare_value = 0;
 
void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
 
  // Perform tare using raw counts
  tare_value = scale.read();
  Serial.println("Tare done! Place weight now.");
}
 
void loop() {
  // Manual tare from Serial
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'T' || command == 't') {
      tare_value = scale.read();  // raw counts at zero
      Serial.println("Tare complete!");
    }
    while (Serial.available() > 0) Serial.read();
  }
 
  // Read raw counts
  long raw = scale.read();
 
  // Negative correlation: subtract raw from tare
  float weight_kg = (float)(tare_value - raw) / CALIBRATION_FACTOR;
 
  //Serial.printf("Raw: %ld\n", raw);
  Serial.printf("Raw: %ld, Weight: %.4f kg\n", raw, weight_kg);
  delay(500);
}
 
