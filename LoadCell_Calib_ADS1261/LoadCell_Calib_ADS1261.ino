
#include <SPI.h>

#define CS0_PIN     10
#define DRDY0_PIN   9
#define RESET0_PIN  8

// ================= LOAD CELLS CALIBRATION =================
#define CAL_FACTOR_1  1f  // mg per ADC unit for LoadCell1 (CALIBRATE THIS)
#define CAL_FACTOR_2  1f  // mg per ADC unit for LoadCell2 (CALIBRATE THIS)

float weight1 = 0.0f;
float weight2 = 0.0f;
// -----------------------
// Register writes (transaction already active during acquisition)
// -----------------------
static inline void writeRegister0_fast(uint8_t reg, uint8_t val) {
  digitalWrite(CS0_PIN, LOW);
  SPI.transfer((uint8_t)(0x40 | reg));
  SPI.transfer(val);
  digitalWrite(CS0_PIN, HIGH);
}


static inline void selectDiff0(uint8_t ainp, uint8_t ainn) {
  uint8_t mux = (uint8_t)(((ainp & 0x0F) << 4) | (ainn & 0x0F));
  writeRegister0_fast(0x11, mux);
}

// ======================================================================================
static inline void ads0_start() { digitalWrite(CS0_PIN, LOW); SPI.transfer((uint8_t)0x08);  digitalWrite(CS0_PIN, HIGH); }
static inline void ads0_stop()  { digitalWrite(CS0_PIN, LOW); SPI.transfer((uint8_t)0x0A);  digitalWrite(CS0_PIN, HIGH); }

// =============================== Fast conversion reads ===============================
static inline int32_t readConv0_24b_signext(int pin1, int pin2) {
  selectDiff0(pin1, pin2);
  digitalWrite(CS0_PIN, LOW);
  while (digitalRead(DRDY0_PIN) == LOW){}
  while (digitalRead(DRDY0_PIN) != LOW){}
  SPI.transfer((uint8_t)0x12);
  (void)SPI.transfer((uint8_t)0x00);
  uint8_t b2 = SPI.transfer((uint8_t)0x00);
  uint8_t b1 = SPI.transfer((uint8_t)0x00);
  uint8_t b0 = SPI.transfer((uint8_t)0x00);
  digitalWrite(CS0_PIN, HIGH);;

  int32_t raw = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | (int32_t)b0;
  if (raw & 0x800000) raw |= 0xFF000000;
  
  return raw;
}

static inline void startAcqHard(int pin1, int pin2) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE1));
  selectDiff0(pin1, pin2);
  ads0_start();
}

// ---------------------------------------------------------------------------
// Helper data converter: 6x int16 IMU (12B) + 2x float weights (8B) = 20B/sample
// ---------------------------------------------------------------------------
union Int16ToBytes {
  uint16_t integer;
  struct {
    uint8_t byte1;
    uint8_t byte2;
  } bytes;
};

void convertSampleToBytes(const int16_t* imu_data, float w1, float w2, uint8_t* bytes) {
  // IMU 6x int16 → 12 bytes (little-endian)
  for (int i = 0; i < 6; i++) {
    Int16ToBytes c;
    c.integer = (uint16_t)imu_data[i];
    bytes[2*i]     = c.bytes.byte1;
    bytes[2*i + 1] = c.bytes.byte2;
  }
  // Weight1 float → bytes 12-15
  memcpy(bytes + 12, &w1, 4);
  // Weight2 float → bytes 16-19
  memcpy(bytes + 16, &w2, 4);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(CS0_PIN, OUTPUT);
  pinMode(DRDY0_PIN, INPUT);
  pinMode(RESET0_PIN, OUTPUT);

  digitalWrite(CS0_PIN, HIGH);
  digitalWrite(RESET0_PIN, HIGH);
  digitalWrite(RESET0_PIN, LOW); delayMicroseconds(10); digitalWrite(RESET0_PIN, HIGH);
  
  SPI.begin();
  
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE1));
  writeRegister0_fast(0x02, 0x48);  // DOR 40kSPS
  writeRegister0_fast(0x03, 0x01);  // set to 21 for CHOP mode
  writeRegister0_fast(0x05, 0x00);  // set to 40 for status byte
  writeRegister0_fast(0x06, 0x10);  // Internal Reference
  writeRegister0_fast(0x10, 0x05);  // Gain 32 
  ads0_start();

  
}


// ---------------------------------------------------------------------------
// Main Loop - 4 samples: 20B/sample = 80B data + 12B meta = 92B total
// ---------------------------------------------------------------------------
void loop() {
  
        long adc1 = readConv0_24b_signext(5, 6);
        long adc2 = readConv0_24b_signext(7, 8);    
      
         //weight1 = adc1;
         weight2 = adc2;
//         Serial.print("loadcell1:");
         Serial.println(weight1);
//          Serial.print("loadcell2:");
//         Serial.println(weight2);

     
    
}
