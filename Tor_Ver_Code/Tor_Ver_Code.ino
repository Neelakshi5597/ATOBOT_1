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

#define ENC1A          3 //16//6//28//2//2//27//3//6//2//5//26
#define ENC1B          2 //17//27//3//3//28//2//7//3//4//25

#define ENC1MAXCOUNT    4*2048*43//4*43*4096//4*2500//4 * 90500//4*43*2048//
#define ENC1COUNT2DEG   0.25f*0.004087//0.25f*0.002044f//0.25f * 0.0039779f//0.25f*0.0040879f//

// Encoder objects.
Encoder angle(ENC1A, ENC1B);
long _enccount;

// Load cell instances
HX711_ADC loadcell1(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
HX711_ADC loadcell2(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

// PWM initialization
int PWMpin = 8;
int enablepin = 9;
int directionpin = 10;
//int speedpin = 22;

float TheDe;
float TheDes;
float Theta;
float PWM;
float error;
float errori;
float errord;
float t;
float ti;
float CL;
float torque;

void setup() {
 // put your setup code here, to run once:

 Serial.begin(115200);

 // Load cell setup
  loadcell1.begin(); loadcell2.begin();

  loadcell1.setCalFactor(calibration_factor1);
  loadcell2.setCalFactor(calibration_factor2);

  loadcell1.tare(); loadcell2.tare();
  
  // motor setup
  pinMode(enablepin, OUTPUT);
  pinMode(directionpin, OUTPUT);
  pinMode(PWMpin, OUTPUT);

  // Encoder reading pins for Motor 1
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);

  digitalWrite(enablepin, LOW);
  
  analogReadResolution(12);
  analogWriteResolution(12);

}

void loop() {

// Update load cells and get forces
          loadcell1.update(); float F1 = loadcell1.getData();
          loadcell2.update(); float F2 = -loadcell2.getData();
          
           Serial.print(F1);
           Serial.print(" , ");
           Serial.println(F2);

//This is the angle that I am giving as an input
           TheDe = 90.0;  
           TheDes = TheDe*3.14/180.0;
          
//           Serial.print("DesiredAngle");
//            Serial.print("\t");
//            Serial.println(TheDes);
//            Serial.print('\n');

//This is the angle read by the encoder          
           Theta = angle_motor()*3.14/180.0; 
          
//            Serial.print("EncoderAngle");
//            Serial.print("\t");
//            Serial.println(Theta);
//            Serial.print('\n');

// gets the time and converts it into seconds          
           t=millis()/1000.0;

// finding the error between the desired angle and the actual angle 
           error = TheDes - Theta;
          
//            Serial.print("Error");
//            Serial.print("\t");
//            Serial.println(error);
//            Serial.print('\n');
            
// finds the derivative of the error        
           errord = (error - errori)/(t-ti);
          
//            Serial.print("Derivative of Error");
//            Serial.print("\t");
//            Serial.println(errord);
//            Serial.print('\n');
            
//finding the desired speed    
           CL = 4*error + 1*errord;
          
//            Serial.print("ControlLaw");
//            Serial.print("\t");
//            Serial.println(CL);
//            Serial.print('\n'); 
           
           PWM=constrain(409 + 311.607*abs(CL), 410, 800);
          
//            Serial.print("PWM");
//            Serial.print("\t");
//            Serial.println(PWM);
//            Serial.print('\n'); 
          
           digitalWrite(enablepin,HIGH);
           analogWrite(PWMpin,PWM);
          
      
          
//           Serial.print("Torque");
//            Serial.print("\t");
//            Serial.println(torque);
//            Serial.print('\n');
//          
              
           if (CL>0)
              {
                
                digitalWrite(directionpin,HIGH);
              }
              else
              {
                
                digitalWrite(directionpin,LOW);
              }
               delay(10);
               ti=t;
               errori = error;
               
          }

 float angle_motor()
          {  
            _enccount = angle.read();
            if (_enccount >= ENC1MAXCOUNT) {
              angle.write(_enccount - ENC1MAXCOUNT);
            } else if (_enccount <= - ENC1MAXCOUNT) {
              angle.write(_enccount + ENC1MAXCOUNT);
            }
            return ENC1COUNT2DEG * _enccount;
          
          }
