#include "HX711_ADC.h" //This library can be obtained here http://librarymanager/All#Avia_HX711

#define calibration_factor  398046.19//-842222.19//47100.54//-418212.88//410000//398046.19//-379671.69//356166.0//-50942.47//398046.19//-51381.58//51581.5  17950.61//-51488.28This value is obtained using the SparkFun_HX711_Calibration sketch
//#define offset_values 0.0//-228.925 

#define LOADCELL1_DOUT_PIN 34
#define LOADCELL1_SCK_PIN 35

#include <Encoder.h>

#define ENC1A           27//16//6//28//2//2//27//3//6//2//5//26
#define ENC1B          28 //17//27//3//3//28//2//7//3//4//25

#define ENC1MAXCOUNT    4*2048*43//4*43*4096//4*2500//4 * 90500//4*43*2048//
#define ENC1COUNT2DEG   0.25f*0.004087//0.25f*0.002044f//0.25f * 0.0039779f//0.25f*0.0040879f//

// Encoder objects.
Encoder angle(ENC1A, ENC1B);
long _enccount;

//HX711 scale1;
HX711_ADC LOADCELL(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

// PWM initialization
int PWMpin = 8;
int enablepin = 9;
int directionpin = 10;
int speedpin = 22;

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

  //Torque Sensor
  LOADCELL.begin();
  LOADCELL.start(1, true);
  LOADCELL.setCalFactor(calibration_factor);

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

  LOADCELL.update();
  // put your main code here, to run repeatedly:
 TheDe = 360.0;  //This is the angle that I am giving as an input
 TheDes = TheDe*3.14/180.0;

 Serial.print("DesiredAngle");
  Serial.print("\t");
  Serial.println(TheDes);
  Serial.print('\n');

 Theta = angle_motor()*3.14/180.0; //This is the angle read by the encoder

  Serial.print("EncoderAngle");
  Serial.print("\t");
  Serial.println(Theta);
  Serial.print('\n');
 
 t=millis()/1000.0;// gets the time and converts it into seconds
 
 error = TheDes - Theta;// finding the error between the desired angle and the actual angle 

  Serial.print("Error");
  Serial.print("\t");
  Serial.println(error);
  Serial.print('\n');
  
 errord = (error - errori)/(t-ti);// finds the derivative of the error

  Serial.print("Derivative of Error");
  Serial.print("\t");
  Serial.println(errord);
  Serial.print('\n');
 
 CL = 4*error + 1*errord;//finding the desired speed

  Serial.print("ControlLaw");
  Serial.print("\t");
  Serial.println(CL);
  Serial.print('\n'); 
 
 PWM=constrain(409 + 311.607*abs(CL), 410, 800);

  Serial.print("PWM");
  Serial.print("\t");
  Serial.println(PWM);
  Serial.print('\n'); 

 digitalWrite(enablepin,HIGH);
 analogWrite(PWMpin,PWM);

 torque = LOADCELL.getData();

 Serial.print("Torque");
  Serial.print("\t");
  Serial.println(torque);
  Serial.print('\n');

    
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
