// Program Reads in Pressure Transducer

// Pressure Transducer MLH100PGL06A
//    OUTPUT: 0.5V ~ 4.5V
//    INPUT:  5V ~ 6V
//    Operating Pressure: 100 PSI

float PRESSURE_TRANSDUCER_MIN = 0.0;
float PRESSURE_TRANSDUCER_MAX = 100.0;
float PRESSURE_TRANSDUCER_RANGE = PRESSURE_TRANSDUCER_MAX - PRESSURE_TRANSDUCER_MIN;

#define BITS 1024; // Arduino Uno 10-bit input, 0-5V

// Linear Calibration Factors y = M*(x-B)
float M_PRESSURE  = PRESSURE_TRANSDUCER_RANGE / BITS;
float B_PRESSURE1 = 0.0;
float B_PRESSURE2 = 0.0;

int Pin_P1 = A1;   // select the input pin for the potentiometer
int P1_count = 0;  // variable to store the value coming from the sensor
float P1;          // converted pressure value [PSI]

int Pin_P2 = A2;   // select the input pin for the potentiometer
int P2_count = 0;  // variable to store the value coming from the sensor
float P2;          // converted pressure value [PSI]

int dP_count;
float dP;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);

  // Zero the Transducer
  int num_zero = 100;
  int sum_zero_1 = 0;
  int sum_zero_2 = 0;
  for ( int i = 0; i < num_zero; i++)
  {
    sum_zero_1 += analogRead(Pin_P1);
    sum_zero_2 += analogRead(Pin_P2);
  }
  B_PRESSURE1 = float( sum_zero_1 / num_zero );
  B_PRESSURE2 = float( sum_zero_2 / num_zero );

  //
  Serial.println("% Pressure Transducer Readings");
}

void loop() {
  // read the value from the sensor:
  P1_count = analogRead(Pin_P1);    
  P1 = (P1_count-B_PRESSURE1) * M_PRESSURE;

  // read the value from the sensor:
  P2_count = analogRead(Pin_P2);    
  P2 = (P2_count-B_PRESSURE2) * M_PRESSURE;
  
  dP_count = P1_count - P2_count;
  dP = P2 - P1;
  
  ///*
  // Print Stuff
//  Serial.print(millis());
//  Serial.print("\t P1 [Counts] \t");
//  Serial.print(P1_count);   
//  Serial.print("\t P1 [PSI] \t");
//  Serial.print(P1);
  
//  Serial.print("\t P2 [Counts] \t");
//  Serial.print(P2_count);   
//  Serial.print("\t P2 [PSI] \t");
//  Serial.print(P2);
//  
//  Serial.print("\t dP [Counts] \t");
//  Serial.print(dP_count);   
//  Serial.print("\t dP [PSI] \t");
//  Serial.print(dP);

  Serial.print(millis()); Serial.print("\t");
  Serial.print(P1_count); Serial.print("\t"); 
  Serial.println(P1);      
  //Serial.println();    
  delay(2);
  //*/
  
  /*
  Serial.println(P1_count);
  delay(100);
*/


}

