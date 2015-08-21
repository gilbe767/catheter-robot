// Program Reads in Omega LVDT
// http://www.omega.com/pptst/LD630.html


float LVDT_TRANSDUCER_MIN = 0.0;
float LVDT_TRANSDUCER_MAX = 100.0;
float TRANSDUCER_RANGE = LVDT_TRANSDUCER_MAX - LVDT_TRANSDUCER_MIN;

#define BITS 1024; // Arduino Uno 10-bit input, 0-5V
#define RESISTANCE 244; // Resistor used to convert to voltage

// define upper and lower bound with the given source using ohms law
float V_LOWER = 0.004*RESISTANCE;
float V_UPPER = 0.020*RESISTANCE;

//#define EFFECTIVE_BITS (V_UPPER - V_LOWER)/5.0; // bits used that can be used

float LVDT_BITS_LOWER = 179;    // Fully extened 0.4mA state
float LVDT_BITS_UPPER = 976; // Compressed 0.21mA state

// Linear Calibration Factors y = M*(x-B)
float M_LVDT  = TRANSDUCER_RANGE / (LVDT_BITS_UPPER-LVDT_BITS_LOWER);
int   B_LVDT = 0;

int Pin_LVDT = A0;   // select the input pin for the potentiometer
int LVDT_count = 0;  // variable to store the value coming from the sensor
float LVDT_mm;          // converted pressure value [PSI]


void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);

  // Zero the LVDT
  delay(100);
  int num_zero = 10;
  long sum_zero = 0;
  for ( int i = 0; i < num_zero; i++)
  {
    sum_zero += analogRead(Pin_LVDT);
    delay(10);
  }
  B_LVDT = (int) sum_zero/(float)num_zero;
  
//  Serial.print(num_zero); Serial.print("\t"); 
//  Serial.print(sum_zero); Serial.print("\t"); 
//  Serial.print(B_LVDT); Serial.print("\t");


 
}

void loop() {
  // read the value from the sensor:
  LVDT_count = analogRead(Pin_LVDT);    
  LVDT_mm = (LVDT_count-B_LVDT) * M_LVDT;


  

  // Print Stuff
  Serial.print(millis()); Serial.print("\t");
  Serial.print(LVDT_count); Serial.print("\t"); 
  Serial.println(LVDT_mm,3);      
  Serial.flush();
  //Serial.println();    
  delay(10);



}

