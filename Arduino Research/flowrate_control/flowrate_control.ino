/*
  Program is used to look at the characteristics of Catheter Robot actuators
  Mark Gilbertson
  2015-08-18
  
  Hardware Features
  Bidirectional linear actuator connected to Bimba cylinder
  Omega LD630 LVDT for piston positioning
  Pressure Transducers
  Load cell capabilities  
*/




// ------------------ ADC7730 DEFINES ----------------- //
#include <SPI.h>
#include "AD7730.h"

//AD7730( mosi, miso, sclk, ready, cs)
AD7730 adc(11,12,13,7,10);
long start;
// ---------------------------------------------------- //

// ----------------- DEFINES FOR SECTIONS -------------- //
#define PRINT_RAW_FORCE // print the raw force values in the function

//#define FILTER_AVG
//#define FILTER_IIR
//#define FILTER_FIR

// Pick one of the controllers
  #define MOTOR_PWM_CONTROL
  //#define FORCE_CONTROL        // select when running force feedback
  //#define FLOW_CONTROL       // select when running flow feedback (LVDT)

// Select which sensors are connected  
  #define LOAD_CELL_CONNECTED  // allows the load cell to run
  #define LVDT_CONNECTED       // allows the LVDT to run
  #define PRESSURE_CONNECTED   // allows the pressure transducer to run
// ---------------------------------------------------- //


// Define Globals
volatile int triggerState = LOW;







// --------------------- LVDT DEFINES ----------------- //
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
float M_LVDT  = TRANSDUCER_RANGE / (LVDT_BITS_UPPER-LVDT_BITS_LOWER) /1000.0; // [m/counts]
int   B_LVDT  = 0;  // offset for not starting at zero [counts]

int   LVDT_count = 0;  // variable to store the value coming from the sensor
float LVDT_m;          // converted pressure value [PSI]
float LVDT_m_dot;

float flowrate;        // Flowrate
float fluid_volume;    // Volume of the fluid expunged

// BIMBA CYLIDNER
float DIAMETER_BIMBA = 0.0254; // [m]
float CYLINDER_AREA  = PI/4 * DIAMETER_BIMBA;
float STROKE_LENGTH  = 0.0254*2; // [m]

// Flow Controller
float flow_des = 6.4352e-06; //?? select value [m3/s]
float flow_error = 0;
float flow_kp = 3.9626e+07;
float flow_ki = 0.0;
float flow_kd = 0.0;
// ---------------------------------------------------- //





// -------------- SET UP ARDUINO PINS ------------------ //
int triggerPin = 3;    // hardware trigger wired to spectrometer
int motor1Pin  = 9;      // motor connected to digital pin 3
int motor2Pin  = 6;      // motor connected to digital pin 3
int enablePin  = 2;      //Hbridge enable pin
int Pin_LVDT   = A0;   // select the input pin for the potentiometer
int Pin_P1     = A1;   // select the input pin for the potentiometer
int Pin_P2     = A2;   // select the input pin for the potentiometer
// ---------------------------------------------------- //


// NOT IN USE ANYMORE
bool ledOn       = false;
int ledState     = LOW;
bool amRecording = false;  // when true, timer interupt runs



char incomingByte = 0;   // for incoming serial data
unsigned long time0, timeStamp, timeSerial;
unsigned long timeStamp_old = 0, TS; // old time step and TS
//String dataToWrite = "% time(ms) force(raw) displacement(mm) serialWriteTime(ms)";
String dataToWrite = "% time(ms) force(raw)  serialWriteTime(ms)";

// ---------------- Load Cell Information   ---------------- //
// Force Calibration stuff (taken From Sachin's forceAndDisplacmentCapture_Ghett0_MTS excel calibration
long   forceRaw ;  // captured force value (raw)
volatile float forceW; // actual force reading (in grams)
volatile float force;  // actual force reading (in N)
//#define INTSMULTIPLY 100

int scale_tare = 0; // tare

float M_LOADCELL =  -4.1251e-04;
long C_LOADCELL = 8084500;
float g = 9.81;//*INTSMULTIPLY;

// ---------------------------------------------------- //



// ---------------- Put Force control information   ------------------- //
// PID Gains
float kp = 30.0;
float ki = 0.0;
float kd = 0.0;
int val = 100;         // variable to store the read value
int force_calibrated = 0; // takes the calibrated force value and uses it for feedback 
int force_calibrated_old = 0; // old calibrated force
long force_raw = 0;
int force_des = 12; // desired force 
int dir = 1;
int force_error            = 0; // error between force_des - force_calibrated
//int force_error = 0; // error between force_des - force_calibrated
// ---------------------------------------------------------------------- //


// ----------------------- Filter Variables ---------------------------- //
int indice;
int count = 0;
const int bufferSize = 3;
int buffer[bufferSize] = {0};
int buffer_FIR[bufferSize] = {0};
int buffer_IIR[bufferSize] = {0};
float force_filt_avg, force_avg_IIR, force_avg_FIR; // filtered force value
float A = 0.9, B = 0.1; // IIR Coefficients
float C = 0.9, D = 0.1; // FIR Coefficients
float IIR_old = 0, IIR = 0, FIR_old = 0, FIR = 0;
// ---------------------------------------------------------------------- //



// ----------------------- PRESSURE TRANSDUCER ------------------------- //
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

int P1_count = 0;  // variable to store the value coming from the sensor
float P1;          // converted pressure value [PSI]

int P2_count = 0;  // variable to store the value coming from the sensor
float P2;          // converted pressure value [PSI]

int dP_count;
float dP;
// ---------------------------------------------------------------------- //


// Run the setup function (only executed once) 
void setup() {
	Serial.begin(115200);
	

        // SETUP AD7730
        // Reset the chip, and run calibration
        adc.init();
        // How many times per second do you want to collect data?
        // Note that this assumes you are using a 4.9152 MHz crystal
        // See page 26 of the data sheet for AD7730 Rev B
        // Reasonable values are from 150 to 2000   YMMV
        float rate = 150;
        int SF= ( 4.9152e6 / 16.0 ) / rate;
        adc.setFilter(SF , false, 1);
        // We want to collect continuously, and we will watch the data
        // ready pin to know when new data is available.
        adc.start();
        start=millis();


	// ----------------  SET PIN I/O   ---------------- //
	pinMode(triggerPin, OUTPUT);  
	pinMode(motor1Pin, OUTPUT);   // sets the pin as output
	pinMode(motor2Pin, OUTPUT);   // sets the pin as output
	pinMode(enablePin, OUTPUT);   // sets the pin as output
	// ---------------------------------------------------------------------- //

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
        
        // Zero the Transducer
        num_zero = 100;
        int sum_zero_1 = 0;
        int sum_zero_2 = 0;
        for ( int i = 0; i < num_zero; i++)
        {
          sum_zero_1 += analogRead(Pin_P1);
          sum_zero_2 += analogRead(Pin_P2);
        }
        B_PRESSURE1 = float( sum_zero_1 / num_zero );
        B_PRESSURE2 = float( sum_zero_2 / num_zero );

	
	
	Serial.println("% Arduino Catheter Robot Actuation with Force Feedback");
	digitalWrite(triggerPin, LOW);





	//MsTimer2::set(samplePeriod, timerIsr); // every 10 ms, call timerIsr(); via timer2, Since servo library runs on timer1, timer2 isr will pre-empt and over-rule it; now timer1 manually issues servo commands at the end of timer2isr
	triggerState=LOW;
	timeSerial = 0;
	time0 = millis();

	// Zero the scale
	//zero();

	digitalWrite(enablePin, HIGH);  // turn the H-bridge ON

}




void loop () {


	// HANDLE SERIAL INPUT ... 
	// Listen for serial codes to set parameters or start logging
	if (Serial.available() > 0) 
	{

		// read the incoming byte:
		incomingByte = Serial.read();

		// say what you got:
		Serial.print("%  Received: ");
		Serial.print((char)incomingByte);

		switch ( incomingByte ) 
		{

			// start recording
		case 'R':
		case 'r':
			Serial.println("   Start Recording...");
			Serial.println(dataToWrite);
                        triggerState=HIGH;
			break;

			// stop recording
		case 'S':
		case 's':

			Serial.println("   Stopping Recording...");
			amRecording = false;
			triggerState=LOW;
			digitalWrite(triggerPin, triggerState);
			break;

			// zero the scale  
		case 't':
		case 'T':
			tare(); // clears the scale
			triggerState=LOW;
			break;

		case 'z':
		case 'Z':
			zero(); // reset the "c" calibration factor
			triggerState = LOW;
			break;

			// Find Calibration factors 
		case 'c':
		case 'C':
			calibrate(); // clears the scale
			triggerState=LOW;
			break;

		default:
			Serial.println("% Code not recognized, ignoring...");  
		}
		Serial.flush();
	} // end if (Serial)
        //else
	// Reads force and moves actuator
	// only run when triggerState is true
	if (triggerState == HIGH)
	{
		triggerState=HIGH; // reading value
                read_time();                            // Updates the clock
                
                #ifdef MOTOR_PWM_CONTROL
                  pwm_motor();
                #endif
                
                #ifdef LVDT_CONNECTED // Run when the LVDT is connected
                    read_LVDT();                            // reads in the LVDT to get position and rate
                    calc_flowrate();                        // Returns the flowrate
                    
                    #ifdef FLOW_CONTROL
                      flow_control();
                    #endif
                #endif
                
                
                #ifdef PRESSURE_CONNECTED // Run when the pressure transducers are connected
                    read_pressure();
                #endif
                

                #ifdef LOAD_CELL_CONNECTED // Run when the Load Cell is connected
                    read_load_sensor();					// updates force_calibrated
                    #ifdef FORCE_CONTROL
		        
                        move_actuator(force_calibrated);	// controller
                    #endif
                #endif
                
		print_stuff();						// print define must be enabled
	}

        // You can poll as often as you want to check for new data,
        // and if its there read it into memory.  Then later (or now)
        // you can call read() and get the most recent value.
        if(adc.isReady()){
          adc.interruptRead();
        }


}

// Open loop pwm motor
void pwm_motor()
{
  unsigned static long time_last_pwm = 0, last_stage = 0, dt_stage = 0;
  
  dt_stage = timeStamp - time_last_pwm;
  // update the every once in a while
  if ( dt_stage > 3000)
  {
    dt_stage = 0;
    time_last_pwm = timeStamp;
  }
    
  if ( dt_stage  <= 500 )
  {
    val = 255;
    dir = 1;
    actuate_forward();
  }
  else
  {
    val = 0;
    dir = 0;
    actuate_stop();
  }
//  Serial.print(timeStamp); Serial.print("\t");
//  Serial.print(time_last_pwm); Serial.print("\t");
//  Serial.print(dt_stage); Serial.print("\t");
//  Serial.print(val); Serial.print("\n");
//  delay(100);
  
}


// Updates the clock
void read_time()
{
  	// Get sampling rate
	timeStamp = millis() - time0; 
	TS = timeStamp - timeStamp_old;
	timeStamp_old = timeStamp;
}

// Controller with flowrate
void flow_control()
{

	static int flow_error_old        = 0; // old force
	static int flow_error_derivative = 0;
	static int flow_error_integrated = 0;

	flow_error = flow_des - flowrate; // compute the error in the force
	flow_error_integrated += flow_error*TS; 
	flow_error_derivative  = (flow_error - flow_error_old) * 1/TS;
        flow_error_old = flow_error;
	// controlled command
	int flow_cmd = int( flow_kp*flow_error + flow_ki*flow_error_integrated + flow_kd*flow_error_derivative);

	// extract sign and limit cmd to 255
	dir = sign(flow_cmd);

	val = abs(flow_cmd);
	if ( val > 255 )
	{
		val = 255;
	}

        // Stop failsafe
        if( LVDT_m >= 0.95*STROKE_LENGTH )
        {
          // over max stroke stop moving
          val = 0;
        }



	// determine whether to move F or B
	switch (dir)
	{
	case 1: // move forward
		actuate_forward();
		break;
	case -1: // move backward
		actuate_reverse();
		break;
	default: // Stop
		actuate_stop();
	}  
  
  
  

 
}


// Calculate the flow rate of the system
void calc_flowrate()
{
 flowrate     = CYLINDER_AREA *  LVDT_m_dot;
 fluid_volume = CYLINDER_AREA * LVDT_m;
}

// Reads LVDT Omega 630
void read_LVDT()
{
  // read the value from the sensor:
  LVDT_count = analogRead(Pin_LVDT); delay(2);    
  LVDT_m = (LVDT_count-B_LVDT) * M_LVDT; 
  
  // Rate of the motion
  //LVDT_count   = LVDT_count/TS;
  LVDT_m_dot  = LVDT_m/TS;

}


// Reads in the force value
void read_load_sensor(){



	digitalWrite(triggerPin, triggerState); //enable H-bridge



        

	// read in the force
	//forceRaw = analogRead(forcePin); // raw force reading from strain gage
        
        // AD7730 Force Read
        forceRaw = adc.read();
        
        
	forceW = (forceRaw - C_LOADCELL) * M_LOADCELL ;
	forceW = forceW - scale_tare;


        

	// save the force values
	force_raw = forceRaw;
	force_calibrated = int( forceW ); // value used for feedback control
	//force_calibrated = int( forceRaw );

	
	// indices for filtering
	indice = count % bufferSize; 


        // If filter defines were chosen
        #ifdef FILTER_FIR
        	FIR = C*force_calibrated + D*force_calibrated_old;
        	buffer_FIR[indice] = FIR;
        	force_avg_FIR = mean(buffer_FIR);        // FIR Averaging Value
        #endif
        
        #ifdef FITLER_IIR
        	IIR = A*IIR_old + B*force_calibrated;
        	buffer_IIR[indice] = IIR;
        	force_avg_IIR = mean(buffer_IIR);        // IIR Averaging Value
        #endif
        
        #ifdef FILTER_AVG
        	buffer[indice] = force_calibrated;     // replace one location in the matrix
        	force_filt_avg = mean(buffer);           // averaging filter
        #endif
        
        	count++;
        	force_calibrated_old = force_calibrated;
  


}


// takes the average of an array, arr
float mean(int *arr)
{
	float sum = 0;     // holds the sum of all of the numbers
	int N = sizeof(arr); // get the length of the array
	for (int i = 0; i<N; i++)
	{
		sum += arr[i];
	}
	return (sum/N);
}

// zeros the scale
void tare()
{
	int num_zero = 50;
	float tot_tare = 0;
	// take the average of ten readings
	for (int i = 0; i<num_zero ; i++)
	{

		//forceRaw = analogRead(forcePin); // raw force reading from strain gage
                forceRaw = (float)adc.read();
		tot_tare += (forceRaw - C_LOADCELL) * M_LOADCELL ;

	}
	scale_tare = tot_tare / num_zero;
	Serial.print("tot_tare \t"); Serial.println(tot_tare);
	Serial.print("Tare Value \t"); Serial.println(scale_tare);
}

// zeros the scale
void zero()
{
	// re-zero force measure
	int numSamples = 50; // average of 50 samples 
	unsigned long int avg = 0; 
	for (int i =0; i < numSamples; i++)
		//avg += analogRead(forcePin);
                avg += adc.read();
	avg = avg/(float)numSamples;
	C_LOADCELL = (long)avg;//*INTSMULTIPLY; // this should re-zero
}

// Finds the calibration factors m and c
void calibrate()
{
	int num_calib = 128; // number of calibration steps
	long c_gross = 0; // collection of offset values
	long m_gross = 0;

	// Update the "c" term
	Serial.println("Remove all objects from the scale"); delay(1000);
	//Serial.println("Remove all objects from the scale, press any key to continue");   
	//while(Serial.available() == 0){}; // waits for a keyboard input
	Serial.println("Reading in values to determine offest c ... ");
	for (int i = 0; i < num_calib; i++)
	{
              while (!adc.isReady() ){}
              adc.interruptRead();
		//c_gross += analogRead( forcePin );
                c_gross += adc.read();
	}
	C_LOADCELL = (long) (c_gross/(float)num_calib);
	Serial.print("C_LOADCELL = \t"); Serial.println(C_LOADCELL);


	// Update the "m" term
	Serial.println(" Set the 100.4 g weight on the scale, press any key to continue");
	while(Serial.available() == 0){}; // waits for a keyboard input
	Serial.println("Reading in values to determine m ... ");
	for (int i = 0; i < num_calib; i++)
	{
                while (!adc.isReady() ){}
                adc.interruptRead();
		//m_gross += analogRead( forcePin )-c;
                m_gross += adc.read();
	}
	M_LOADCELL = ( m_gross/ (float)num_calib );
	Serial.print("M_LOADCELL = \t"); Serial.println(M_LOADCELL);

}

// controller for the actuator
void move_actuator(int force_calibrated)
{
	
	static int force_error_old        = 0; // old force
	static int force_error_derivative = 0;
	static int force_error_integrated = 0;

	force_error = force_des - force_calibrated; // compute the error in the force
	force_error_integrated += force_error*TS; 
	force_error_derivative  = (force_error - force_error_old) * 1/TS;
	
	// controlled command
	int cmd = int( kp*force_error + ki*force_error_integrated + kd*force_error_derivative);

	// extract sign and limit cmd to 255
	dir = sign(cmd);

	val = abs(cmd);
	if ( val > 255 )
	{
		val = 255;
	}





	// determine whether to move F or B
	switch (dir)
	{
	case 1: // move forward
		actuate_forward();
		break;
	case -1: // move backward
		actuate_reverse();
		break;
	default: // Stop
		actuate_stop();
	}  

}

void actuate_forward()
{
	analogWrite(motor1Pin, val); 
	analogWrite(motor2Pin, 0);
}

void actuate_reverse()
{
	analogWrite(motor1Pin, 0); 
	analogWrite(motor2Pin, val);  
}

void actuate_stop()
{
	analogWrite(motor1Pin, 0); 
	analogWrite(motor2Pin, 0);  
}

// dithers the linear actutator back and forth
void dither_actuator(int dt)
{
	//int dt = 1000;

	// Moves the actuator forward
	analogWrite(motor1Pin, val);  
	delay(dt);
	analogWrite(motor1Pin, 0);

	// Move the actuator backwards
	analogWrite(motor2Pin, val);  
	delay(dt);
	analogWrite(motor2Pin, 0);
}

// returns the sign of the number
int sign(int num)
{
	int num_sign;
	if ( num >= 0 )
	{
		num_sign = 1;
	}
	else
	{
		num_sign = -1;
	}
	return num_sign; 
}

void read_pressure()
{
    // read the value from the sensor:
  P1_count = analogRead(Pin_P1);    
  P1 = (P1_count-B_PRESSURE1) * M_PRESSURE;

  // read the value from the sensor:
  P2_count = analogRead(Pin_P2);    
  P2 = (P2_count-B_PRESSURE2) * M_PRESSURE;
  
  dP_count = P1_count - P2_count;
  dP = P2 - P1;
}


void print_stuff()
{

                  
                      
#ifdef PRINT_RAW_FORCE
	{
		// Print out time stamp and forces
		Serial.print(timeStamp);
                
                
            #ifdef LOAD_CELL_CONNECTED
                Serial.print("\t"); Serial.print(force_calibrated,3); // calibrated force
                Serial.print("\t"); Serial.print(force_raw);        // raw force
                #ifdef FORCE_CONTROL
		  Serial.print("\t"); Serial.print(force_raw);        // raw force
                  Serial.print("\t"); Serial.print(force_error);			// pwm value for pump
                #endif
            #endif
            
            #ifdef LVDT_CONNECTED // LVDT prints
                Serial.print("\t"); Serial.print(LVDT_count);     
		Serial.print("\t"); Serial.print(LVDT_m,4);
                #ifdef FORCE_CONTROL
                  Serial.print("\t"); Serial.print(flowrate,6);     
                  Serial.print("\t"); Serial.print(fluid_volume,4);
                #endif
            #endif
                
                
            #ifdef PRESSURE_CONNECTED // Pressure Transducers print
               Serial.print("\t"); Serial.print(P1,4);
               Serial.print("\t"); Serial.print(P1_count);   
            #endif    
                
                
		Serial.print("\t"); Serial.print(val);			// pwm value for pump
                Serial.print("\t"); Serial.print(dir);			// pwm value for pump
		Serial.println();
		//Serial.flush();
                delay(10);
	}
#endif
}





