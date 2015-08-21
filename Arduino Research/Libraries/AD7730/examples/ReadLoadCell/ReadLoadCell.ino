

#include <SPI.h>
//#include <AD7730.h>
#include "C:\Users\MRD\Documents\GitHub\smart-light\arduinoCode\Libraries\AD7730\AD7730.h"

//AD7730( mosi, miso, sclk, ready, cs)
AD7730 adc(11,12,13,7,10);


long start;
int count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(112500);
  
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
  count=0;
}

void loop() {
  // You can poll as often as you want to check for new data,
  // and if its there read it into memory.  Then later (or now)
  // you can call read() and get the most recent value.
  if(adc.isReady()){
    adc.interruptRead();
    count++;
  }
  
  //Since we are running fast, only display every few
  if((millis()-start)>=500){
    Serial.print("Load Cell = ");
    Serial.print(adc.read());
    Serial.print("   Rate = ");
    Serial.println(1000.0*count/(millis()-start));
    start=millis();
    count=0;
  }
}
