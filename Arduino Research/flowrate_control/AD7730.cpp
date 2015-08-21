/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


#include "AD7730.h"

/************************************************************************************************************************/
//default constructor requires pin names for spi and ready pin
/************************************************************************************************************************/
AD7730::AD7730(long mosi, long miso, long sclk, long ready, long cs){
    pinMode(ready, INPUT);           // set pin to input
    pinMode(cs, OUTPUT);           // set pin to input
	
    _readyPin = ready;
	_csPin = cs;
      
    //setup default parameters for spi
    //_spi.format(8, 1);  //8bits with polarity 0 (clock idle low), phase 1 (write on falling edge, read on rising edge)
    //_spi.frequency(2000000); //2Mhz
	SPI.begin();
	SPI.setDataMode(SPI_MODE1);
	SPI.setClockDivider(SPI_CLOCK_DIV8); //2Mhz
    
    //set default setup
    //_minWeight = 1;
    //_maxWeight = 5;
    //_fullScaleWeight = 11.023; //5kgs
    
}

/************************************************************************************************************************/
// default destructor
/************************************************************************************************************************/
AD7730::~AD7730(void){}

void AD7730::init(void){
	
    //reset the device
    reset(true);
	
    //get default registry values
    _mode = readRegistry(MODE_REG);
    _filter = readRegistry(FILTER_REG);
    _dac = readRegistry(DAC_REG);
    _offset = readRegistry(OFFSET_REG);
    _gain = readRegistry(GAIN_REG);
    
    //set chip select high
    digitalWrite(_csPin, HIGH); 

    _continous = false;
    _bufferCount = 0;
      
}

void AD7730::dump(void){
    
    //get default registry values
    _mode = readRegistry(MODE_REG);
    _filter = readRegistry(FILTER_REG);
    _dac = readRegistry(DAC_REG);
    _offset = readRegistry(OFFSET_REG);
    _gain = readRegistry(GAIN_REG);
    _status = readRegistry(STATUS_REG);
        
    printf("_mode=0x%04x\n",_mode);
    printf("_filter=0x%06x\n",_filter);
    printf("_dac=0x%02x\n",_dac);
    printf("_offset=0x%06x\n",_offset);
    printf("_gain=0x%06x\n",_gain);
    printf("_status=0x%02x\n",_status);
}


/************************************************************************************************************************/
//function to read the specified registry value
/************************************************************************************************************************/
long AD7730::readRegistry(long registry){

    long readReg = 0;
    long bytes = 0;
    
    switch(registry){
    
        case STATUS_REG: //status registry
        {   
            readReg = 0x10;
            bytes = 1;
            break;
        }
        case DATA_REG: //data registry
        {
            readReg = 0x11;
            bytes = 3;
            break;
        }
        case MODE_REG: //mode registry
        {
            readReg = 0x12;
            bytes = 2;
            break;
        }
        case FILTER_REG: //filter registry
        {
            readReg = 0x13;
            bytes = 3;
            break;
        }
        case DAC_REG: //dac registry
        {
            readReg = 0x14;
            bytes = 1;
            break;
        }
        case OFFSET_REG: //offset registry
        {
            readReg = 0x15;
            bytes = 3;
            break;
        }
        case GAIN_REG: //gain registry
        {
            readReg = 0x16;
            bytes = 3;
            break;
        }
        case TEST_REG: //test registry
        {
            readReg = 0x17;
            bytes = 3;
            break;
        }
        default: //default to status registry
        {
            readReg = 0x10;
            bytes = 1;
            break;
        }
    } // end of switch statement
    
    //send command to read the registry
    digitalWrite(_csPin, LOW); 
    delayMicroseconds(5);
    SPI.transfer(readReg);
    
    long value = 0;
    //loop through bytes of returned data
    for(long i=0; i<bytes; i++){
        value = (value << 8) | SPI.transfer(EMPTY_SPI); //shift previous return value up by 8 bits to make room for new data
    }
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
    
    return value;
}

/************************************************************************************************************************/
//function to write to the specified registry
/************************************************************************************************************************/
long AD7730::writeRegistry(long registry, long value){

    long writeReg = 0;
    long bytes = 0;
    
    switch(registry){
    
        case MODE_REG: //mode registry
        {
            writeReg = 0x02;
            bytes = 2;
            _mode = value;
            //check if continous converstion is being stopped or started
            _continous = ((value & 0x2000) > 0)? true : false;
            break;
        }
        case FILTER_REG: //filter registry
        {
            writeReg = 0x03;
            bytes = 3;
            _filter = value;
            break;
        }
        case DAC_REG: //dac registry
        {
            writeReg = 0x04;
            bytes = 1;
            _dac = value;
            break;
        }
        case OFFSET_REG: //offset registry
        {
            writeReg = 0x05;
            bytes = 3;
            _offset = value;
            break;
        }
        case GAIN_REG: //gain registry
        {
            writeReg = 0x06;
            bytes = 3;
            _gain = value;
            break;
        }
        default: //default to communications register
        {
            writeReg = 0x00;
            bytes = 0;
            break;
        }
    } // end of switch statement
    
    //send command to write to a the registry
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(writeReg);
    
    //send data
    switch(bytes)
    {
        case 3:
            SPI.transfer(((value >> 16) & 255));
        case 2:
            SPI.transfer(((value >> 8) & 255));
        case 1:
            SPI.transfer((value & 255));
            break;
        default:
            break;
    }
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
    
    return value;
}


/************************************************************************************************************************/
//function to initiate an internal full-scale calibration
/************************************************************************************************************************/
long AD7730::internalFullCal(void){
    

    //1011000100110000 (0xB130) Internal Full-Scale Calibration, unipolar, long data, low reference, 0-80mv, channel 0
    writeRegistry(MODE_REG, 0xB130);
    delayMicroseconds(1); //give time for ready line to go high
    
    //wait for ready pin to go low indicating calibration complete with timeout of 2000ms
    long time = 0;
    while(digitalRead(_readyPin) && time < 2000){
        delay(2);
        time += 2;
    }
    
    return 0;
}


/************************************************************************************************************************/
//function to initiate an internal zero-scale calibration
/************************************************************************************************************************/
long AD7730::internalZeroCal(void){
    

    //1001000100110000 (0x9100) Internal Zero-Scale Calibration, unipolar, long data, low reference, 0-10mv, channel 0
    writeRegistry(MODE_REG, 0x9100);
    delayMicroseconds(1); //give time for ready line to go high
    
    //wait for ready pin to go low indicating calibration complete with timeout of 2000ms
    long time = 0;
    while(digitalRead(_readyPin) && time < 2000){
        delay(2);
        time += 2;
    }
    
	return 0;
}



/************************************************************************************************************************/
//function to initialize the chip settings to default values after power up and to run internal calibration
/************************************************************************************************************************/
long AD7730::initialize(void){

    //reset device
    reset(true); //initate a full reset
    
    systemLowCal(_minWeight);
    
    //set the Offset
    writeRegistry(OFFSET_REG, _offset);
    
    //set the Gain
    writeRegistry(GAIN_REG, _gain);
    
    //set the DAC
    writeRegistry(DAC_REG, _dac); 
    
    return 0;   
     
}


/************************************************************************************************************************/
//function to initiate a system zero-scale calibration
/************************************************************************************************************************/
long AD7730::systemLowCal(double wgt){
    
    //set minimum weight for low calibration
    _minWeight = wgt;
    
    
    //1101000100000000 (0xD100) System Zero-Scale Calibration, unipolar, long data, low reference, 0-10mv, channel 0
    long mode = 0xD100;
    
    writeRegistry(MODE_REG, mode);
    
    delayMicroseconds(1);  //give time for ready pin to go high
    long time = 0;
    while(digitalRead(_readyPin) && time < 2000){
        time += 2;
        delay(2);
    }//wait for ready pin to go low     
    
    if(digitalRead(_readyPin)){
        //printf("External Zero Failed\r\n");
    }
    
    return (time >= 2000) ? 1 : 0;
    
}


/************************************************************************************************************************/
//function to initiate a system high calibration
/************************************************************************************************************************/
long AD7730::systemHighCal(double max, double fullScale){
    
    //get the current offset value
    long offset = readRegistry(OFFSET_REG);
    long fullScaleAdjust = ((double)offset - 8388608) + 16777215;
    fullScaleAdjust /= 2;
    //double calFactor = fullScale / (fullScaleAdjust / 2); //dual load cells splits the weight in half
    
    //set maximum weight for high calibration
    _maxWeight = max;
    //calculate the expected value for the maximum weight based on the full scale of the load cell
    double expected = ((fullScaleAdjust * max) / fullScale);
    
    //take some samples
    double avg = 0;
    double value = 0;
    for(long i=0; i<20; i++){
        value = (double)read();
        avg += value;  
    }
    
    avg = avg / 20;
    
    //get current gain setting
    double gain = (double)readRegistry(GAIN_REG);
    
    //calculate new gain value
    double adjustedGain = gain * (expected / avg);
    
    printf("Expected: %.3f\r\nActual: %.3f\r\n", expected, avg);
    
    long err = 0;
    if(adjustedGain > 16777215){
        //printf("Exceeded Maximum Gain Value\r\n");
        err = 1;
    }
    
    //update gain registry
    writeRegistry(GAIN_REG, (int)adjustedGain);
    
    return err;
}



/************************************************************************************************************************/
//function to initiate the conversion of a sample
/************************************************************************************************************************/
long AD7730::startConversion(bool wait){
    
    //set the mode to do a single conversion
    //5432109876543210
    //0100000100000000 (0x4100)  Single Conversion, bipolar, short data, low reference, 0-10mv, channel 0
    long mode = 0x4100;

    writeRegistry(MODE_REG, mode);
    
    if(wait){
        //wait for conversion to complete
         
        delayMicroseconds(1); //give time for ready to go high*/
    
        long time = 0;
        while(digitalRead(_readyPin) && time < 2000000){
            time += 2;
            delayMicroseconds(2);
        }//wait for ready pin to go low.*/
        
        if(time >= 2000000){
            printf("Convert Timeout\r\n");
            return 1;
        }
    }
    
    return 0;
}

/************************************************************************************************************************/
//function to do a single read with conversion
/************************************************************************************************************************/
long AD7730::read(){

    if(_continous){
        //chip is running in continous conversion mode
        return _lastValue;
    }
    else {
        //a new conversion must be started
        if(startConversion(true)==1){
            return 0;
        }
        return readRegistry(DATA_REG);
    }              
}


/************************************************************************************************************************/
//function to set the filter settings
/************************************************************************************************************************/
void AD7730::setFilter(long SF, bool chop, long filter2){
    
    SF = SF << 12; //slide SF settings up 12 bits
    
    switch(filter2){
        case 2:
            SF = SF | 512;  //turn on bit 10 for skip mode
            break;
        case 1:
            SF = SF | 256; //turn on bit 09 for fast mode
            break;
        default:
            break; //leave bits 9 & 10 off so secondary filter is fully enabled
    }
    
    if(chop){
        SF = SF | 16; //turn on bit 5 to enable chop mode
    }
    
    writeRegistry(FILTER_REG,SF);
    
}

/************************************************************************************************************************/
//function to reset the chip
/************************************************************************************************************************/
void AD7730::reset(bool fullReset){
    
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    delayMicroseconds(5);
    digitalWrite(_csPin, HIGH);
    
    //if not a full reset, then reload registry settings
    if(!fullReset){
        writeRegistry(MODE_REG, _mode);
        writeRegistry(FILTER_REG, _filter);
        writeRegistry(DAC_REG, _dac);
        writeRegistry(OFFSET_REG, _offset);
        writeRegistry(GAIN_REG, _gain);
    }
    else {
        //reinitiate internal calibration
        internalFullCal();
        delay(100);
        internalZeroCal();
        delay(100);
    }
}

/************************************************************************************************************************/
//function to adjust the DAC
/************************************************************************************************************************/
long AD7730::adjustDAC(long direction){

    long DAC = readRegistry(DAC_REG);
    long sign = DAC & 32; //get the sign bit
    DAC &= 31; //remove sign bit 
 
    
    if(direction > 0){
        //increment DAC
        if((sign > 0) && (DAC == 1)){ //sign bit is set and DAC is already at 1 change to 0;
            DAC = 0;
            sign = 0;
        }
        else if((sign == 0) && (DAC >= 0)){  //sign bit is not set increment DAC
            DAC++;
        }
        else if ((sign > 0) && (DAC > 0)){  //sign bit is set decrement DAC because it is negative
            DAC--;
        }
        
    }
    
   else if(direction < 0){
        //decrement DAC
        if((sign == 0) && (DAC == 0)){ //sign bit is not set and DAC is already at 0
            DAC = 1;
            sign = 1;
        }
        else if((sign == 0) && (DAC > 0)){  //sign bit is not set increment DAC
            DAC--;
        }
        else if ((sign > 0) && (DAC >= 0)){  //sign bit is set decrement DAC because it is negative
            DAC++;
        }
        
    }    

    else{
        //no change in direction
        //do nothing
        return DAC;
    }
    
    if(DAC > 31){
        DAC = 31;  //limit DAC to maximum value of 31 (5 bits)
    }
    
    if(sign > 0){
        DAC |= 32; //set the sign bit of DAC
    }
    
    //update DAC
    writeRegistry(DAC_REG, DAC);
    
    return DAC;
}

/************************************************************************************************************************/
//function to set the filtering SF setting
/************************************************************************************************************************/
void AD7730::setFilterSF(long sf){

    //get current filter setting
    long filter = readRegistry(FILTER_REG);
    
    //clear sf bits 
    filter &= 0xFFF;
    
    //bitshift sf up by 12 bits
    sf = sf << 12;
    
    //or sf bits with filter bits
    filter = filter | sf;
    
    //apply new filter setting
    writeRegistry(FILTER_REG, filter);
}

/************************************************************************************************************************/
//function to set the filtering chop mode
/************************************************************************************************************************/
void AD7730::setFilterChop(long enabled){

    //get current filter setting
    long filter = readRegistry(FILTER_REG);
    
    //clear bit 5 
    filter &= ~0x10;
    
    //apply chop setting
    if(enabled)
        filter |= 0x10;
    
    //apply new filter setting
    writeRegistry(FILTER_REG, filter);
}

/************************************************************************************************************************/
//function to set the mode of the second filter
/************************************************************************************************************************/
void AD7730::setFilterMode(long mode){

    //get current filter setting
    long filter = readRegistry(FILTER_REG);
    
    //clear bits 9 & 10 
    filter &= ~0x300;
    
    //apply mode setting
    if(mode == 1){
        filter |= 0x100; //apply fast mode
    }
    else if(mode == 2){
        filter |= 0x200; //apply skip mode
    }
    else {
        //leave both bits unset
    }
    
    
    //apply new filter setting
    writeRegistry(FILTER_REG, filter);
}

/************************************************************************************************************************/
//function to set the chip to continous conversion
/************************************************************************************************************************/
void AD7730::start(void){
    
    writeRegistry(MODE_REG, 0x2100); //set to continous conversion mode
    _frequency = 0;
    _continous = true; //set flag indicating that chip is running in continous mode
}

/************************************************************************************************************************/
//function to stop the chip running in continous conversion mode
/************************************************************************************************************************/
void AD7730::stop(void){
    
    writeRegistry(MODE_REG, 0x1100); //set to idle mode
    
	_continous = false;
  
}

/************************************************************************************************************************/
//function to check if the ready digital line is low
/************************************************************************************************************************/
bool AD7730::isReady(void){
     
    //if digital line is high, return not ready  
    return (digitalRead(_readyPin))? false : true;
    
}

/************************************************************************************************************************/
//function to read data on falling edge of ready pin when running in continous conversion mode
/************************************************************************************************************************/
void AD7730::interruptRead(void){

    
    _lastValue = readRegistry(DATA_REG);
    
    //store data in circular buffer
    _readBuffer[_bufferCount] = _lastValue;
    if(_bufferCount < 99){
        _bufferCount++;
    }
    else {
        _bufferCount = 0;
    }
    
}

/************************************************************************************************************************/
//function to return the hertz of the data conversion 
/************************************************************************************************************************/
double AD7730::getHz(void){

    return (1000000 / (double)_frequency);
}