#ifndef AD7730_h
#define AD7730_h
#define LIBRARY_VERSION	1.0.0


#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SPI.h>

class AD7730{

    public:
        AD7730(long mosi, long miso, long sclk, long ready, long cs);
        ~AD7730(void);
        void init(void);
        void dump(void);
        long readRegistry(long registry);
        long writeRegistry(long registry, long value);
        long initialize(void);
        long systemLowCal(double wgt);
        long systemHighCal(double wgt, double fullScale);
        long read();
        long startConversion(bool wait); 
        void setFilter(long SF, bool chop, long filter2);
        void setFilterSF(long SF);
        void setFilterMode(long mode);
        void setFilterChop(long enabled);
        void reset(bool fullReset);
        long adjustDAC(long direction);
        void start(void);
        void stop(void);
        bool isReady(void);
        double getHz(void);
        void interruptRead(void);     

    
    private:
    
		long _readyPin;
	    long _csPin;
        double _fullScaleWeight;
        double _minWeight;
        double _maxWeight;
        long _readBuffer[100];
        long _bufferCount;
        long _lastValue;
        bool _continous;
        long _frequency;
        
        //Registry value constants
        static const long STATUS_REG = 0;
        static const long DATA_REG = 1;
        static const long MODE_REG = 2;
        static const long FILTER_REG = 3;
        static const long DAC_REG = 4;
        static const long OFFSET_REG = 5;
        static const long GAIN_REG = 6;
        static const long TEST_REG = 7;
        
        //define empty spi command
        static const long EMPTY_SPI = 0xFF;
        
        //internal registries
        long _mode;
        long _filter;
        long _dac;
        long _offset;
        long _gain;
        long _status;
		
        //private functions
        long internalZeroCal(void);
        long internalFullCal(void);   
        
        
};


#endif

