// ACIO Library

#include <ACIO.h>

ACIO::ACIO(HardwareSerial &_ser, byte mode){
    _serial = _ser;
    _isHost = mode;
}

int ACIO::begin(){
    // Begin ACIO bus and autodetect baud.
    begin(true);
}

int ACIO::begin(bool waitForBaud, int bR){
    if(_serial == NULL){
        #ifdef ACIO_ERROR
            DBG_SERIAL.println("Fatal ACIO Error: UART port returned NULL");
        #endif
        return -1;
    }

    if(waitForBaud){
        #ifdef ACIO_VERBOSE
        DBG_SERIAL.println("ACIO: Waiting to detect baudrate...");
        #endif
        _currentBaud = autodetectBaud();    // Autodetect already sets up the UART port for us.
        _ACIOready = true;

        #ifdef ACIO_VERBOSE
        DBG_SERIAL.print("ACIO: Detected bus running at ");
        DBG_SERIAL.print(_currentBaud);
        DBG_SERIAL.println("baud.");
        #endif

    } else {
        // Begin at a given baudrate
        #ifdef ACIO_VERBOSE
        DBG_SERIAL.print("ACIO: Begin, using ");
        DBG_SERIAL.print(bR);
        DBG_SERIAL.println("baud.");
        #endif

        _serial->begin(bR);
        while (_serial->available()){
            _serial->read();
        }
        _serial->flush();
        _currentBaud = bR;
        _ACIOready = true;
    }
}


int ACIO::update(){
    // Return 1 if a packet is in the rxBuffer.
    // Return -1 if ACIO is down, -2 if the bus had to be reset
    if(!_ACIOready){
        return -1;
    }

    if(_serial->available() >= ACIO_MIN_SIZE){
        // Wait for 6 bytes in serial buffer before processing.
        _lastReceivedTime = millis();
        return readRequest(_rxBuffer);
    } else if ((millis() - _lastReceivedTime) > ACIO_TIMEOUT_PERIOD){
        #ifdef ACIO_ERROR
            DBG_SERIAL.println("ACIO Error: Bus went quiet, no messages for a long time. (bad baud?)");
        #endif
        reset();
        if(!_disableRestart) begin(ACIO_AUTODETECT_BAUD);
        return -2;
    } else {
        return 0;
    }
}

void ACIO::reset(){
    _serial->end();
    _ACIOready = false;
}

int ACIO::readRequest(ACIO_Frame &_frame){
    do {
        byte dataIn = readByte();
        switch(_frame->currentByte){
            case 0:
                // "Preamble"
                if(dataIn == 0xAA){
                    // Valid start
                    _frame->preamble = 0xAA;
                    return -1;
                }
            break;
            case 1:
                // Device ID
                _frame->nodeID = dataIn;
            break;
            case 2:
                // Command
                _frame->command = dataIn;
            break;
            case 3:
                // Frame ID
                _frame->frameID = dataIn;
            break;
            case 4:
                // Data Length
                _frame->numBute = dataIn;
            break;
            default:
                // Data bytes from here onwards.
                _frame->data[currentByte] = dataIn;
            break;
        }
        _frame->currentByte++;
    } while(_serial->available());

    if(_frame->fullyReceived){
        // Only do something when true
        return 1;
    }
    
    return 0;
}

byte ACIO::readByte(){
    // All raw reads should come through this function. Returns processed byte, but will return 0xAA and 0xFF too.
    unsigned static int byteCount = 0;  // This is used to check for initialization sequences.
    static bool escNextByte = false;    // If set, byte needs to be inverted (escaped byte)

    byte data = _serial->read();
    switch(data){
        case 0xAA:
            if(byteCount == 0){
                _serial->write(0xAA);
            }
            byteCount = 0;
            break;
        case 0xFF:
            escNextByte = true;
            break;
        default:
            if(escNextByte){
                // Escaped bytes need to be inverted.
                data = ~data;
                escNextByte = false;
            }
            break;
    }

    return data;
}

long ACIO::autodetectBaud(){
     // Keep trying to receive start bytes at all baud rates until one is found
    int i=0;
    boolean baudFound;

    do {
        i++;    // Select next baud rate to use
        if (i> (sizeof(_baudrates)/sizeof(_baudrates[0])) -1 ){
            i=0;
        }

        //flush in and out buffer
        while (_serial->available()){
            _serial->read();
        }
        _serial->flush();
        _serial->end();
        _serial->begin(baudrates[i]);

        while (_serial->available() < 10);  // Wait to receive 10 bytes for proper test

        // Check if all bytes received have value of 0xAA (else baud rate is wrong)
        baudFound = true;
        for (int j=0;j<10;j++){
            if (_serial->read() != 0xAA){
                baudFound = false;
                break;
            }
        }
    }
    while (!baudFound);

    // Valid baudrate has been detected, and port is open at correct baudrate
    return baudrates[i];
}