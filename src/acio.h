// ACIO Protocol library
// NKS - 2022
// Based on work by Nadeflore/ACreal_IO
// https://github.com/Nadeflore/ACreal_IO

#ifndef ACIO_H
#define ACIO_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Debugging tools: Use with caution as they may break comms with some hosts
#define DBG_SERIAL Serial           // CHANGE THIS or disable logging if you do not want debug output on this port.
//#define ACIO_VERBOSE              // Enables library logging, but might break comms when running as device
#define ACIO_ERROR
//#define ACIO_VERBOSE_LOG_FRAME    // Enables frame logging but will break comms with certain hosts
//#define ACIO_VERBOSE_LOG_CMG      // Same as above but for command packet decoder
//#define ACIO_VERBOSE_CMD

#ifdef ACIO_VERBOSE
#warning "ACIO Verbose mode is enabled. You may experience communication issues or disconnects when running in device mode!"
#endif

#ifdef ACIO_VERBOSE_CMD
#ifdef ACIO_VERBOSE_LOG_CMD
#ifdef ACIO_VERBOSE_LOG_FRAME
#warning "ACIO Frame/Command logging is very slow and *will* break device mode!"
#endif
#endif
#endif

#define ACIO_MIN_SIZE 6             // All ACIO packets are garenteed to be a minimum of 6 bytes long
#define ACIO_TIMEOUT_PERIOD 50000   // Time to wait before assuming bus is dead (wrong baud?)
#define ACIO_AUTODETECT_BAUD 1

#define ACIO_DEVICE_NODE    0
#define ACIO_HOST_NODE      1

struct ACIO_Frame {
    bool    fullyReceived = false;  // Should only be true when size of bytes match received data length
    uint8_t preamble = 0xAA;
    uint8_t nodeID = 0x00;
    uint8_t command = 0x00;
    uint8_t frameID = 0x00;
    uint8_t numBytes = 0;
    uint8_t currentByte = 0;          // ACIO is minimum 6 bytes long, plus however many data bytes are specified.
    uint8_t data[250];             // Data byte buffer. Probably way more than actually needed.
    uint8_t sumByte = 0;
};

class ACIO {
    public:
    ACIO(HardwareSerial &_ser, byte mode = ACIO_DEVICE_NODE);


    void reset();
    int begin();
    int begin(bool waitForBaud, int bR = 19200);
    int update();
    int readFrame(ACIO_Frame &_frame);
    int writeFrame(ACIO_Frame &_frame);
    long autodetectBaud();

    void disableAutoRestart(bool s){_disableRestart = s};   // Pass true to disable auto-restarting the bus

    private:
    _ACIOready = false;         // Is ACIO bus ready to use
    ACIO_Frame _rxBuffer;        // Packet in memory
    HardwareSerial* _serial;    // UART port to use
    
    bool _disableRestart = false;   // If bus crashes and this is true, the library will stop working until the bus is reinitialised.
    bool _sHost = false;
    long _currentBaud = 0;       // Baudrate currently being used by ACIO bus.
    const long _baudrates[] = {57600,38400,19200};     // ACIO baudrates supported
    unsigned long _lastReceivedTime = 0;               // If timeout is exceeded, reset self

    byte calcSumByte(ACIO_Frame &_frame);
    byte readByte();
}