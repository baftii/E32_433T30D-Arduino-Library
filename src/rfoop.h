#include <Arduino.h>
#include <HardwareSerial.h>

#define MAX_FREQ_VAL 0xFF
#define MIN_FREQ_VAL 0x00

#define MAX_TX_BUFFER_SIZE 58L
#define MAX_TX_BUFFER_SIZE_CRC MAX_TX_BUFFER_SIZE - 1
#define MAX_TX_BUFFER_SIZE_FIXED_CRC MAX_TX_BUFFER_SIZE - 4

#define RF_Operating_Config() {digitalWrite(this->_pinConfs.M0Pin, HIGH); digitalWrite(this->_pinConfs.M1Pin, HIGH);}
#define RF_Operating_Normal() {digitalWrite(this->_pinConfs.M0Pin, LOW); digitalWrite(this->_pinConfs.M1Pin, LOW);}
#define RF_WaitAUX() {if(waitAUX(this->_paramConfs.auxTimeout) == E32_Timeout)return E32_Timeout;}
#define RF_PackageTimerCheck() {if(packageTimerCheck() == E32_NoPackageTime)return E32_NoPackageTime;}

typedef enum Error_Status : uint8_t{
    E32_Success = 1,
    E32_Timeout,
    E32_CrcBroken,
    E32_FailureMode,
    E32_NoMessage,
    E32_BigPacket,
    E32_BrokenGetSet,
    E32_NoPackageTime,
    E32_OutOfLimit
} Status;

typedef enum RF_FREQ : uint8_t{
    FREQ_410 = 0x00,
    FREQ_411 = 0x01,
    FREQ_412 = 0x02,
    FREQ_413 = 0x03,
    FREQ_414 = 0x04,
    FREQ_415 = 0x05,
    FREQ_416 = 0x06,
    FREQ_417 = 0x07,
    FREQ_418 = 0x08,
    FREQ_419 = 0x09,
    FREQ_420 = 0x0A,
    FREQ_421 = 0x0B,
    FREQ_422 = 0x0C,
    FREQ_423 = 0x0D,
    FREQ_424 = 0x0E,
    FREQ_425 = 0x0F,
    FREQ_426 = 0x10,
    FREQ_427 = 0x11,
    FREQ_428 = 0x12,
    FREQ_429 = 0x13,
    FREQ_430 = 0x14,
    FREQ_431 = 0x15,
    FREQ_432 = 0x16,
    FREQ_433 = 0x17,
    FREQ_434 = 0x18,
    FREQ_435 = 0x19,
    FREQ_436 = 0x1A,
    FREQ_437 = 0x1B,
    FREQ_438 = 0x1C,
    FREQ_439 = 0x1D,
    FREQ_440 = 0x1E,
    FREQ_441 = 0x1F
}RF_FREQ;

typedef enum RF_DEBUGGER_UART_PARITY{
    DEBUGGER_UART_PARITY_8N1 = SERIAL_8N1,
    DEBUGGER_UART_PARITY_8O1 = SERIAL_8O1,
    DEBUGGER_UART_PARITY_8E1 = SERIAL_8E1
}RF_DEBUGGER_UART_PARITY;

/*
------------------------
 Ayar Öntanim Makrolari
------------------------
*/
// SPED 7, 6 bit
typedef enum RF_UART_PARITY : uint8_t{
    UARTPARITY_8N1 = 0b00,
    UARTPARITY_8O1 = 0b01,
    UARTPARITY_8E1 = 0b10,
}RF_UART_PARITY;

// SPED 5, 4, 3 bit
typedef enum RF_UART_BAUD : uint8_t{
    UARTBAUDRATE_1200 = 0b000,
    UARTBAUDRATE_2400 = 0b001,
    UARTBAUDRATE_4800 = 0b010,
    UARTBAUDRATE_9600 = 0b011,
    UARTBAUDRATE_19200 = 0b100,
    UARTBAUDRATE_38400 = 0b101,
    UARTBAUDRATE_57600 = 0b110,
    UARTBAUDRATE_115200 = 0b111,
}RF_UART_BAUD;

// SPED 2, 1, 0
typedef enum RF_AIR_DATA : uint8_t{
    AIRDATARATE_03k = 0b000,
    AIRDATARATE_12k = 0b001,
    AIRDATARATE_24k = 0b010,
    AIRDATARATE_48k = 0b011,
    AIRDATARATE_96k = 0b100,
    AIRDATARATE_192k = 0b101,
}RF_AIR_DATA;

// OPTION 7 bit
typedef enum RF_TRANS_MODE : uint8_t{
    TRANSPARENTMODE = 0b0,
    FIXEDMODE = 0b1,
    BROADCASTMODE = 0b11111,
}RF_TRANS_MODE;

// OPTION 6 bit
typedef enum RF_IO_MODE : uint8_t{
    IO_PUSHPULL = 0b0,
    IO_OPENDRAIN = 0b1,
}RF_IO_MODE;

// OPTION 5, 4, 3 bit
typedef enum RF_WIRELESS : uint8_t{
    WIRELESSWAKEUP_250 = 0b000,
    WIRELESSWAKEUP_500 = 0b001,
    WIRELESSWAKEUP_750 = 0b010,
    WIRELESSWAKEUP_1000 = 0b011,
    WIRELESSWAKEUP_1250 = 0b100,
    WIRELESSWAKEUP_1500 = 0b101,
    WIRELESSWAKEUP_1750 = 0b110,
    WIRELESSWAKEUP_2000 = 0b111,
}RF_WIRELESS;

// OPTION 2 bit
typedef enum RF_FEC : uint8_t{
    FEC_OFF = 0b0,
    FEC_ON = 0b1,
}RF_FEC;

// OPTION 1, 0 bit
typedef enum RF_TRANS_POWER : uint8_t{
    TRANSMISSIONPOWER_30 = 0b00,
    TRANSMISSIONPOWER_27 = 0b01,
    TRANSMISSIONPOWER_24 = 0b10,
    TRANSMISSIONPOWER_21 = 0b11,
}RF_TRANS_POWER;

/* 
--------------------
 Struct Tanimlari
--------------------
*/
struct Sped{
    RF_UART_PARITY UARTParity = UARTPARITY_8N1;
    RF_UART_BAUD UARTBaud = UARTBAUDRATE_9600;
    RF_AIR_DATA AirDataRate = AIRDATARATE_24k;
};

struct Option{
    RF_TRANS_MODE TransmissionMode = TRANSPARENTMODE;
    RF_IO_MODE IODriver = IO_PUSHPULL;
    RF_WIRELESS WirelessWakeUp = WIRELESSWAKEUP_250;
    RF_FEC FECset = FEC_ON;
    RF_TRANS_POWER TransmissionPower = TRANSMISSIONPOWER_30;
};

struct ConfigRF{
    struct Sped RFSped;
    struct Option RFOption;
    uint8_t Channel;
    uint8_t AddressHigh;
    uint8_t AddressLow;
};

class E32_433T30D{
private:
    // Config Variables
    HardwareSerial *RFSerialPort = NULL;
    HardwareSerial *DebuggerPort = NULL;
    ConfigRF *tempConfig = NULL;

    struct _paramConfs{
        time_t auxTimeout = 1000;
        time_t noAuxTimeOut = 30;
        time_t serialTimeout = 1000;
        time_t packetStartTimeStamp = 0;
        time_t packetEndTimeStamp = 0;
    }_paramConfs;

    struct _debuggerConfs{
        unsigned long baudRate = 9600;
        RF_DEBUGGER_UART_PARITY parity = DEBUGGER_UART_PARITY_8N1;
    }_debuggerConfs;
    
    struct _devConfs{
        struct Sped RFSped;
        struct Option RFOption;
        uint8_t channel;
        uint8_t AddHigh;
        uint8_t AddLow;
    }_devConfs;
    
    // Pin Variables
    struct _pinConfs{
        int16_t M0Pin;
        int16_t M1Pin;
        int16_t AUXPin;
    }_pinConfs;

    // Private Functions
    void clearSerialBuffer() const;
    Status waitAUX(unsigned long timeout) const;
    void managedDelay(unsigned long timeout) const;
    Status setSettings(void) const;
    Status getSettings(void);

    String getTranmissionPower(const byte &transmissionpower) const;
    String getFECFilter(const byte &fecbyte) const;
    String getWirelessWakeup(const byte &wireless) const;
    String getIOMode(const byte &iotype) const;
    String getTransmissionType(const byte &transmissiontype) const;
    String getAirData(const byte &airdata) const;
    String getUARTParity(const byte &uartparity) const;
    String getUARTBaudRate(const byte &uartbaud) const;

    float airDataRateEnum2Value(const RF_AIR_DATA &dataRate) const;
    long UARTRateEnum2Value(const RF_UART_BAUD &dataRate) const;
    time_t calculatePacketSendTime(const size_t &packetSize) const;
    Status setPinConfig(const int8_t &m0, const int8_t &m1, const int8_t &aux);
    Status setSerialBaudRateBegin() const;
    uint8_t setSerialParityBegin() const;
    Status packageTimerCheck() const;
    Status tempConftoDevice();
public:

    // Constructors
    E32_433T30D(HardwareSerial *serialPort);
    E32_433T30D(HardwareSerial *serialPort, HardwareSerial *debugger);
    E32_433T30D(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E32_433T30D(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger);
    E32_433T30D(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E32_433T30D(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger);
    E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin);

    // Destructor
    ~E32_433T30D();

    // Public
    uint8_t calculateCRC8(const uint8_t *data, const size_t& length) const;
    Status receiveSingleData(uint8_t *data) const;
    Status receiveDataPacket(uint8_t *data, const size_t& size) const;
    Status sendFixedSingleData(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t& data);
    Status sendTransparentSingleData(const uint8_t& data);
    Status sendFixedDataPacket(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t *data, const size_t& size);
    Status sendBroadcastDataPacket(const uint8_t& Channel, const uint8_t *data, const size_t& size);
    Status sendTransparentDataPacket(uint8_t *data, const size_t& size);

    // Setters
    Status setTransmissionMode(const RF_TRANS_MODE &Mode);
    Status setAddresses(const uint8_t &AddHigh, const uint8_t &AddLow);
    Status setChannel(const RF_FREQ &channel);
    Status setTransmissionPower(const RF_TRANS_POWER &power);
    Status setIODriver(const RF_IO_MODE &driver);
    Status setFECSettings(const RF_FEC &fecmode);
    Status setWirelesWakeup(const RF_WIRELESS &time);
    Status setUARTParity(const RF_UART_PARITY &paritybyte);
    Status setUARTBaudRate(const RF_UART_BAUD &baudrate);
    Status setAirDataRate(const RF_AIR_DATA &airdatarate);
    Status setAuxTimeoutTime(const time_t &time);
    Status setNoAuxTimeoutTime(const time_t &time);

    Status viewSettings() const;

    Status RFBegin(const uint8_t& HighAddress, const uint8_t& LowAddress, const uint8_t& channel, 
                   const RF_UART_PARITY& parity, const RF_UART_BAUD& baud, const RF_AIR_DATA& airdata,
                   const RF_TRANS_MODE& transmode, const RF_IO_MODE& IOmode, const RF_WIRELESS& wirelesswake,
                   const RF_FEC& fecmode, const RF_TRANS_POWER& transpower);
    Status RFStart() const;
};
