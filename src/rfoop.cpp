#include "rfoop.h"

uint8_t E32_433T30D::calculateCRC8(const uint8_t *data, const size_t& length) const {
    if(sizeof(data) + 1 > MAX_TX_BUFFER_SIZE){
        DebuggerPort->println(F("CRC8 Fonksiyonu Maksimum Paketten Büyük"));
        return E32_CrcBroken;
    }

    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80){
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void E32_433T30D::clearSerialBuffer() const{
    while (RFSerialPort->available() > 0) {
        RFSerialPort->read();
    }
}

Status E32_433T30D::waitAUX(unsigned long timeout) const{
    long startTime = millis();
    #ifdef AUXPin_Set
        while(digitalRead(this->_pinConfs.AUXPin) == LOW){
            if (millis() - startTime > timeout) {
                return E32_Timeout;
            }
            managedDelay(2);
        }

        managedDelay(10);
    #else
        while(millis() - startTime < this->_paramConfs.noAuxTimeOut){
            managedDelay(2);
        }
    #endif

    return E32_Success;
}

void E32_433T30D::managedDelay(unsigned long timeout) const{
    unsigned long t = millis();

    if((unsigned long) (t + timeout) == 0){
        t = 0;
    }

    while((millis()-t) < timeout){
    }
}

Status E32_433T30D::setPinConfig(const int8_t &m0, const int8_t &m1, const int8_t &aux){
    this->_pinConfs.M0Pin = m0;
    this->_pinConfs.M1Pin = m1;
    this->_pinConfs.AUXPin = aux;
    return E32_Success;
}

Status E32_433T30D::setTransmissionMode(const RF_TRANS_MODE &Mode){
    switch (Mode)
    {
    case TRANSPARENTMODE:
        this->_devConfs.RFOption.TransmissionMode = TRANSPARENTMODE;
        break;
    
    case FIXEDMODE:
        this->_devConfs.RFOption.TransmissionMode = FIXEDMODE;
        break;

    case BROADCASTMODE:
        this->_devConfs.RFOption.TransmissionMode = FIXEDMODE;
        break;

    default:
        this->_devConfs.RFOption.TransmissionMode = TRANSPARENTMODE;
        return E32_FailureMode;
    }

    return E32_Success;
}

Status E32_433T30D::setAddresses(const uint8_t &AddHigh, const uint8_t &AddLow){
    if(AddHigh > MAX_FREQ_VAL || AddHigh < MIN_FREQ_VAL){
        return E32_OutOfLimit;
    }
    if(AddLow > MAX_FREQ_VAL || AddLow < MIN_FREQ_VAL){
        return E32_OutOfLimit;
    }

    this->_devConfs.AddHigh = AddHigh;
    this->_devConfs.AddLow = AddLow;
    return E32_Success;
}

Status E32_433T30D::setChannel(const RF_FREQ &channel){
    if(channel > FREQ_441 || channel < FREQ_410){
        return E32_OutOfLimit;
    }

    this->_devConfs.channel = channel;
    return E32_Success;
}

Status E32_433T30D::setTransmissionPower(const RF_TRANS_POWER &power){
    if(!(power > TRANSMISSIONPOWER_21 || power < TRANSMISSIONPOWER_30)){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.TransmissionPower = power;
    return E32_Success;
}

Status E32_433T30D::setIODriver(const RF_IO_MODE &driver){
    if(!(driver > IO_OPENDRAIN || driver < IO_PUSHPULL)){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.IODriver = driver;
    return E32_Success;
}

Status E32_433T30D::setFECSettings(const RF_FEC &fecmode){
    if(!(fecmode > FEC_ON || fecmode < FEC_OFF)){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.FECset = fecmode;
    return E32_Success;
}

Status E32_433T30D::setWirelesWakeup(const RF_WIRELESS &time){
    if(!(time > WIRELESSWAKEUP_2000 || time < WIRELESSWAKEUP_250)){
        return E32_Success;
    }

    this->_devConfs.RFOption.WirelessWakeUp = time;
    return E32_Success;
}

Status E32_433T30D::setUARTParity(const RF_UART_PARITY &paritybyte){
    if(!(paritybyte > UARTPARITY_8E1 || paritybyte << UARTPARITY_8N1)){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.UARTParity = paritybyte;
    return E32_Success;
}

Status E32_433T30D::setUARTBaudRate(const RF_UART_BAUD &baudrate){
    if(!(baudrate > UARTBAUDRATE_115200 || baudrate < UARTBAUDRATE_1200)){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.UARTBaud = baudrate;
    return E32_Success;
}

Status E32_433T30D::setAirDataRate(const RF_AIR_DATA &airdatarate){
    if(!(airdatarate > AIRDATARATE_192k || airdatarate < AIRDATARATE_03k)){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.AirDataRate = airdatarate;
    return E32_Success;
}

Status E32_433T30D::setSerialBaudRateBegin() const{
    switch (this->_devConfs.RFSped.UARTBaud)
    {
    case UARTBAUDRATE_1200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(1200, setSerialParityBegin());
        break;

    case UARTBAUDRATE_2400:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(2400, setSerialParityBegin());
        break;

    case UARTBAUDRATE_4800:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(4800, setSerialParityBegin());
        break;

    case UARTBAUDRATE_9600:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600, setSerialParityBegin());
        break;
    
    case UARTBAUDRATE_19200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(19200, setSerialParityBegin());
        break;

    case UARTBAUDRATE_38400:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(38400, setSerialParityBegin());
        break;

    case UARTBAUDRATE_57600:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(57600, setSerialParityBegin());
        break;

    case UARTBAUDRATE_115200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(115200, setSerialParityBegin());
        break;
    }

    return E32_Success;
}

uint8_t E32_433T30D::setSerialParityBegin() const{
    switch (this->_devConfs.RFSped.UARTParity)
    {
    case UARTPARITY_8N1:
        return SERIAL_8N1;
    
    case UARTPARITY_8E1:
        return SERIAL_8E1;

    case UARTPARITY_8O1:
        return SERIAL_8O1;
    }

    return SERIAL_8N1;
}

Status E32_433T30D::setAuxTimeoutTime(const time_t &time){
    if(time < 30){
        return E32_FailureMode;
    }

    this->_paramConfs.auxTimeout = time;
    return E32_Success;
}

Status E32_433T30D::setNoAuxTimeoutTime(const time_t &time){
    if(time < 30){
        return E32_FailureMode;
    }

    this->_paramConfs.noAuxTimeOut = time;
    return E32_Success;
}

String E32_433T30D::getUARTBaudRate(const byte &uartbaud) const{
    switch (uartbaud)
    {
    case UARTBAUDRATE_1200:
        return F("1200 bps");
        break;
    
    case UARTBAUDRATE_2400:
        return F("2400 bps");
        break;

    case UARTBAUDRATE_4800:
        return F("4800 bps");
        break;

    case UARTBAUDRATE_9600:
        return F("9600 bps (Varsayilan)");
        break;

    case UARTBAUDRATE_19200:
        return F("19200 bps");
        break;

    case UARTBAUDRATE_38400:
        return F("38400 bps");
        break;

    case UARTBAUDRATE_57600:
        return F("57600 bps");
        break;

    case UARTBAUDRATE_115200:
        return F("115200 bps");
        break;
    }
    return F("30 dBm");
}

String E32_433T30D::getUARTParity(const byte &uartparity) const{
    switch (uartparity)
    {
    case UARTPARITY_8N1:
        return F("8 Bit, Parity Yok, 1 Durdurma Biti");
        break;
    
    case UARTPARITY_8E1:
        return F("8 Bit, Çift Parity, 1 Durdurma Biti");
        break;

    case UARTPARITY_8O1:
        return F("8 Bit, Tek Parity, 1 Durdurma Biti");
        break;
    }
    return F("30 dBm");
}

String E32_433T30D::getAirData(const byte &airdata) const{
    switch (airdata)
    {
    case AIRDATARATE_03k:
        return F("0.3k bps");
        break;
    
    case AIRDATARATE_12k:
        return F("1.2k bps");
        break;

    case AIRDATARATE_24k:
        return F("2.4k bps (Varsayilan)");
        break;

    case AIRDATARATE_48k:
        return F("4.8k bps");
        break;

    case AIRDATARATE_96k:
        return F("9.6k bps");
        break;

    case AIRDATARATE_192k:
        return F("19.2k bps");
        break;
    }
    return F("30 dBm");
}

String E32_433T30D::getTransmissionType(const byte &transmissiontype) const{
    switch (transmissiontype)
    {
    case TRANSPARENTMODE:
        return F("Seffaf Mod");
        break;
    
    case FIXEDMODE:
        return F("Sabit Kanal Modu");
        break;
    }
    return F("30 dBm");
}

String E32_433T30D::getIOMode(const byte &iotype) const{
    switch (iotype)
    {
    case IO_OPENDRAIN:
        return F("IO Open Drain Modu");
        break;
    
    case IO_PUSHPULL:
        return F("IO Push Pull Modu");
        break;
    }  
    return F("30 dBm");
}

String E32_433T30D::getWirelessWakeup(const byte &wireless) const{
    switch (wireless)
    {
    case WIRELESSWAKEUP_250:
        return F("250 ms");
        break;
    
    case WIRELESSWAKEUP_500:
        return F("500 ms");
        break;

    case WIRELESSWAKEUP_750:
        return F("750 ms");
        break;
    
    case WIRELESSWAKEUP_1000:
        return F("1000 ms");
        break;
    
    case WIRELESSWAKEUP_1250:
        return F("1250 ms");
        break;
    
    case WIRELESSWAKEUP_1500:
        return F("1500 ms");
        break;

    case WIRELESSWAKEUP_1750:
        return F("1750 ms");
        break;

    case WIRELESSWAKEUP_2000:
        return F("2000 ms");
        break;
    } 
    return F("30 dBm");
}

String E32_433T30D::getFECFilter(const byte &fecbyte) const{
    switch (fecbyte)
    {
    case FEC_ON:
        return F("Aktif");
        break;
    
    case FEC_OFF:
        return F("Devre Disi");
        break;
    } 
    return F("30 dBm"); 
}

String E32_433T30D::getTranmissionPower(const byte &transmissionpower) const{
    switch (transmissionpower)
    {
    case TRANSMISSIONPOWER_21:
        return F("21 dBm");
        break;
    
    case TRANSMISSIONPOWER_24:
        return F("24 dBm");
        break;
    
    case TRANSMISSIONPOWER_27:
        return F("27 dBm");
        break;
    
    case TRANSMISSIONPOWER_30:
        return F("30 dBm");
        break;
    }
    return F("30 dBm");
}

float E32_433T30D::airDataRateEnum2Value(const RF_AIR_DATA &dataRate) const{
    if(dataRate == AIRDATARATE_03k){
        return 0.3;
    }
    else if(dataRate == AIRDATARATE_12k){
        return 1.2;
    }
    else if(dataRate == AIRDATARATE_24k){
        return 2.4;
    }
    else if(dataRate == AIRDATARATE_48k){
        return 4.8;
    }
    else if(dataRate == AIRDATARATE_96k){
        return 9.6;
    }
    else if(dataRate == AIRDATARATE_192k){
        return 19.2;
    }
    else{
        return -1;
    }
}

long E32_433T30D::UARTRateEnum2Value(const RF_UART_BAUD &dataRate) const{
    if(dataRate == UARTBAUDRATE_1200){
        return 1200;
    }
    else if(dataRate == UARTBAUDRATE_2400){
        return 2400;
    }
    else if(dataRate == UARTBAUDRATE_4800){
        return 4800;
    }
    else if(dataRate == UARTBAUDRATE_9600){
        return 9600;
    }
    else if(dataRate == UARTBAUDRATE_19200){
        return 19200;
    }
    else if(dataRate == UARTBAUDRATE_38400){
        return 38400;
    }
    else if(dataRate == UARTBAUDRATE_57600){
        return 57600;
    }
    else if(dataRate == UARTBAUDRATE_115200){
        return 115200;
    }
}

time_t E32_433T30D::calculatePacketSendTime(const size_t &packetSize) const{
    uint16_t packetBitSize = packetSize * 8;
    time_t packetTime = 0;

    // Air Data Rate Time
    packetTime += (1000 * packetBitSize) / (airDataRateEnum2Value(this->_devConfs.RFSped.AirDataRate) * 1000);

    // UART Time
    packetTime += (1000 * packetBitSize) / UARTRateEnum2Value(this->_devConfs.RFSped.UARTBaud);

    return packetTime;
}

Status E32_433T30D::setSettings(void) const{
    #ifdef MPin_Set
        uint8_t SpedByte = 0, OptionByte = 0;
        uint8_t MesArr[6];

        SpedByte = (this->_devConfs.RFSped.UARTParity << 6) | (this->_devConfs.RFSped.UARTBaud << 3) | (this->_devConfs.RFSped.AirDataRate);
        OptionByte = (this->_devConfs.RFOption.TransmissionMode << 7) | (this->_devConfs.RFOption.IODriver << 6) |
                    (this->_devConfs.RFOption.WirelessWakeUp << 3)   | (this->_devConfs.RFOption.FECset << 2)   | 
                    (this->_devConfs.RFOption.TransmissionPower);

        RF_WaitAUX();

        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600);
        managedDelay(200);

        RF_Operating_Config();

        managedDelay(20);

        MesArr[0] = 0xC0;
        MesArr[1] = this->_devConfs.AddHigh;
        MesArr[2] = this->_devConfs.AddLow;
        MesArr[3] = SpedByte;
        MesArr[4] = this->_devConfs.channel;
        MesArr[5] = OptionByte;

        RFSerialPort->write((uint8_t *)MesArr, sizeof(MesArr) / sizeof(MesArr[0]));

        RF_WaitAUX();

        managedDelay(750);

        RF_Operating_Normal();

        setSerialBaudRateBegin();
        managedDelay(200);

        managedDelay(20);

        return E32_Success;
    #else
        this->DebuggerPort->println(F("M0 and M1 pins not set. Device's configs cannot be change."));
        return E32_FailureMode;
    #endif
};

Status E32_433T30D::getSettings(void){
    #ifdef MPin_Set
        this->tempConfig = new ConfigRF;

        clearSerialBuffer();
        uint8_t SpedByte = 0, OptionByte = 0;
        uint8_t MesArr[6], sendpack[3];

        RF_WaitAUX();

        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600);
        managedDelay(200);
        RF_Operating_Config();

        managedDelay(100);

        sendpack[0] = 0xC1;
        sendpack[1] = 0xC1;
        sendpack[2] = 0xC1;

        RFSerialPort->write((uint8_t *)sendpack, sizeof(sendpack) / sizeof(sendpack[0]));
        
        long startTime = millis();
        while(RFSerialPort->available() < sizeof(MesArr)){
            if(millis() - startTime > this->_paramConfs.serialTimeout){
                return E32_Timeout;
            }
            managedDelay(20);
        }

        RFSerialPort->readBytes(MesArr, sizeof(MesArr));

        this->tempConfig->AddressHigh = MesArr[1];
        this->tempConfig->AddressLow = MesArr[2];
        SpedByte = MesArr[3];
        this->tempConfig->Channel = MesArr[4];
        OptionByte = MesArr[5];

        this->tempConfig->RFSped.UARTParity = (RF_UART_PARITY)((SpedByte >> 6) & 0b11);
        this->tempConfig->RFSped.UARTBaud = (RF_UART_BAUD)((SpedByte >> 3) & 0b111);
        this->tempConfig->RFSped.AirDataRate = (RF_AIR_DATA)((SpedByte) & 0b111);

        this->tempConfig->RFOption.TransmissionMode = (RF_TRANS_MODE)((OptionByte >> 7) & 0b1);
        this->tempConfig->RFOption.IODriver = (RF_IO_MODE)((OptionByte >> 6) & 0b1);
        this->tempConfig->RFOption.WirelessWakeUp = (RF_WIRELESS)((OptionByte >> 3) & 0b111);
        this->tempConfig->RFOption.FECset = (RF_FEC)((OptionByte >> 2) & 0b1);
        this->tempConfig->RFOption.TransmissionPower = (RF_TRANS_POWER)((OptionByte) & 0b11);

        RF_Operating_Normal();

        setSerialBaudRateBegin();
        managedDelay(200);

        managedDelay(100);

        return E32_Success;
    #else
        this->DebuggerPort->println(F("M0 and M1 pins not set. Device's configs cannot be fetch."));
    #endif
}

Status E32_433T30D::viewSettings(void) const{
    DebuggerPort->println(F("------------------------------------------------------"));
    DebuggerPort->print(F("Yuksek Adres: "));    DebuggerPort->println(this->_devConfs.AddHigh);

    DebuggerPort->print(F("Dusuk Adres: "));    DebuggerPort->println(this->_devConfs.AddLow);

    DebuggerPort->print(F("Kanal: "));    DebuggerPort->print(this->_devConfs.channel);
    DebuggerPort->print(F(" - "));    DebuggerPort->print(410+this->_devConfs.channel);   DebuggerPort->println(F(" MHz"));
    DebuggerPort->println();

    DebuggerPort->println(F("Sped Ayarlari"));
    DebuggerPort->print(F("  UART Baud Rate: "));     DebuggerPort->println(getUARTBaudRate(this->_devConfs.RFSped.UARTBaud));
    DebuggerPort->print(F(" UART Parity: "));     DebuggerPort->println(getUARTParity(this->_devConfs.RFSped.UARTParity));
    DebuggerPort->print(F("  Air Data Rate: "));     DebuggerPort->println(getAirData(this->_devConfs.RFSped.AirDataRate));
    DebuggerPort->println();

    DebuggerPort->println(F("Option Ayarlari"));
    DebuggerPort->print(F("  Transfer Turu: "));      DebuggerPort->println(getTransmissionType(this->_devConfs.RFOption.TransmissionMode));
    DebuggerPort->print(F("  IO Turu: "));        DebuggerPort->println(getIOMode(this->_devConfs.RFOption.IODriver));
    DebuggerPort->print(F("  Wireless Uyanma Suresi: "));     DebuggerPort->println(getWirelessWakeup(this->_devConfs.RFOption.WirelessWakeUp));
    DebuggerPort->print(F("  FEC Filtresi: "));       DebuggerPort->println(getFECFilter(this->_devConfs.RFOption.FECset));
    DebuggerPort->print(F("  Aktarim Gucu: "));       DebuggerPort->println(getTranmissionPower(this->_devConfs.RFOption.TransmissionPower));
    DebuggerPort->println();
    DebuggerPort->println(F("------------------------------------------------------"));
}

Status E32_433T30D::receiveSingleData(uint8_t *data) const {
    RF_WaitAUX();

    unsigned long t = millis();
    while(this->RFSerialPort->available() == 0){
        if(millis() - t > 1000){
            this->DebuggerPort->println(F("Veri Okuma Zaman Asimina Ugradi..."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    RF_WaitAUX();

    *data = this->RFSerialPort->read();
 
    clearSerialBuffer();
    this->DebuggerPort->println(F("Veri Alindi..."));
    return E32_Success;
}

Status E32_433T30D::receiveDataPacket(uint8_t *data, const size_t& size) const{
    unsigned long t = millis();

    while(this->RFSerialPort->available() < size + 1){
        if(this->RFSerialPort->available() == 0 && millis() - t > 100){
            //this->DebuggerPort->println(F("Herhangi bir Veri Paketi gelmedi..."));
            return E32_NoMessage;
        }
        else if(millis() - t > this->_paramConfs.serialTimeout){
            //this->DebuggerPort->println(F("Veri okuma zaman asimina ugradi..."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    RF_WaitAUX();

    this->RFSerialPort->readBytes(data, size + 1);

    uint8_t crc = 0x00;
    crc = calculateCRC8(data, size);

    if(crc != data[size]){
        this->DebuggerPort->println(F("Paket CRC Uyuşmuyor"));
        return E32_CrcBroken;
    }

    clearSerialBuffer();
    return E32_Success;
}

Status E32_433T30D::sendFixedSingleData(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t& data) {
    RF_PackageTimerCheck();
    
    uint8_t packet[4];
    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    packet[3] = data;

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime; 
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

    RF_WaitAUX();

    clearSerialBuffer();

    return E32_Success;
}

Status E32_433T30D::sendTransparentSingleData(const uint8_t& data){
    RF_PackageTimerCheck();

    RF_WaitAUX();

    time_t packageTime = calculatePacketSendTime(sizeof(data));

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write(data);

    RF_WaitAUX();
    
    clearSerialBuffer();

    return E32_Success;
}

Status E32_433T30D::sendFixedDataPacket(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t *data, const size_t& size) {
    RF_PackageTimerCheck();
    
    if(size > MAX_TX_BUFFER_SIZE_FIXED_CRC){
        return E32_BigPacket;
    }

    uint8_t packetSize = size + 4;

    uint8_t packet[packetSize];

    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    memcpy(&packet[3], data, size);

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    uint8_t crc;
    crc = this->calculateCRC8(data, size);
    packet[packetSize - 1] = crc;

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet));

    RF_WaitAUX();

    managedDelay(5);

    clearSerialBuffer();

    return E32_Success;
}

Status E32_433T30D::sendBroadcastDataPacket(const uint8_t& Channel, const uint8_t *data, const size_t& size) {
    return this->sendFixedDataPacket(0x00, 0x00, Channel, data, size);
}

Status E32_433T30D::sendTransparentDataPacket(uint8_t *data, const size_t& size){
    RF_PackageTimerCheck();

    if(size > MAX_TX_BUFFER_SIZE_CRC){
        return E32_BigPacket;
    }

    uint8_t packetSize = size + 1;

    uint8_t packet[packetSize];
    memcpy(packet, data, size);

    uint8_t crc = calculateCRC8(packet, size);
    packet[(sizeof(data) / sizeof(data[0]))] = crc;

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet));

    RF_WaitAUX();

    managedDelay(5);

    clearSerialBuffer();

    return E32_Success;
}

Status E32_433T30D::packageTimerCheck() const{
    if(millis() <= this->_paramConfs.packetEndTimeStamp){
        this->DebuggerPort->println(F("Delay duration is not enough to send this data packet !!!"));
        this->DebuggerPort->println(F("Previous Packet Still Sending.."));
        this->DebuggerPort->println(F("Try to increase UART or Air Data Rate. Or decrease data packet size."));
        this->DebuggerPort->println(F("Working in this config can cause unwanted delays and can cause a damage on device..."));
        this->DebuggerPort->print(F("Delay Time : "));this->DebuggerPort->print(millis() - this->_paramConfs.packetStartTimeStamp);this->DebuggerPort->print(F("   "));
        this->DebuggerPort->print(F("Previous Package's Start Time : "));this->DebuggerPort->println(this->_paramConfs.packetStartTimeStamp);
        this->DebuggerPort->print(F("Previous Package's End Time : "));this->DebuggerPort->println(this->_paramConfs.packetEndTimeStamp);
        this->DebuggerPort->print(F("Previous Package's Duration : "));this->DebuggerPort->println(this->_paramConfs.packetEndTimeStamp - this->_paramConfs.packetStartTimeStamp);
        return E32_NoPackageTime;
    }
    return E32_Success;
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort){
    this->RFSerialPort = serialPort;

    this->RFSerialPort->begin(9600);

    setPinConfig(-1, -1, -1);
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->RFSerialPort->begin(9600);
    this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(-1, -1, -1);
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = serialPort;

    setPinConfig(M0_Pin, M1_Pin, -1);
    #define MPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(M0_Pin, M1_Pin, -1);
    #define MPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort, const uint8_t &AUX_Pin, const uint8_t &M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = serialPort;

    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    #define MPin_Set;
    #define AUXPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    #define MPin_Set;
    #define AUXPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    #define DynamicRFPort

    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    #define MPin_Set;
    #define AUXPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    #define DynamicRFPort

    setPinConfig(M0_Pin, M1_Pin, -1);
    #define MPin_Set;

    this->getSettings();
    this->tempConftoDevice();
}

E32_433T30D::E32_433T30D(const uint8_t& TX_Pin, const uint8_t& RX_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    #define DynamicRFPort

    this->RFSerialPort->begin(9600);

    setPinConfig(-1, -1, -1);
}

E32_433T30D::~E32_433T30D(){
    #ifdef DynamicRFPort
        delete this->RFSerialPort;
    #endif

    #ifdef MPin_Set
        delete this->tempConfig;
    #endif
}

Status E32_433T30D::tempConftoDevice(){
    this->_devConfs.RFOption = this->tempConfig->RFOption;
    this->_devConfs.RFSped = this->tempConfig->RFSped;
    this->_devConfs.AddHigh = this->tempConfig->AddressHigh;
    this->_devConfs.AddLow = this->tempConfig->AddressLow;
    this->_devConfs.channel = this->tempConfig->Channel;
    return E32_Success;
}

Status E32_433T30D::RFBegin(const uint8_t& HighAddress, const uint8_t& LowAddress, const uint8_t& channel, 
                            const RF_UART_PARITY& parity, const RF_UART_BAUD& baud, const RF_AIR_DATA& airdata,
                            const RF_TRANS_MODE& transmode, const RF_IO_MODE& IOmode, const RF_WIRELESS& wirelesswake,
                            const RF_FEC& fecmode, const RF_TRANS_POWER& transpower){
                                this->_devConfs.AddHigh = HighAddress;
                                this->_devConfs.AddLow = LowAddress;
                                this->_devConfs.channel = channel;
                                this->_devConfs.RFSped.UARTParity = parity;
                                this->_devConfs.RFSped.UARTBaud = baud;
                                this->_devConfs.RFSped.AirDataRate = airdata;
                                this->_devConfs.RFOption.TransmissionMode = transmode;
                                this->_devConfs.RFOption.IODriver = IOmode;
                                this->_devConfs.RFOption.WirelessWakeUp = wirelesswake;
                                this->_devConfs.RFOption.FECset = fecmode;
                                this->_devConfs.RFOption.TransmissionPower = transpower;

                                this->setSettings();
                            }

Status E32_433T30D::RFStart() const{
    setSettings();
}