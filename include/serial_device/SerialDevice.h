#pragma once

#include <string>
#include <stdint.h>
#include <memory>

namespace bayukael{
namespace SerialComm{

enum class BaudRate{
    B_UNSET,
    B_50,
    B_75,
    B_110,
    B_134,
    B_150,
    B_200,
    B_300,
    B_600,
    B_1200,
    B_1800,
    B_2400,
    B_4800,
    B_9600,
    B_19200,
    B_38400,
    B_57600,
    B_115200,
    B_230400,
    B_460800,
    B_500000,
    B_576000,
    B_921600,
    B_1000000,
    B_1152000,
    B_1500000,
    B_2000000,
    B_2500000,
    B_3000000,
    B_3500000,
    B_4000000
};

enum class NumOfBitsPerByte{
    FIVE,
    SIX,
    SEVEN,
    EIGHT,
};

enum class Parity{
    NONE,
    ODD,
    EVEN,
};

enum class RWMode{
    READ_ONLY,
    WRITE_ONLY,
    BOTH,
};

enum class State{
    CLOSED,
    OPEN,

};

enum class StopBits{
    ONE,
    TWO,
};


class SerialDevice{
public:
    SerialDevice();
    ~SerialDevice();

    bool setBaudRate(BaudRate);
    bool setDevicePath(const std::string& device_path);
    bool setHardwareFlowControl(bool);
    bool setNumOfBitsPerByte(NumOfBitsPerByte);
    bool setParity(Parity);
    bool setReadConfig(uint8_t vmin, uint8_t vtime);
    bool setRWMode(RWMode);
    bool setSoftwareFlowControl(bool);
    bool setStopBits(StopBits);
    State status();
    bool connect();
    bool disconnect();
    int readData(uint8_t * read_buffer);
    int writeData(uint8_t * write_buffer, unsigned int length);

private:
    struct SerialDeviceImpl;
    std::unique_ptr<SerialDeviceImpl> p_impl_;
};

} // namespace SerialComm
} // namespace bayukael