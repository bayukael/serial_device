#include "SerialDevice.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <string>
#include <mutex>

namespace bekael{
namespace SerialComm{

enum class ConfigResult{
    SUCCESS,
    DEVICE_IS_CLOSED,
    TERMIOS_ERROR,
    BAUDRATE_IS_NOT_SET,
};

struct SerialDevice::SerialDeviceImpl{
    std::mutex fd_mutex_;
    int device_desc_;
    State status_ = State::CLOSED;
    std::string device_path_;
    BaudRate baud_rate_;
    Parity parity_ = Parity::NONE;
    StopBits stop_bits_ = StopBits::ONE;
    NumOfBitsPerByte num_of_bits_per_byte_ = NumOfBitsPerByte::EIGHT;
    bool use_hardware_flow_control_ = false;
    bool use_software_flow_control_ = false;
    RWMode rw_mode_ = RWMode::BOTH;
    uint8_t vmin_ = 1;
    uint8_t vtime_ = 10;

    ConfigResult configure();
};

ConfigResult SerialDevice::SerialDeviceImpl::configure(){
    if(status_ == State::CLOSED){
        return ConfigResult::DEVICE_IS_CLOSED;
    }
    termios tty;
    if (tcgetattr(device_desc_, &tty) != 0){
        return ConfigResult::TERMIOS_ERROR;
    }

    speed_t baud_rate;
    switch(baud_rate_){
        case BaudRate::B_UNSET :
            return ConfigResult::BAUDRATE_IS_NOT_SET; 

        case BaudRate::B_50 :
            baud_rate = B50;
            break;

        case BaudRate::B_75 :
            baud_rate = B75;
            break;

        case BaudRate::B_110 :
            baud_rate = B110;
            break;

        case BaudRate::B_134 :
            baud_rate = B134;
            break;

        case BaudRate::B_150 :
            baud_rate = B150;
            break;

        case BaudRate::B_200 :
            baud_rate = B200;
            break;

        case BaudRate::B_300 :
            baud_rate = B300;
            break;

        case BaudRate::B_600 :
            baud_rate = B600;
            break;

        case BaudRate::B_1200 :
            baud_rate = B1200;
            break;

        case BaudRate::B_1800 :
            baud_rate = B1800;
            break;

        case BaudRate::B_2400 :
            baud_rate = B2400;
            break;

        case BaudRate::B_4800 :
            baud_rate = B4800;
            break;

        case BaudRate::B_9600 :
            baud_rate = B9600;
            break;

        case BaudRate::B_19200 :
            baud_rate = B19200;
            break;

        case BaudRate::B_38400 :
            baud_rate = B38400;
            break;

        case BaudRate::B_57600 :
            baud_rate = B57600;
            break;

        case BaudRate::B_115200 :
            baud_rate = B115200;
            break;

        case BaudRate::B_230400 :
            baud_rate = B230400;
            break;

        case BaudRate::B_460800 :
            baud_rate = B460800;
            break;

        case BaudRate::B_500000 :
            baud_rate = B500000;
            break;

        case BaudRate::B_576000 :
            baud_rate = B576000;
            break;

        case BaudRate::B_921600 :
            baud_rate = B921600;
            break;

        case BaudRate::B_1000000 :
            baud_rate = B1000000;
            break;

        case BaudRate::B_1152000 :
            baud_rate = B1152000;
            break;

        case BaudRate::B_1500000 :
            baud_rate = B1500000;
            break;

        case BaudRate::B_2000000 :
            baud_rate = B2000000;
            break;

        case BaudRate::B_2500000 :
            baud_rate = B2500000;
            break;

        case BaudRate::B_3000000 :
            baud_rate = B3000000;
            break;

        case BaudRate::B_3500000 :
            baud_rate = B3500000;
            break;

        case BaudRate::B_4000000 :
            baud_rate = B4000000;
            break;
    }
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    if (use_hardware_flow_control_){
        tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
    }
    else {
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    }

    switch (num_of_bits_per_byte_){
    case NumOfBitsPerByte::FIVE :
        tty.c_cflag |= CS5;
        break;
    
    case NumOfBitsPerByte::SIX :
        tty.c_cflag |= CS6;
        break;
    
    case NumOfBitsPerByte::SEVEN :
        tty.c_cflag |= CS7;
        break;
    
    case NumOfBitsPerByte::EIGHT :
        tty.c_cflag |= CS8;
        break;
    }

    switch (parity_){
    case Parity::NONE :
        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        break;
    
    case Parity::ODD :
        tty.c_cflag |= PARODD; // Use odd parity
        break;
    
    case Parity::EVEN :
        tty.c_cflag &= ~PARODD; // Use even parity
        break;
    }

    tty.c_cc[VMIN] = vmin_;
    tty.c_cc[VTIME] = vtime_;

    if (use_software_flow_control_){
        tty.c_iflag |= (IXON | IXOFF | IXANY); // Turn on software flow control
    }
    else{
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
    }

    switch (stop_bits_){
    case StopBits::ONE :
        tty.c_cflag &= ~CSTOPB; // Use only one stop bit used in communication (most common)
        break;

    case StopBits::TWO :
        tty.c_cflag |= CSTOPB; // Use two stop bits used in communication
        break;
    }

    tty.c_cflag |= CREAD; // Enable Receiver
    tty.c_lflag &= ~ICANON; // Use non-canonical mode, i.e. just read the data in raw
    tty.c_lflag &= ~ECHO; // Do not echo input characters
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Turn off special handling of bytes on receive
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    
    if (tcsetattr(device_desc_, TCSANOW, &tty) != 0){
        return ConfigResult::TERMIOS_ERROR;
    }

    return ConfigResult::SUCCESS;
}

bool SerialDevice::setBaudRate(BaudRate val){
    pImpl->baud_rate_ = val;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}

bool SerialDevice::setDevicePath(const std::string& device_path){
    if(pImpl->status_ == State::OPEN){ // We should not change the device path when the device is opened
        return false;
    }
    pImpl->device_path_ = device_path;
    return true;
}

bool SerialDevice::setHardwareFlowControl(bool use){
    pImpl->use_hardware_flow_control_ = use;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}
bool SerialDevice::setNumOfBitsPerByte(NumOfBitsPerByte val){
    pImpl->num_of_bits_per_byte_ = val;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}
bool SerialDevice::setParity(Parity val){
    pImpl->parity_ = val;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}
bool SerialDevice::setReadConfig(uint8_t vmin, uint8_t vtime){
    pImpl->vmin_ = vmin;
    pImpl->vtime_ = vtime;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}

bool SerialDevice::setRWMode(RWMode val){
    if(pImpl->status_ == State::OPEN){ // We should not change the device path when the device is opened
        return false;
    }
    pImpl->rw_mode_ = val;
    return true;
}
bool SerialDevice::setSoftwareFlowControl(bool use){
    pImpl->use_software_flow_control_ = use;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}
bool SerialDevice::setStopBits(StopBits val){
    pImpl->stop_bits_ = val;
    ConfigResult res = pImpl->configure();

    return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
}

State SerialDevice::status(){
    return pImpl->status_;
}

bool SerialDevice::connect(){
    int mode = O_NOCTTY | O_NDELAY;
    switch (pImpl->rw_mode_){
    case RWMode::READ_ONLY :
        mode |= O_RDONLY;
        break;
    
    case RWMode::WRITE_ONLY :
        mode |= O_WRONLY;
        break;
    
    case RWMode::BOTH :
        mode |= O_RDWR;
        break;
    }
    pImpl->device_desc_ = open(pImpl->device_path_.c_str(), mode);
    if (pImpl->device_desc_ < 0){
        pImpl->status_ = State::CLOSED;
        return false;
    }
    pImpl->status_ = State::OPEN;
    ConfigResult res = pImpl->configure();

    if (res != ConfigResult::SUCCESS){
        disconnect();
    }

    return (res == ConfigResult::SUCCESS) ? true : false;
}

bool SerialDevice::disconnect(){
    if (pImpl->status_ == State::OPEN){
        pImpl->status_ = State::CLOSED;
        if (close(pImpl->device_desc_) != 0){
            return false;
        }
    }
    return true;
}

int SerialDevice::readData(uint8_t * read_buffer){
    std::lock_guard<std::mutex> lock(pImpl->fd_mutex_);
    if (pImpl->status_ == State::CLOSED){
        return -1;
    }
    if (pImpl->rw_mode_ == RWMode::WRITE_ONLY){
        return -2;
    }
    return read(pImpl->device_desc_, read_buffer, sizeof(*read_buffer));
}

int SerialDevice::writeData(uint8_t * write_buffer, unsigned int length){
    std::lock_guard<std::mutex> lock(pImpl->fd_mutex_);
    if (pImpl->status_ == State::CLOSED){
        return -1;
    }
    if (pImpl->rw_mode_ == RWMode::READ_ONLY){
        return -2;
    }
    int bytes_written = static_cast<int>(write(pImpl->device_desc_, write_buffer, length));
    tcdrain(pImpl->device_desc_);
    return bytes_written;
}

} // namespace SerialComm
} // namespace bekael