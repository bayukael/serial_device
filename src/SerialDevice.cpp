#include "SerialDevice.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <mutex>
#include <string>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

namespace bayukael
{
  namespace serial_comm
  {

    enum class ConfigResult {
      SUCCESS,
      DEVICE_IS_CLOSED,
      TERMIOS_ERROR,
      BAUDRATE_IS_NOT_SET,
    };

    struct SerialDevice::SerialDeviceImpl {
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
      uint8_t vmin_ = 1;   // read() will block until it gets this number of byte
      uint8_t vtime_ = 10; // read() will block until this number of time in deciseconds. 10 deciseconds means 1 second.

      ConfigResult configure();
    };

    ConfigResult SerialDevice::SerialDeviceImpl::configure()
    {
      if (status_ == State::CLOSED) {
        return ConfigResult::DEVICE_IS_CLOSED;
      }
      termios tty;
      if (tcgetattr(device_desc_, &tty) != 0) {
        return ConfigResult::TERMIOS_ERROR;
      }

      speed_t baud_rate;
      switch (baud_rate_) {
        case BaudRate::B_UNSET: return ConfigResult::BAUDRATE_IS_NOT_SET;

        case BaudRate::B_50: baud_rate = B50; break;

        case BaudRate::B_75: baud_rate = B75; break;

        case BaudRate::B_110: baud_rate = B110; break;

        case BaudRate::B_134: baud_rate = B134; break;

        case BaudRate::B_150: baud_rate = B150; break;

        case BaudRate::B_200: baud_rate = B200; break;

        case BaudRate::B_300: baud_rate = B300; break;

        case BaudRate::B_600: baud_rate = B600; break;

        case BaudRate::B_1200: baud_rate = B1200; break;

        case BaudRate::B_1800: baud_rate = B1800; break;

        case BaudRate::B_2400: baud_rate = B2400; break;

        case BaudRate::B_4800: baud_rate = B4800; break;

        case BaudRate::B_9600: baud_rate = B9600; break;

        case BaudRate::B_19200: baud_rate = B19200; break;

        case BaudRate::B_38400: baud_rate = B38400; break;

        case BaudRate::B_57600: baud_rate = B57600; break;

        case BaudRate::B_115200: baud_rate = B115200; break;

        case BaudRate::B_230400: baud_rate = B230400; break;

        case BaudRate::B_460800: baud_rate = B460800; break;

        case BaudRate::B_500000: baud_rate = B500000; break;

        case BaudRate::B_576000: baud_rate = B576000; break;

        case BaudRate::B_921600: baud_rate = B921600; break;

        case BaudRate::B_1000000: baud_rate = B1000000; break;

        case BaudRate::B_1152000: baud_rate = B1152000; break;

        case BaudRate::B_1500000: baud_rate = B1500000; break;

        case BaudRate::B_2000000: baud_rate = B2000000; break;

        case BaudRate::B_2500000: baud_rate = B2500000; break;

        case BaudRate::B_3000000: baud_rate = B3000000; break;

        case BaudRate::B_3500000: baud_rate = B3500000; break;

        case BaudRate::B_4000000: baud_rate = B4000000; break;
      }
      cfsetispeed(&tty, baud_rate);
      cfsetospeed(&tty, baud_rate);

      if (use_hardware_flow_control_) {
        tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
      } else {
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
      }

      switch (num_of_bits_per_byte_) {
        case NumOfBitsPerByte::FIVE: tty.c_cflag |= CS5; break;

        case NumOfBitsPerByte::SIX: tty.c_cflag |= CS6; break;

        case NumOfBitsPerByte::SEVEN: tty.c_cflag |= CS7; break;

        case NumOfBitsPerByte::EIGHT: tty.c_cflag |= CS8; break;
      }

      switch (parity_) {
        case Parity::NONE:
          tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
          break;

        case Parity::ODD:
          tty.c_cflag |= PARODD; // Use odd parity
          break;

        case Parity::EVEN:
          tty.c_cflag &= ~PARODD; // Use even parity
          break;
      }

      tty.c_cc[VMIN] = vmin_;
      tty.c_cc[VTIME] = vtime_;

      if (use_software_flow_control_) {
        tty.c_iflag |= (IXON | IXOFF | IXANY); // Turn on software flow control
      } else {
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
      }

      switch (stop_bits_) {
        case StopBits::ONE:
          tty.c_cflag &= ~CSTOPB; // Use only one stop bit used in communication (most common)
          break;

        case StopBits::TWO:
          tty.c_cflag |= CSTOPB; // Use two stop bits used in communication
          break;
      }

      tty.c_cflag |= CREAD;                                                        // Enable Receiver
      tty.c_lflag &= ~ICANON;                                                      // Use non-canonical mode, i.e. just read the data in raw
      tty.c_lflag &= ~ECHO;                                                        // Do not echo input characters
      tty.c_lflag &= ~ISIG;                                                        // Do not process signal character
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Turn off special handling of bytes on receive
      tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

      if (tcsetattr(device_desc_, TCSANOW, &tty) != 0) {
        return ConfigResult::TERMIOS_ERROR;
      }

      return ConfigResult::SUCCESS;
    }

    SerialDevice::SerialDevice() : p_impl_(std::make_unique<SerialDevice::SerialDeviceImpl>())
    {
    }

    SerialDevice::~SerialDevice()
    {
      disconnect();
    }

    bool SerialDevice::setBaudRate(BaudRate val)
    {
      p_impl_->baud_rate_ = val;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setHardwareFlowControl(bool use)
    {
      p_impl_->use_hardware_flow_control_ = use;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setNumOfBitsPerByte(NumOfBitsPerByte val)
    {
      p_impl_->num_of_bits_per_byte_ = val;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setParity(Parity val)
    {
      p_impl_->parity_ = val;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setReadConfig(uint8_t vmin, uint8_t vtime)
    {
      p_impl_->vmin_ = vmin;
      p_impl_->vtime_ = vtime;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setSoftwareFlowControl(bool use)
    {
      p_impl_->use_software_flow_control_ = use;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    bool SerialDevice::setStopBits(StopBits val)
    {
      p_impl_->stop_bits_ = val;
      ConfigResult res = p_impl_->configure();
      // DEVICE_IS_CLOSED is accepted because eventually when the connect() is called, the settings will be applied
      return (res == ConfigResult::SUCCESS || res == ConfigResult::DEVICE_IS_CLOSED) ? true : false;
    }

    std::string SerialDevice::getDevicePath()
    {
      return p_impl_->device_path_;
    }

    State SerialDevice::status()
    {
      return p_impl_->status_;
    }

    bool SerialDevice::connect(const std::string& device_path, const RWMode& rw_mode)
    {
      if (p_impl_->status_ == State::OPEN) {
        // If it is already connected, we disconnect it first. If it fails to disconnect, we return false.
        if (!this->disconnect()) {
          return false;
        }
        // After it is disconnected, we can continue the connecting process.
      }
      int mode = O_NOCTTY | O_NDELAY;
      switch (rw_mode) {
        case RWMode::READ_ONLY: mode |= O_RDONLY; break;

        case RWMode::WRITE_ONLY: mode |= O_WRONLY; break;

        case RWMode::BOTH: mode |= O_RDWR; break;
      }
      p_impl_->device_desc_ = open(device_path.c_str(), mode);
      if (p_impl_->device_desc_ < 0) {
        p_impl_->status_ = State::CLOSED;
        return false;
      }
      p_impl_->device_path_ = device_path;
      p_impl_->rw_mode_ = rw_mode;
      p_impl_->status_ = State::OPEN;
      ConfigResult res = p_impl_->configure();

      // We have to disconnect to close the file when the configuration is not successfully set
      if (res != ConfigResult::SUCCESS) {
        disconnect();
      }

      return (res == ConfigResult::SUCCESS) ? true : false;
    }

    bool SerialDevice::disconnect()
    {
      if (p_impl_->status_ == State::OPEN) {
        p_impl_->status_ = State::CLOSED;
        if (close(p_impl_->device_desc_) != 0) {
          return false;
        }
      }
      return true;
    }

    int SerialDevice::readData(uint8_t* read_buffer)
    {
      std::lock_guard<std::mutex> lock(p_impl_->fd_mutex_);
      // When a terminal device is disconnected, any read() operation will return 0.
      // Reference: https://pubs.opengroup.org/onlinepubs/009656599/toc.pdf, page 120, chapter 9.1.10. Modem Disconnect
      //
      // Because O_NDELAY is set, if read() is called and there is no data available, read() will return -1.
      // Reference: https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap11.html
      int read_status = read(p_impl_->device_desc_, read_buffer, sizeof(*read_buffer));

      if (p_impl_->status_ == State::CLOSED) {
        return -3;
      }
      if (p_impl_->rw_mode_ == RWMode::WRITE_ONLY) {
        return -2;
      }
      if (read_status == 0 || p_impl_->status_ == State::DISCONNECTED) {
        disconnect();
        p_impl_->status_ = State::DISCONNECTED;
        return -1;
      }
      if (read_status == -1) {
        return 0;
      }
      return read_status;
    }

    int SerialDevice::writeData(uint8_t* write_buffer, unsigned int length)
    {
      std::lock_guard<std::mutex> lock(p_impl_->fd_mutex_);
      if (p_impl_->status_ == State::CLOSED) {
        return -1;
      }
      if (p_impl_->rw_mode_ == RWMode::READ_ONLY) {
        return -2;
      }
      int bytes_written = static_cast<int>(write(p_impl_->device_desc_, write_buffer, length));
      tcdrain(p_impl_->device_desc_);
      return bytes_written;
    }

  } // namespace serial_comm
} // namespace bayukael