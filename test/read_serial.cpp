#include <iostream>
#include <serial_device/SerialDevice.h>
#include <string>

int main(int argc, char* argv[])
{
  bayukael::serial_comm::SerialDevice test_device;
  test_device.setBaudRate(bayukael::serial_comm::BaudRate::B_2000000);
  test_device.setReadConfig(0, 10);
  test_device.connect("/dev/ttyACM0", bayukael::serial_comm::RWMode::BOTH);

  while (test_device.status() == bayukael::serial_comm::State::OPEN) {
    // unsigned char read_buffer[280];
    // int bytes_read = 0;
    // bytes_read = test_device.readData(read_buffer);

    uint8_t a_buffer;
    int bytes_read = 0;
    bytes_read = test_device.readData(&a_buffer);
    if (bytes_read > 0) {
      std::cout << "Bytes read: " << bytes_read << " --- " << (int) a_buffer << std::endl;
    }
  }

  return 0;
}