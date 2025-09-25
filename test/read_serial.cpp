#include <chrono>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <iostream>
#include <mutex>
#include <serial_device/SerialDevice.h>
#include <string>
#include <termios.h> // Contains POSIX terminal control definitions
#include <thread>
#include <unistd.h> // write(), read(), close()

void reader_routine()
{
  std::string reader_port("/dev/pts/16");
  bayukael::serial_comm::SerialDevice reader;
  reader.setBaudRate(bayukael::serial_comm::BaudRate::B_2000000);
  reader.setReadConfig(0, 100);
  reader.connect(reader_port, bayukael::serial_comm::RWMode::BOTH);

  if (reader.status() == bayukael::serial_comm::State::OPEN) {
    std::cout << "READER_THREAD: Reader is connected to " << reader_port << std::endl;
  }

  while (reader.status() == bayukael::serial_comm::State::OPEN) {
    uint8_t a_buffer;
    int bytes_read = 0;
    bytes_read = reader.readData(&a_buffer);
    if (bytes_read > 0) {
      std::cout << "READER_THREAD: Bytes read: " << bytes_read << " --- " << (int)a_buffer << std::endl;
      if (a_buffer == 255) {
        break;
      }
    } else {
      std::cout << "READER_THREAD: Bytes read: " << bytes_read << " --- " << (int)a_buffer << std::endl;
      perror("READER_THREAD: hm...:");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::cout << "READER_THREAD: Done " << std::endl;
}

void writer_routine()
{
  std::string writer_port("/dev/pts/15");
  bayukael::serial_comm::SerialDevice writer;
  writer.setBaudRate(bayukael::serial_comm::BaudRate::B_2000000);
  writer.setReadConfig(0, 10);
  writer.connect(writer_port, bayukael::serial_comm::RWMode::BOTH);

  if (writer.status() == bayukael::serial_comm::State::OPEN) {
    std::cout << "WRITER_THREAD: Writer is connected to " << writer_port << std::endl;
    std::cout << "WRITER_THREAD: Sending data every 250 milliseconds" << std::endl;

    for (size_t i = 0; i < 256; i++) {
      uint8_t byte_to_send = i;
      int bytes_written = writer.writeData(&byte_to_send, sizeof(byte_to_send));
      if (bytes_written > 0) {
        std::cout << "WRITER_THREAD: byte value sent -> " << (int)byte_to_send << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  std::cout << "WRITER_THREAD: Done" << std::endl;
}

int thread_test(int argc, char* argv[])
{
  std::thread writer_thread(&writer_routine);
  std::thread reader_thread(&reader_routine);

  writer_thread.join();
  reader_thread.join();
  return 0;
}

int write_and_read_test(int argc, char* argv[])
{
  std::string writer_port;
  std::string reader_port;
  if (argc > 1) {
    // Loop through each argument (starting from the first argument after the program name)
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      // Check if the argument matches "-w"
      if (arg == "-w") {
        // Check if there is another argument after "-w"
        if (i + 1 < argc) {
          writer_port = argv[i + 1];
          std::cout << "Assign writer_port to: " << writer_port << std::endl;
          i++; // Skip the next argument since it's the value for "-w"
        } else {
          std::cerr << "Error: -w option requires a value" << std::endl;
          return 1;
        }
      }
      // Check if the argument matches "-r"
      else if (arg == "-r") {
        if (i + 1 < argc) {
          reader_port = argv[i + 1];
          std::cout << "Assign reader_port to: " << reader_port << std::endl;
          ++i; // Skip the next argument since it's the value for "-r"
        } else {
          std::cerr << "Error: -r option requires a value" << std::endl;
          return 1;
        }
      }
      // Handle the case where the argument is unknown
      else {
        std::cerr << "Unknown option: " << arg << std::endl;
        return 1;
      }
    }
  }

  bayukael::serial_comm::BaudRate baud_rate = bayukael::serial_comm::BaudRate::B_2000000;

  bayukael::serial_comm::SerialDevice writer;
  writer.setBaudRate(baud_rate);
  writer.setReadConfig(0, 10);
  bool writer_connect = writer.connect(writer_port, bayukael::serial_comm::RWMode::BOTH);
  std::cout << "writer_connect: " << writer_connect << std::endl;
  if (writer.status() == bayukael::serial_comm::State::OPEN) {
    std::cout << "___WRITER___: connected to " << writer_port << std::endl;
    std::cout << "___WRITER___: Sending data every 250 milliseconds" << std::endl;
  } else {
    std::cout << "___WRITER___: failed to connect to " << writer_port << std::endl;
  }

  bayukael::serial_comm::SerialDevice reader;
  reader.setBaudRate(baud_rate);
  reader.setReadConfig(0, 10);
  bool reader_connect = reader.connect(reader_port, bayukael::serial_comm::RWMode::BOTH);
  std::cout << "reader_connect: " << reader_connect << std::endl;
  if (reader.status() == bayukael::serial_comm::State::OPEN) {
    std::cout << "##_READER_##: connected to " << reader_port << std::endl;
  } else {
    std::cout << "##_READER_##: failed to connect to " << reader_port << std::endl;
  }

  // This is for filling the port buffer
  {
    for (size_t i = 0; i < 51; i++) {
      uint8_t byte_to_send = i;
      int bytes_written = writer.writeData(&byte_to_send, sizeof(byte_to_send));
      if (bytes_written > 0) {
        std::cout << "___WRITER___: byte value sent -> " << (int)byte_to_send << std::endl;
      }
    }
  }

  // This is for emptying buffered port.
  {
    uint8_t a_buffer;
    int bytes_read = 0;
    bytes_read = reader.readData(&a_buffer);
    std::cout << "Bytes read pre test: " << bytes_read << std::endl;
    while (bytes_read > 0) {
      bytes_read = reader.readData(&a_buffer);
    }
  }

  for (size_t i = 0; i < 256; i++) {
    uint8_t byte_to_send = i;
    int bytes_written = writer.writeData(&byte_to_send, sizeof(byte_to_send));
    if (bytes_written > 0) {
      std::cout << "___WRITER___: byte value sent -> " << (int)byte_to_send << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t a_buffer;
    int bytes_read = 0;
    bytes_read = reader.readData(&a_buffer);
    if (bytes_read > 0) {
      std::cout << "##_READER_##: Bytes read: " << bytes_read << " --- " << (int)a_buffer << std::endl;
    } else {
      std::cout << "##_READER_##: Bytes read: " << bytes_read << " --- " << (int)a_buffer << std::endl;
      perror("##_READER_##: hm...:");
    }

    if (byte_to_send == a_buffer) {
      std::cout << "Data is transmitted and received successfully!" << std::endl;
    } else {
      std::cout << "Data is not received!" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  return 0;
}

int file_type_test(int argc, char* argv[])
{
  std::string writer_port;
  std::string reader_port;
  if (argc > 1) {
    // Loop through each argument (starting from the first argument after the program name)
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      // Check if the argument matches "-w"
      if (arg == "-w") {
        // Check if there is another argument after "-w"
        if (i + 1 < argc) {
          writer_port = argv[i + 1];
          std::cout << "Assign writer_port to: " << writer_port << std::endl;
          i++; // Skip the next argument since it's the value for "-w"
        } else {
          std::cerr << "Error: -w option requires a value" << std::endl;
          return 1;
        }
      }
      // Check if the argument matches "-r"
      else if (arg == "-r") {
        if (i + 1 < argc) {
          reader_port = argv[i + 1];
          std::cout << "Assign reader_port to: " << reader_port << std::endl;
          ++i; // Skip the next argument since it's the value for "-r"
        } else {
          std::cerr << "Error: -r option requires a value" << std::endl;
          return 1;
        }
      }
      // Handle the case where the argument is unknown
      else {
        std::cerr << "Unknown option: " << arg << std::endl;
        return 1;
      }
    }
  }

  bayukael::serial_comm::SerialDevice writer;
  // writer.setBaudRate(baud_rate);
  // writer.setReadConfig(0, 10);
  writer.connect(writer_port, bayukael::serial_comm::RWMode::BOTH);
  if (writer.status() == bayukael::serial_comm::State::OPEN) {
    std::cout << "___WRITER___: connected to " << writer_port << std::endl;
  } else {
    std::cout << "___WRITER___: failed to connect to " << writer_port << std::endl;
  }
  return 0;
}

void just_for_open(int argc, char* argv[])
{
  if (argc <= 1) {
    return;
  }

  std::string device_path = argv[1];

  int mode = O_NDELAY | O_RDWR;
  int fd = open(device_path.c_str(), mode);
  std::cout << "fd: " << fd << std::endl;
  termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cout << "Cannot get termios" << std::endl;
  }
}

void open_and_read(int argc, char* argv[])
{
  if (argc <= 1) {
    return;
  }

  std::string device_path = argv[1];

  int mode = O_NOCTTY | O_NDELAY | O_RDWR;
  int fd = open(device_path.c_str(), mode);
  std::cout << "fd: " << fd << std::endl;

  uint8_t a_buffer;
  int bytes_read = read(fd, &a_buffer, sizeof(a_buffer));
  std::cout << "bytes_read: " << bytes_read << std::endl;
}

void open_and_write(int argc, char* argv[])
{
  if (argc <= 1) {
    return;
  }

  std::string writer_path = argv[1];

  int mode = O_NOCTTY | O_NDELAY | O_RDWR;
  int fd = open(writer_path.c_str(), mode);
  std::cout << "fd: " << fd << std::endl;

  char write_buffer[] = "qwertyuiop";
  int bytes_written = static_cast<int>(write(fd, write_buffer, sizeof(write_buffer)));
  std::cout << "bytes_written: " << bytes_written << std::endl;
  tcdrain(fd);
}

int main(int argc, char* argv[])
{
  // thread_test(argc,argv);
  write_and_read_test(argc, argv);
  // file_type_test(argc, argv);
  // just_for_open(argc, argv);
  // open_and_read(argc, argv);
  // open_and_write(argc, argv);

  return 0;
}