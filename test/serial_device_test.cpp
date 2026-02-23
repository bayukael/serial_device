#include <chrono>
#include <gtest/gtest.h>
#include <serial_device/SerialDevice.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>

const std::string g_path_pty_1("/tmp/pty1");
const std::string g_path_pty_2("/tmp/pty2");
using namespace pendarlab::lib::comm::transport;

namespace
{
  class SerialDeviceTest : public testing::Test
  {
  protected:
    SerialDeviceTest()
    {
      SerialDevice::BaudRate baud_rate = SerialDevice::BaudRate::B_115200;

      pty1.setBaudRate(baud_rate);
      pty1.setReadConfig(0, 10);
      pty1.connect(g_path_pty_1, SerialDevice::RWMode::BOTH);

      pty2.setBaudRate(baud_rate);
      pty2.setReadConfig(0, 10);
      pty2.connect(g_path_pty_2, SerialDevice::RWMode::BOTH);
    }

    SerialDevice pty1;
    SerialDevice pty2;
  };

  TEST_F(SerialDeviceTest, WriteThenReadTest)
  {
    for (size_t i = 0; i < 256; i++) {
      uint8_t byte_to_send = i;pty1.writeData(&byte_to_send, sizeof(byte_to_send));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      uint8_t a_buffer;
      pty2.readData(&a_buffer);

      EXPECT_EQ(byte_to_send, a_buffer);
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  }
} // namespace

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  pid_t pid = fork();
  if (pid < 0) {
    perror("fork failed");
    return 1;
  } else if (pid == 0) {
    std::string writer_arg = "pty,raw,echo=0,link=" + g_path_pty_1;
    std::string reader_arg = "pty,raw,echo=0,link=" + g_path_pty_2;
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    execlp("socat", "socat", writer_arg.c_str(), reader_arg.c_str(), (char*)nullptr);
    perror("execlp failed");
    return 1;
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int test_result = RUN_ALL_TESTS();

    kill(pid, SIGTERM);
    waitpid(pid, nullptr, 0);

    return test_result; // <-- Google Test return code
  }
}