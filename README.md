# serial_device

A library for talking to serial devices on Linux, using POSIX termios under the hood. It wraps the common serial configuration knobs (baud rate, parity, stop bits, flow control, etc.) behind a simple object-oriented interface.

## What it does

`SerialDevice` lets you:

- **configure** the port before opening it: baud rate, number of bits per byte, parity, stop bits, hardware/software flow control, and the `VMIN`/`VTIME` read behaviour
- **connect** to a device path in read-only, write-only, or read/write mode
- **read** data into a caller-supplied buffer
- **write** data from a caller-supplied buffer
- **disconnect** and query the current state

The class uses the PIMPL idiom, so the public header has no platform-specific includes.

## Quick start (build)

Requires CMake >= 3.13, a C++17 compiler, and GoogleTest (`find_package(GTest CONFIG REQUIRED)`).

```bash
cmake -S . -B build -G Ninja
cmake --build build
./build/serial_device_test    # GoogleTest runner
```

Run a single test with a GTest filter:

```bash
./build/serial_device_test --gtest_filter='SerialDeviceTest.WriteThenReadTest'
```

There are no lint/format targets; building and running `serial_device_test` is the only verification.

> **Testing quirk:** the test suite opens a PTY pair at the hardcoded paths `/tmp/pty1` and `/tmp/pty2` (via `socat`). If the tests fail to open the port, make sure `/tmp/pty1` and `/tmp/pty2` exist and are free.

## Install

Install the library to a prefix. The version is appended to the install prefix, so files land under `<prefix>/<version>/`:

```bash
cmake -S . -B build -G Ninja
cmake --install build --prefix <prefix>
```

This installs with this layout:

```
<prefix>/1.0.0/
  include/serial_device/SerialDevice.h   # public header
  include/serial_device/Version.h        # version header
  lib/libpendarlab-serial_device.so.1.0.0
  lib/cmake/PendarlabSerialDevice        # CMake package config
```

## Using the library from another project

The library installs as the CMake package `PendarlabSerialDevice` (exported target `pendarlab::SerialDevice`). Point `CMAKE_PREFIX_PATH` at the versioned install prefix and request it from `find_package`:

```bash
cmake -S my_app -B build -DCMAKE_PREFIX_PATH=<prefix>/1.0.0
```

```cmake
find_package(PendarlabSerialDevice REQUIRED)
target_link_libraries(my_app PRIVATE pendarlab::SerialDevice)
```

Example usage:

```cpp
#include <serial_device/SerialDevice.h>

using namespace pendarlab::lib::comm::transport;

SerialDevice serial;

serial.setBaudRate(SerialDevice::BaudRate::B_115200);
serial.setNumOfBitsPerByte(SerialDevice::NumOfBitsPerByte::EIGHT);
serial.setParity(SerialDevice::Parity::NONE);
serial.setStopBits(SerialDevice::StopBits::ONE);

if (serial.connect("/dev/ttyUSB0", SerialDevice::RWMode::BOTH)) {
  uint8_t tx = 0x41;
  serial.writeData(&tx, sizeof(tx));

  uint8_t rx = 0;
  serial.readData(&rx, sizeof(rx));
}

serial.disconnect();
```

## Tests

`test/serial_device_test.cpp` uses GoogleTest. It connects two `SerialDevice` instances to a PTY pair and verifies that bytes written on one end are read back on the other (single-byte write/read and bulk write/read). `test/read_serial.cpp` is a small manual PTY helper and is not built by CMake.
