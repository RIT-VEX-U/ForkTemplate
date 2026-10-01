#pragma once

#include <cstdint>
#include <cstddef>

namespace mantle {

/**
 * Hardware Abstraction Layer interface for serial communication ports.
 */
class SerialPort {
public:
    virtual ~SerialPort() = default;
    virtual void write(const uint8_t *data, size_t length) = 0;
    virtual int read_char() = 0;
    virtual int available() = 0;
};

class MockSerialPort : public SerialPort {
public:
    void write(const uint8_t *, size_t) override {}
    int read_char() override { return -1; }
    int available() override { return 0; }
};

} // namespace mantle
