#ifndef SERIAL_COMM_CLASS_HPP
#define SERIAL_COMM_CLASS_HPP

#define BAUDRATE 115200

#include <string>
#include <libserial/SerialPort.h>

class SerialCommDriver {
public:
    SerialCommDriver(const std::string& device_name);
    ~SerialCommDriver();

    void init();
    std::string read(std::string order);
    void write(const std::string& msg);

private:
    std::string device_;
    LibSerial::SerialPort serial_;
};

#endif  // SERIAL_COMM_CLASS_HPP