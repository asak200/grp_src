#include "hw_interface_ssmr/comm_class.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

SerialCommDriver::SerialCommDriver(const std::string& device_name) {
    device_ = device_name;}

SerialCommDriver::~SerialCommDriver() {
    if (serial_.IsOpen()) {
        serial_.Close();
    }
}

void SerialCommDriver::init() {
    serial_.Open(device_);
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Optional delay

    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    std::cout << "Serial port initialized: " << device_ << std::endl;
}

std::string SerialCommDriver::read(std::string order) {
    write(order);

    const int timeout_ms = 500;
    auto start_time = std::chrono::steady_clock::now();

    while (!serial_.IsDataAvailable()){
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        if (elapsed_ms > timeout_ms) {
            return "Timeout";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::string response;
    char c;

    while (serial_.IsDataAvailable()) {
        try {
            serial_.ReadByte(c, 100);  // 100 ms timeout
            if (c == '\n') break;
            response += c;
        } catch (const LibSerial::ReadTimeout&) {
            break;
        }
    }

    return response;
}

void SerialCommDriver::write(const std::string& msg) {
    if (serial_.IsOpen()) {
        serial_.Write(msg + "\n");
    } else {
        std::cerr << "Serial port not open. Can't write.\n";
    }
}
