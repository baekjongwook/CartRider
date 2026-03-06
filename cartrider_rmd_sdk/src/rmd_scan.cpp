// RMD ID Scan Node
// 2026.02.18 백종욱

#include <myactuator_rmd/myactuator_rmd.hpp>
#include <myactuator_rmd/can/exceptions.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main()
{
    auto driver = std::make_unique<myactuator_rmd::CanDriver>("can0");

    std::cout << "Scanning RMD IDs...\n";

    for (int id = 1; id <= 32; ++id)
    {
        try
        {
            myactuator_rmd::ActuatorInterface motor(*driver, id);

            auto status = motor.getMotorStatus1();

            std::cout << "\nFound motor ID: " << id << "\n";
            std::cout << "  Temp: " << status.temperature << " °C\n";
            std::cout << "  Voltage: " << status.voltage << " V\n";

            uint16_t error = static_cast<uint16_t>(status.error_code);

            if (error == 0)
            {
                std::cout << "  Status: OK\n";
            }
            else
            {
                std::cout << "  Error Code: 0x"
                          << std::hex << std::setw(4)
                          << std::setfill('0')
                          << error << std::dec << "\n";

                if (error & 0x0002) std::cout << "   - Motor Stall\n";
                if (error & 0x0004) std::cout << "   - Low Voltage\n";
                if (error & 0x0008) std::cout << "   - Over Voltage\n";
                if (error & 0x0010) std::cout << "   - Over Current\n";
                if (error & 0x0040) std::cout << "   - Power Overrun\n";
                if (error & 0x0080) std::cout << "   - Calibration Write Error\n";
                if (error & 0x0100) std::cout << "   - Overspeed\n";
                if (error & 0x0800) std::cout << "   - Component Overtemperature\n";
                if (error & 0x1000) std::cout << "   - Motor Overtemperature\n";
                if (error & 0x2000) std::cout << "   - Encoder Calibration Error\n";
                if (error & 0x4000) std::cout << "   - Encoder Data Error\n";
            }
        }
        catch (...)
        {
            std::cout << "\nMotor is not founded.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\nScan complete.\n";
    return 0;
}