// RMD ID Scan Node
// 2026.02.18 백종욱

#include <myactuator_rmd/myactuator_rmd.hpp>
#include <myactuator_rmd/can/exceptions.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <string>
#include <optional>
#include <array>
#include <stdexcept>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

constexpr float DEG2RAD = 3.141592653589793f / 180.0f;

static float dps2_to_radps2(std::int32_t dps2)
{
    return static_cast<float>(dps2) * DEG2RAD;
}

static void printErrorFlags(uint16_t error)
{
    if (error == 0)
    {
        std::cout << "  Status: OK\n";
        return;
    }

    std::cout << "  Error Code: 0x" << std::hex << std::setw(4) << std::setfill('0') << error << std::dec << "\n";
}

class RawCanSocket
{
public:
    explicit RawCanSocket(const std::string& ifname)
    {
        socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_ < 0)
            throw std::runtime_error("CAN socket fail");

        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

        if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("ioctl fail");

        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            throw std::runtime_error("bind fail");

        struct timeval tv {0, 100000};
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ~RawCanSocket()
    {
        if (socket_ >= 0) ::close(socket_);
    }

    std::optional<can_frame> request(uint32_t id, uint8_t cmd, uint8_t index)
    {
        uint32_t tx_id = 0x140 + id;
        uint32_t rx_id = 0x240 + id;

        can_frame tx {};
        tx.can_id = tx_id;
        tx.can_dlc = 8;
        tx.data[0] = cmd;
        tx.data[1] = index;

        if (::write(socket_, &tx, sizeof(tx)) != sizeof(tx))
            return std::nullopt;

        while (true)
        {
            can_frame rx {};
            if (::read(socket_, &rx, sizeof(rx)) < 0)
                return std::nullopt;

            if ((rx.can_id & 0x7FF) != rx_id)
                continue;

            if (rx.data[0] != cmd || rx.data[1] != index)
                continue;

            return rx;
        }
    }

private:
    int socket_;
};

static float parseFloatLE(const can_frame& f)
{
    uint32_t raw =
        f.data[4] |
        (f.data[5] << 8) |
        (f.data[6] << 16) |
        (f.data[7] << 24);

    float val;
    std::memcpy(&val, &raw, sizeof(val));
    return val;
}

static std::int32_t parseInt32LE(const can_frame& f)
{
    return
        f.data[4] |
        (f.data[5] << 8) |
        (f.data[6] << 16) |
        (f.data[7] << 24);
}

static void printPID(RawCanSocket& can, int id)
{
    struct {uint8_t idx; const char* name;} items[] = {
        {0x01,"Current KP"},
        {0x02,"Current KI"},
        {0x04,"Speed KP"},
        {0x05,"Speed KI"},
        {0x07,"Position KP"},
        {0x08,"Position KI"},
        {0x09,"Position KD"}
    };

    std::cout << "\n  [0x30] PID\n";

    for (auto& it : items)
    {
        auto r = can.request(id, 0x30, it.idx);
        if (!r)
            std::cout << "    " << it.name << " : fail\n";
        else
            std::cout << "    " << it.name << " : " << parseFloatLE(*r) << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

static void printAccel(RawCanSocket& can, int id)
{
    struct {uint8_t idx; const char* name;} items[] = {
        {0x00,"Position Accel"},
        {0x01,"Position Decel"},
        {0x02,"Speed Accel"},
        {0x03,"Speed Decel"}
    };

    std::cout << "\n  [0x42] Acceleration (SI)\n";

    for (auto& it : items)
    {
        auto r = can.request(id, 0x42, it.idx);
        if (!r)
        {
            std::cout << "    " << it.name << " : fail\n";
        }
        else
        {
            float rad = dps2_to_radps2(parseInt32LE(*r));
            std::cout << "    " << it.name << " : " << std::fixed << std::setprecision(3) << rad << " rad/s^2\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main()
{
    auto driver = std::make_unique<myactuator_rmd::CanDriver>("can0");
    RawCanSocket raw("can0");

    std::cout << "Scanning...\n";

    for (int id=1; id<=32; ++id)
    {
        try
        {
            myactuator_rmd::ActuatorInterface m(*driver, id);
            auto s = m.getMotorStatus1();

            std::cout << "\nID: " << id << "\n";
            std::cout << "Temp: " << s.temperature << " °C\n";
            std::cout << "Volt: " << s.voltage << " V\n";

            printErrorFlags((uint16_t)s.error_code);

            printPID(raw,id);
            printAccel(raw,id);

            break;
        }
        catch (...) {}
    }

    std::cout << "\nDone\n";
}