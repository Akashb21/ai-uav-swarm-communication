#ifndef AIRSIM_UDP_BRIDGE_H
#define AIRSIM_UDP_BRIDGE_H

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

struct AirSimUavState
{
    uint32_t id;

    double timestamp;

    double x;
    double y;
    double z;

    double vx;
    double vy;
    double vz;

    bool valid;

    AirSimUavState()
        : id(0),
          timestamp(0.0),
          x(0.0),
          y(0.0),
          z(0.0),
          vx(0.0),
          vy(0.0),
          vz(0.0),
          valid(false)
    {
    }
};


class AirSimUdpBridge
{
public:

    AirSimUdpBridge(
        uint32_t uavCount,
        const std::string& bindAddress,
        uint32_t port);

    ~AirSimUdpBridge();


    bool Start(std::string& error);

    void Stop();

    void Poll(double now);


    bool HasInitialStates() const;


    const AirSimUavState*
    GetLatestState(uint32_t id) const;


    bool HasFreshState(
        uint32_t id,
        double now,
        double staleTimeout) const;


    std::vector<uint32_t>
    GetMissingInitialUavIds() const;


private:

    bool ProcessTelemetry(
        const std::string& message,
        double receiveTime);


    bool ExtractNumber(
        const std::string& message,
        const std::string& key,
        double& value) const;


    bool ExtractUavId(
        const std::string& message,
        uint32_t& id) const;


    bool ParseDroneId(
        const std::string& name,
        uint32_t& id) const;


private:

    uint32_t m_uavCount;

    std::string m_bindAddress;

    uint32_t m_port;

    int m_socketFd;

    bool m_running;


    std::vector<AirSimUavState> m_states;

    std::vector<bool> m_received;

    std::vector<double> m_lastReceiveTime;
};

} // namespace ns3

#endif // AIRSIM_UDP_BRIDGE_H