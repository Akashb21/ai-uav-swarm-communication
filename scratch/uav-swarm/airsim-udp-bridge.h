#ifndef AIRSIM_UDP_BRIDGE_H
#define AIRSIM_UDP_BRIDGE_H

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

struct AirSimUavState
{
    bool valid{false};
    uint32_t uavId{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double timestamp{0.0};
    double receivedAtSeconds{0.0};
};

/** A non-blocking UDP receiver for one JSON AirSim state datagram per UAV. */
class AirSimUdpBridge
{
  public:
    AirSimUdpBridge(uint32_t numUavs, const std::string& bindAddress, uint16_t port);
    ~AirSimUdpBridge();

    bool Start(std::string& error);
    void Stop();
    uint32_t Poll(double simulationTimeSeconds);
    const AirSimUavState* GetLatestState(uint32_t uavId) const;
    bool HasFreshState(uint32_t uavId, double simulationTimeSeconds, double timeoutSeconds) const;
    bool HasInitialStates() const;
    std::vector<uint32_t> GetMissingInitialUavIds() const;

  private:
    bool ParseDatagram(const std::string& text, AirSimUavState& state, std::string& error) const;

    int m_socket{-1};
    std::string m_bindAddress;
    uint16_t m_port;
    std::vector<AirSimUavState> m_states;
};

} // namespace ns3

#endif // AIRSIM_UDP_BRIDGE_H
