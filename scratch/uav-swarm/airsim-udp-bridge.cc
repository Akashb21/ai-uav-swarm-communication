#include "airsim-udp-bridge.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <regex>
#include <sys/socket.h>
#include <unistd.h>

namespace ns3
{

AirSimUdpBridge::AirSimUdpBridge(uint32_t numUavs, const std::string& bindAddress, uint16_t port)
    : m_bindAddress(bindAddress),
      m_port(port),
      m_states(numUavs)
{
}

AirSimUdpBridge::~AirSimUdpBridge()
{
    Stop();
}

bool
AirSimUdpBridge::Start(std::string& error)
{
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_socket < 0)
    {
        error = "socket(): " + std::string(std::strerror(errno));
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(m_port);
    if (inet_pton(AF_INET, m_bindAddress.c_str(), &address.sin_addr) != 1)
    {
        error = "bind address must be an IPv4 address: " + m_bindAddress;
        Stop();
        return false;
    }
    if (bind(m_socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        error = "bind(" + m_bindAddress + ":" + std::to_string(m_port) + "): " +
                std::strerror(errno);
        Stop();
        return false;
    }
    int flags = fcntl(m_socket, F_GETFL, 0);
    if (flags < 0 || fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        error = "fcntl(O_NONBLOCK): " + std::string(std::strerror(errno));
        Stop();
        return false;
    }
    return true;
}

void
AirSimUdpBridge::Stop()
{
    if (m_socket >= 0)
    {
        close(m_socket);
        m_socket = -1;
    }
}

bool
AirSimUdpBridge::ParseDatagram(const std::string& text, AirSimUavState& state, std::string& error) const
{
    auto field = [&text, &error](const std::string& name, double& value) {
        const std::regex pattern("\\\"" + name + "\\\"\\s*:\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
        std::smatch match;
        if (!std::regex_search(text, match, pattern))
        {
            error = "missing or invalid JSON numeric field '" + name + "'";
            return false;
        }
        try
        {
            value = std::stod(match[1].str());
        }
        catch (const std::exception&)
        {
            error = "invalid numeric field '" + name + "'";
            return false;
        }
        return true;
    };

    double id = 0.0;
    if (!field("uav_id", id) || id < 0.0 || id != static_cast<uint32_t>(id) ||
        static_cast<uint32_t>(id) >= m_states.size())
    {
        if (error.empty())
        {
            error = "uav_id is outside the configured range";
        }
        return false;
    }
    state.uavId = static_cast<uint32_t>(id);
    return field("x", state.x) && field("y", state.y) && field("z", state.z) &&
           field("vx", state.vx) && field("vy", state.vy) && field("vz", state.vz) &&
           field("timestamp", state.timestamp);
}

uint32_t
AirSimUdpBridge::Poll(double simulationTimeSeconds)
{
    uint32_t received = 0;
    char buffer[2048];
    while (m_socket >= 0)
    {
        ssize_t count = recvfrom(m_socket, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        if (count < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                // The next scheduled poll may recover from a transient socket error.
            }
            break;
        }
        buffer[count] = '\0';
        AirSimUavState state;
        std::string error;
        if (!ParseDatagram(buffer, state, error))
        {
            continue;
        }
        state.valid = true;
        state.receivedAtSeconds = simulationTimeSeconds;
        m_states[state.uavId] = state;
        ++received;
    }
    return received;
}

const AirSimUavState*
AirSimUdpBridge::GetLatestState(uint32_t uavId) const
{
    return uavId < m_states.size() && m_states[uavId].valid ? &m_states[uavId] : nullptr;
}

bool
AirSimUdpBridge::HasFreshState(uint32_t uavId,
                                double simulationTimeSeconds,
                                double timeoutSeconds) const
{
    const AirSimUavState* state = GetLatestState(uavId);
    return state != nullptr && simulationTimeSeconds - state->receivedAtSeconds <= timeoutSeconds;
}

bool
AirSimUdpBridge::HasInitialStates() const
{
    for (const auto& state : m_states)
    {
        if (!state.valid)
        {
            return false;
        }
    }
    return true;
}

std::vector<uint32_t>
AirSimUdpBridge::GetMissingInitialUavIds() const
{
    std::vector<uint32_t> missing;
    for (uint32_t i = 0; i < m_states.size(); ++i)
    {
        if (!m_states[i].valid)
        {
            missing.push_back(i);
        }
    }
    return missing;
}

} // namespace ns3
