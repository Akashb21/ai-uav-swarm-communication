#include "airsim-udp-bridge.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <sstream>

namespace ns3
{

// ============================================================
// Constructor
// ============================================================

AirSimUdpBridge::AirSimUdpBridge(
    uint32_t uavCount,
    const std::string& bindAddress,
    uint32_t port
)
    : m_uavCount(uavCount),
      m_bindAddress(bindAddress),
      m_port(port),
      m_socketFd(-1),
      m_running(false),
      m_states(uavCount),
      m_received(uavCount, false),
      m_lastReceiveTime(uavCount, 0.0)
{
    for (uint32_t i = 0; i < m_uavCount; ++i)
    {
        m_states[i].id = i;
    }
}


// ============================================================
// Destructor
// ============================================================

AirSimUdpBridge::~AirSimUdpBridge()
{
    Stop();
}


// ============================================================
// Start UDP receiver
// ============================================================

bool
AirSimUdpBridge::Start(
    std::string& error
)
{
    if (m_running)
    {
        return true;
    }

    // --------------------------------------------------------
    // Create UDP socket
    // --------------------------------------------------------

    m_socketFd = socket(
        AF_INET,
        SOCK_DGRAM,
        0
    );

    if (m_socketFd < 0)
    {
        error =
            "Failed to create UDP socket: "
            + std::string(std::strerror(errno));

        return false;
    }


    // --------------------------------------------------------
    // Allow address reuse
    // --------------------------------------------------------

    int reuse = 1;

    if (setsockopt(
            m_socketFd,
            SOL_SOCKET,
            SO_REUSEADDR,
            &reuse,
            sizeof(reuse)) < 0)
    {
        error =
            "Failed to set SO_REUSEADDR: "
            + std::string(std::strerror(errno));

        close(m_socketFd);
        m_socketFd = -1;

        return false;
    }


    // --------------------------------------------------------
    // Make socket non-blocking
    // --------------------------------------------------------

    int flags =
        fcntl(
            m_socketFd,
            F_GETFL,
            0
        );

    if (flags < 0)
    {
        error =
            "Failed to get socket flags: "
            + std::string(std::strerror(errno));

        close(m_socketFd);
        m_socketFd = -1;

        return false;
    }

    if (fcntl(
            m_socketFd,
            F_SETFL,
            flags | O_NONBLOCK) < 0)
    {
        error =
            "Failed to set non-blocking socket: "
            + std::string(std::strerror(errno));

        close(m_socketFd);
        m_socketFd = -1;

        return false;
    }


    // --------------------------------------------------------
    // Configure bind address
    // --------------------------------------------------------

    sockaddr_in serverAddress;

    std::memset(
        &serverAddress,
        0,
        sizeof(serverAddress)
    );

    serverAddress.sin_family =
        AF_INET;

    serverAddress.sin_port =
        htons(
            static_cast<uint16_t>(m_port)
        );


    // --------------------------------------------------------
    // Bind address
    // --------------------------------------------------------

    if (m_bindAddress.empty() ||
        m_bindAddress == "0.0.0.0")
    {
        serverAddress.sin_addr.s_addr =
            htonl(INADDR_ANY);
    }
    else
    {
        int result =
            inet_pton(
                AF_INET,
                m_bindAddress.c_str(),
                &serverAddress.sin_addr
            );

        if (result <= 0)
        {
            error =
                "Invalid bind address: "
                + m_bindAddress;

            close(m_socketFd);
            m_socketFd = -1;

            return false;
        }
    }


    // --------------------------------------------------------
    // Bind socket
    // --------------------------------------------------------

    if (bind(
            m_socketFd,
            reinterpret_cast<sockaddr*>(
                &serverAddress
            ),
            sizeof(serverAddress)) < 0)
    {
        error =
            "Failed to bind UDP socket to "
            + m_bindAddress
            + ":"
            + std::to_string(m_port)
            + " : "
            + std::string(std::strerror(errno));

        close(m_socketFd);
        m_socketFd = -1;

        return false;
    }


    m_running = true;


    std::cout
        << "[AirSimBridge] Started"
        << std::endl;

    std::cout
        << "[AirSimBridge] Bind address : "
        << m_bindAddress
        << std::endl;

    std::cout
        << "[AirSimBridge] UDP port    : "
        << m_port
        << std::endl;

    std::cout
        << "[AirSimBridge] UAV count    : "
        << m_uavCount
        << std::endl;


    return true;
}


// ============================================================
// Stop UDP receiver
// ============================================================

void
AirSimUdpBridge::Stop()
{
    if (m_socketFd >= 0)
    {
        close(m_socketFd);

        m_socketFd = -1;
    }

    m_running = false;
}


// ============================================================
// Poll socket
// ============================================================

void
AirSimUdpBridge::Poll(
    double now
)
{
    if (!m_running ||
        m_socketFd < 0)
    {
        return;
    }


    char buffer[8192];


    while (true)
    {
        sockaddr_in senderAddress;

        socklen_t senderLength =
            sizeof(senderAddress);


        ssize_t receivedBytes =
            recvfrom(
                m_socketFd,
                buffer,
                sizeof(buffer) - 1,
                0,
                reinterpret_cast<sockaddr*>(
                    &senderAddress
                ),
                &senderLength
            );


        // ----------------------------------------------------
        // No more packets available
        // ----------------------------------------------------

        if (receivedBytes < 0)
        {
            if (errno == EAGAIN ||
                errno == EWOULDBLOCK)
            {
                break;
            }

            if (errno == EINTR)
            {
                continue;
            }

            std::cerr
                << "[AirSimBridge] recvfrom() error: "
                << std::strerror(errno)
                << std::endl;

            break;
        }


        // ----------------------------------------------------
        // Ignore empty packet
        // ----------------------------------------------------

        if (receivedBytes == 0)
        {
            continue;
        }


        buffer[receivedBytes] = '\0';


        std::string message(
            buffer,
            static_cast<size_t>(receivedBytes)
        );


        // ----------------------------------------------------
        // Process telemetry
        // ----------------------------------------------------

        ProcessTelemetry(
            message,
            now
        );
    }
}


// ============================================================
// Process one telemetry message
// ============================================================

bool
AirSimUdpBridge::ProcessTelemetry(
    const std::string& message,
    double receiveTime
)
{
    uint32_t uavId = 0;


    // --------------------------------------------------------
    // UAV ID
    // --------------------------------------------------------

    if (!ExtractUavId(
            message,
            uavId))
    {
        std::cerr
            << "[AirSimBridge] Could not extract uav_id"
            << std::endl;

        return false;
    }


    if (uavId >= m_uavCount)
    {
        std::cerr
            << "[AirSimBridge] Invalid UAV ID: "
            << uavId
            << std::endl;

        return false;
    }


    // --------------------------------------------------------
    // Create temporary state
    // --------------------------------------------------------

    AirSimUavState state;

    state.id = uavId;


    // --------------------------------------------------------
    // Timestamp
    // --------------------------------------------------------

    double timestamp = 0.0;

    if (!ExtractNumber(
            message,
            "timestamp",
            timestamp))
    {
        timestamp = receiveTime;
    }

    state.timestamp = timestamp;


    // --------------------------------------------------------
    // Position
    // --------------------------------------------------------

    if (!ExtractNumber(
            message,
            "x",
            state.x))
    {
        return false;
    }

    if (!ExtractNumber(
            message,
            "y",
            state.y))
    {
        return false;
    }

    if (!ExtractNumber(
            message,
            "z",
            state.z))
    {
        return false;
    }


    // --------------------------------------------------------
    // Velocity
    // --------------------------------------------------------

    if (!ExtractNumber(
            message,
            "vx",
            state.vx))
    {
        return false;
    }

    if (!ExtractNumber(
            message,
            "vy",
            state.vy))
    {
        return false;
    }

    if (!ExtractNumber(
            message,
            "vz",
            state.vz))
    {
        return false;
    }


    state.valid = true;


    // --------------------------------------------------------
    // Store latest state
    // --------------------------------------------------------

    m_states[uavId] =
        state;

    m_received[uavId] =
        true;

    m_lastReceiveTime[uavId] =
        receiveTime;


    // --------------------------------------------------------
    // Debug output
    // --------------------------------------------------------

    std::cout
        << "[AirSimBridge] UAV "
        << uavId
        << " | "
        << "Pos=("
        << state.x
        << ", "
        << state.y
        << ", "
        << state.z
        << ") "
        << "Vel=("
        << state.vx
        << ", "
        << state.vy
        << ", "
        << state.vz
        << ")"
        << std::endl;


    return true;
}


// ============================================================
// Extract UAV ID from JSON
// ============================================================

bool
AirSimUdpBridge::ExtractUavId(
    const std::string& message,
    uint32_t& id
) const
{
    const std::string key =
        "\"uav_id\"";


    size_t keyPosition =
        message.find(key);


    if (keyPosition == std::string::npos)
    {
        return false;
    }


    size_t colon =
        message.find(
            ':',
            keyPosition + key.length()
        );


    if (colon == std::string::npos)
    {
        return false;
    }


    size_t quoteStart =
        message.find(
            '"',
            colon + 1
        );


    if (quoteStart == std::string::npos)
    {
        return false;
    }


    size_t quoteEnd =
        message.find(
            '"',
            quoteStart + 1
        );


    if (quoteEnd == std::string::npos)
    {
        return false;
    }


    std::string droneName =
        message.substr(
            quoteStart + 1,
            quoteEnd - quoteStart - 1
        );


    return ParseDroneId(
        droneName,
        id
    );
}


// ============================================================
// Parse Drone1 ... Drone6
// ============================================================

bool
AirSimUdpBridge::ParseDroneId(
    const std::string& name,
    uint32_t& id
) const
{
    const std::string prefix =
        "Drone";


    if (name.size() <= prefix.size())
    {
        return false;
    }


    if (name.compare(
            0,
            prefix.size(),
            prefix) != 0)
    {
        return false;
    }


    std::string number =
        name.substr(
            prefix.size()
        );


    char* endPointer =
        nullptr;


    long droneNumber =
        std::strtol(
            number.c_str(),
            &endPointer,
            10
        );


    if (endPointer == number.c_str() ||
        *endPointer != '\0')
    {
        return false;
    }


    if (droneNumber < 1)
    {
        return false;
    }


    uint32_t convertedId =
        static_cast<uint32_t>(
            droneNumber - 1
        );


    if (convertedId >= m_uavCount)
    {
        return false;
    }


    id = convertedId;

    return true;
}


// ============================================================
// Extract numeric JSON field
// ============================================================

bool
AirSimUdpBridge::ExtractNumber(
    const std::string& message,
    const std::string& key,
    double& value
) const
{
    std::string searchKey =
        "\"" + key + "\"";


    size_t keyPosition =
        message.find(searchKey);


    if (keyPosition == std::string::npos)
    {
        return false;
    }


    size_t colon =
        message.find(
            ':',
            keyPosition + searchKey.length()
        );


    if (colon == std::string::npos)
    {
        return false;
    }


    size_t valueStart =
        colon + 1;


    // --------------------------------------------------------
    // Skip whitespace
    // --------------------------------------------------------

    while (
        valueStart < message.length() &&
        std::isspace(
            static_cast<unsigned char>(
                message[valueStart]
            )
        )
    )
    {
        valueStart++;
    }


    if (valueStart >= message.length())
    {
        return false;
    }


    const char* start =
        message.c_str() + valueStart;


    char* endPointer =
        nullptr;


    double parsedValue =
        std::strtod(
            start,
            &endPointer
        );


    if (endPointer == start)
    {
        return false;
    }


    value = parsedValue;

    return true;
}


// ============================================================
// Check whether all UAVs sent initial telemetry
// ============================================================

bool
AirSimUdpBridge::HasInitialStates() const
{
    for (uint32_t i = 0;
         i < m_uavCount;
         ++i)
    {
        if (!m_received[i])
        {
            return false;
        }
    }

    return true;
}


// ============================================================
// Get latest UAV state
// ============================================================

const AirSimUavState*
AirSimUdpBridge::GetLatestState(
    uint32_t id
) const
{
    if (id >= m_uavCount)
    {
        return nullptr;
    }


    if (!m_received[id])
    {
        return nullptr;
    }


    return &m_states[id];
}


// ============================================================
// Check telemetry freshness
// ============================================================

bool
AirSimUdpBridge::HasFreshState(
    uint32_t id,
    double now,
    double staleTimeout
) const
{
    if (id >= m_uavCount)
    {
        return false;
    }


    if (!m_received[id])
    {
        return false;
    }


    double age =
        now - m_lastReceiveTime[id];


    return age <= staleTimeout;
}


// ============================================================
// Get UAVs that have not sent telemetry
// ============================================================

std::vector<uint32_t>
AirSimUdpBridge::GetMissingInitialUavIds() const
{
    std::vector<uint32_t> missing;


    for (uint32_t i = 0;
         i < m_uavCount;
         ++i)
    {
        if (!m_received[i])
        {
            missing.push_back(i);
        }
    }


    return missing;
}

} // namespace ns3