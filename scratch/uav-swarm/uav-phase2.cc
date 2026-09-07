#include "ns3/aodv-helper.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/wifi-module.h"

#include "airsim-udp-bridge.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace ns3;

// ============================================================
// Configuration
// ============================================================

static uint32_t g_numUavs = 6;
static double g_simTime = 20.0;
static double g_speed = 2.0;
static double g_mobilityChangeTime = 10.0;
static bool g_useAirSim = false;
static std::string g_airSimBindAddress = "0.0.0.0";
static uint32_t g_airSimPort = 41451;
static double g_airSimUpdateMs = 50.0;
static double g_airSimStaleTimeout = 1.0;
static double g_airSimInitTimeout = 5.0;
static double g_airSimScaleX = 1.0;
static double g_airSimScaleY = 1.0;
static double g_airSimScaleZ = -1.0;
static double g_airSimOffsetX = 0.0;
static double g_airSimOffsetY = 0.0;
static double g_airSimOffsetZ = 0.0;

static const double g_neighborRange = 150.0;

struct AirSimMobilityContext
{
    NodeContainer nodes;
    std::unique_ptr<AirSimUdpBridge> bridge;
    std::ofstream log;
    std::vector<Vector> lastPosition;
    std::vector<bool> hasPosition;
    std::vector<bool> stale;
    bool initialStatesReceived{false};
    bool initializationTimedOut{false};
};

Vector
TransformAirSimPosition(const AirSimUavState& state)
{
    return Vector(g_airSimScaleX * state.x + g_airSimOffsetX,
                  g_airSimScaleY * state.y + g_airSimOffsetY,
                  g_airSimScaleZ * state.z + g_airSimOffsetZ);
}

Vector
TransformAirSimVelocity(const AirSimUavState& state)
{
    return Vector(g_airSimScaleX * state.vx,
                  g_airSimScaleY * state.vy,
                  g_airSimScaleZ * state.vz);
}

void
PollAirSimMobility(AirSimMobilityContext* context)
{
    double now = Simulator::Now().GetSeconds();
    context->bridge->Poll(now);
    if (context->bridge->HasInitialStates() && !context->initialStatesReceived)
    {
        context->initialStatesReceived = true;
        std::cout << "[AirSim] Initial state received for all UAVs at t = " << now << " s\n";
    }

    for (uint32_t i = 0; i < context->nodes.GetN(); ++i)
    {
        Ptr<MobilityModel> model = context->nodes.Get(i)->GetObject<MobilityModel>();
        const AirSimUavState* state = context->bridge->GetLatestState(i);
        bool fresh = context->bridge->HasFreshState(i, now, g_airSimStaleTimeout);
        if (fresh)
        {
            Vector position = TransformAirSimPosition(*state);
            Vector velocity = TransformAirSimVelocity(*state);
            model->SetPosition(position);
            context->lastPosition[i] = position;
            context->hasPosition[i] = true;
            context->stale[i] = false;
            context->log << std::fixed << std::setprecision(6) << now << ',' << i << ",Drone" << i + 1
                         << ',' << position.x << ',' << position.y << ',' << position.z << ',' << velocity.x
                         << ',' << velocity.y << ',' << velocity.z << ',' << state->timestamp << ",0\n";
        }
        else if (state != nullptr)
        {
            if (!context->stale[i])
            {
                std::cerr << "[AirSim][WARN] UAV " << i << " state is stale at t = " << now
                          << " s; freezing its last known position.\n";
                context->stale[i] = true;
            }
            if (context->hasPosition[i])
            {
                model->SetPosition(context->lastPosition[i]);
            }
            Vector position = context->hasPosition[i] ? context->lastPosition[i] : model->GetPosition();
            context->log << std::fixed << std::setprecision(6) << now << ',' << i << ",Drone" << i + 1
                         << ',' << position.x << ',' << position.y << ',' << position.z
                         << ",0.000000,0.000000,0.000000," << state->timestamp << ",1\n";
        }
    }
    Simulator::Schedule(MilliSeconds(g_airSimUpdateMs), &PollAirSimMobility, context);
}

void
CheckAirSimInitialization(AirSimMobilityContext* context)
{
    if (!context->initialStatesReceived)
    {
        context->initializationTimedOut = true;
        std::cerr << "[AirSim][ERROR] Telemetry initialization timeout after " << g_airSimInitTimeout
                  << " seconds. Missing:";
        for (uint32_t id : context->bridge->GetMissingInitialUavIds())
        {
            std::cerr << " Drone" << id + 1;
        }
        std::cerr << ". Stopping simulation.\n";
        Simulator::Stop();
    }
}

void
LogNeighbors(NodeContainer nodes, std::ofstream* neighborFile)
{
    double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<MobilityModel> source = nodes.Get(i)->GetObject<MobilityModel>();
        Vector position = source->GetPosition();
        Ptr<ConstantVelocityMobilityModel> velocityModel =
            DynamicCast<ConstantVelocityMobilityModel>(source);
        Vector velocity = velocityModel ? velocityModel->GetVelocity() : Vector(0.0, 0.0, 0.0);
        for (uint32_t j = 0; j < nodes.GetN(); ++j)
        {
            if (i == j)
            {
                continue;
            }
            Vector other = nodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
            double distance = CalculateDistance(position, other);
            if (distance <= g_neighborRange)
            {
                *neighborFile << std::fixed << std::setprecision(3) << now << ',' << i << ',' << j << ','
                              << distance << ',' << position.x << ',' << position.y << ',' << position.z << ','
                              << velocity.x << ',' << velocity.y << ',' << velocity.z << '\n';
            }
        }
    }
    if (now + 1.0 < g_simTime)
    {
        Simulator::Schedule(Seconds(1.0), &LogNeighbors, nodes, neighborFile);
    }
}

// ============================================================
// Change UAV mobility
// ============================================================

void
ChangeMobility(NodeContainer nodes)
{
    std::cout
        << "\n[INFO] Changing UAV mobility at t = "
        << Simulator::Now().GetSeconds()
        << " s\n";

    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<ConstantVelocityMobilityModel> model =
            nodes.Get(i)->GetObject<
                ConstantVelocityMobilityModel>();

        if (model == nullptr)
        {
            std::cerr
                << "[ERROR] UAV "
                << i
                << " does not have ConstantVelocityMobilityModel\n";

            continue;
        }

        if (i % 2 == 0)
        {
            // UAV0, UAV2, UAV4
            model->SetVelocity(
                Vector(
                    -g_speed,
                    1.0,
                    0.0));
        }
        else
        {
            // UAV1, UAV3, UAV5
            model->SetVelocity(
                Vector(
                    -g_speed,
                    -1.0,
                    0.0));
        }
    }
}

// ============================================================
// Main
// ============================================================

int
main(int argc, char* argv[])
{
    // --------------------------------------------------------
    // Command-line arguments
    // --------------------------------------------------------

    CommandLine cmd(__FILE__);

    cmd.AddValue(
        "nUavs",
        "Number of UAV nodes",
        g_numUavs);

    cmd.AddValue(
        "simTime",
        "Simulation time in seconds",
        g_simTime);

    cmd.AddValue(
        "speed",
        "Initial UAV speed in m/s",
        g_speed);

    cmd.AddValue(
        "changeTime",
        "Time at which UAV velocities change",
        g_mobilityChangeTime);

    cmd.AddValue("useAirSim", "Receive Drone1..Drone6 mobility over UDP", g_useAirSim);
    cmd.AddValue("airSimBindAddress", "IPv4 address on which to bind the UDP receiver", g_airSimBindAddress);
    cmd.AddValue("airSimPort", "UDP port used by AirSim telemetry", g_airSimPort);
    cmd.AddValue("airSimUpdateMs", "Non-blocking AirSim receiver polling interval in milliseconds", g_airSimUpdateMs);
    cmd.AddValue("airSimStaleTimeout", "Seconds before an AirSim UAV state is stale", g_airSimStaleTimeout);
    cmd.AddValue("airSimInitTimeout", "Seconds to wait for all initial AirSim UAV states", g_airSimInitTimeout);
    cmd.AddValue("airSimScaleX", "AirSim-to-NS-3 X scale", g_airSimScaleX);
    cmd.AddValue("airSimScaleY", "AirSim-to-NS-3 Y scale", g_airSimScaleY);
    cmd.AddValue("airSimScaleZ", "AirSim-to-NS-3 Z scale", g_airSimScaleZ);
    cmd.AddValue("airSimOffsetX", "AirSim-to-NS-3 X offset in metres", g_airSimOffsetX);
    cmd.AddValue("airSimOffsetY", "AirSim-to-NS-3 Y offset in metres", g_airSimOffsetY);
    cmd.AddValue("airSimOffsetZ", "AirSim-to-NS-3 Z offset in metres", g_airSimOffsetZ);

    cmd.Parse(argc, argv);

    // --------------------------------------------------------
    // Safety adjustment
    // --------------------------------------------------------

    if (g_numUavs < 2)
    {
        std::cerr
            << "[ERROR] Number of UAVs must be at least 2.\n";

        return 1;
    }

    if (g_mobilityChangeTime >= g_simTime)
    {
        g_mobilityChangeTime =
            g_simTime / 2.0;
    }

    if (g_useAirSim && (g_airSimUpdateMs <= 0.0 || g_airSimStaleTimeout <= 0.0 ||
                        g_airSimInitTimeout <= 0.0 || g_airSimPort > 65535))
    {
        std::cerr << "[ERROR] AirSim update interval, timeouts, and port must be positive and valid.\n";
        return 1;
    }

    if (g_useAirSim && g_numUavs != 6)
    {
        std::cerr << "[ERROR] AirSim mode currently requires exactly 6 UAVs (Drone1..Drone6).\n";
        return 1;
    }

    if (g_useAirSim)
    {
        // External I/O is polled by scheduled events; real-time pacing lets it receive live telemetry.
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    }

    // --------------------------------------------------------
    // Print configuration
    // --------------------------------------------------------

    std::cout << "\n";
    std::cout
        << "====================================================\n";
    std::cout
        << "        UAV SWARM - PHASE 2\n";
    std::cout
        << "     MOBILE AODV MULTI-HOP NETWORK\n";
    std::cout
        << "====================================================\n";

    std::cout
        << "Number of UAVs       : "
        << g_numUavs
        << "\n";

    std::cout
        << "Simulation time      : "
        << g_simTime
        << " seconds\n";

    std::cout
        << "Initial speed        : "
        << g_speed
        << " m/s\n";

    std::cout
        << "Mobility change time : "
        << g_mobilityChangeTime
        << " seconds\n";

    if (g_useAirSim)
    {
        std::cout << "AirSim mobility      : enabled\n"
                  << "AirSim UDP bind      : " << g_airSimBindAddress << ':' << g_airSimPort << "\n"
                  << "AirSim polling       : " << g_airSimUpdateMs << " ms\n"
                  << "AirSim mapping       : Drone1..Drone6 -> UAV0..UAV5\n";
    }

    // --------------------------------------------------------
    // Create UAV nodes
    // --------------------------------------------------------

    NodeContainer uavs;

    uavs.Create(g_numUavs);

    // --------------------------------------------------------
    // UAV mobility
    //
    // Baseline uses velocity-driven motion. AirSim mode uses position-only mobility
    // so no NS-3 trajectory is generated between telemetry updates.
    // --------------------------------------------------------

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> positionAllocator = CreateObject<ListPositionAllocator>();

    // Initial positions:
    //
    // UAV0 -> (0,   0, 50)
    // UAV1 -> (100, 0, 50)
    // UAV2 -> (200, 0, 50)
    // UAV3 -> (300, 0, 50)
    // UAV4 -> (400, 0, 50)
    // UAV5 -> (500, 0, 50)

    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        // AirSim mode starts at a neutral placeholder; its first telemetry state is authoritative.
        positionAllocator->Add(g_useAirSim ? Vector(0.0, 0.0, 0.0) : Vector(i * 100.0, 0.0, 50.0));
    }

    mobility.SetPositionAllocator(
        positionAllocator);

    mobility.SetMobilityModel(g_useAirSim ? "ns3::ConstantPositionMobilityModel"
                                          : "ns3::ConstantVelocityMobilityModel");

    mobility.Install(uavs);

    // --------------------------------------------------------
    // Set initial UAV velocities
    // --------------------------------------------------------

    if (!g_useAirSim)
    {
        for (uint32_t i = 0; i < g_numUavs; ++i)
        {
            Ptr<ConstantVelocityMobilityModel> model =
                uavs.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (model == nullptr)
            {
                std::cerr << "[ERROR] Could not obtain mobility model for UAV " << i << "\n";
                return 1;
            }

            if (i % 2 == 0)
            {
                model->SetVelocity(Vector(g_speed, 0.5, 0.0));
            }
            else
            {
                model->SetVelocity(Vector(g_speed, -0.5, 0.0));
            }
        }
    }

    // --------------------------------------------------------
    // Print initial UAV state
    // --------------------------------------------------------

    std::cout << "\nInitial UAV state:\n";

    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        Ptr<MobilityModel> model = uavs.Get(i)->GetObject<MobilityModel>();
        Vector position = model->GetPosition();
        Ptr<ConstantVelocityMobilityModel> velocityModel =
            DynamicCast<ConstantVelocityMobilityModel>(model);
        Vector velocity = velocityModel ? velocityModel->GetVelocity() : Vector(0.0, 0.0, 0.0);

        std::cout
            << "UAV "
            << i
            << " -> Position: ("
            << position.x
            << ", "
            << position.y
            << ", "
            << position.z
            << ")"
            << "  Velocity: ("
            << velocity.x
            << ", "
            << velocity.y
            << ", "
            << velocity.z
            << ")\n";
    }

    // --------------------------------------------------------
    // Wi-Fi channel
    //
    // One propagation-loss model only.
    // 2.412 GHz = 802.11g channel 1.
    // --------------------------------------------------------

    YansWifiChannelHelper channel;

    channel.SetPropagationDelay(
        "ns3::ConstantSpeedPropagationDelayModel");

    channel.AddPropagationLoss(
        "ns3::FriisPropagationLossModel",
        "Frequency",
        DoubleValue(2.412e9));

    YansWifiPhyHelper phy;

    phy.SetChannel(
        channel.Create());

    // --------------------------------------------------------
    // Wi-Fi
    // --------------------------------------------------------

    WifiHelper wifi;

    wifi.SetStandard(
        WIFI_STANDARD_80211g);

    wifi.SetRemoteStationManager(
        "ns3::ConstantRateWifiManager",
        "DataMode",
        StringValue("ErpOfdmRate6Mbps"),
        "ControlMode",
        StringValue("ErpOfdmRate6Mbps"),
        "RtsCtsThreshold",
        UintegerValue(0));

    WifiMacHelper mac;

    // No access point.
    // Every UAV communicates using ad-hoc Wi-Fi.
    mac.SetType(
        "ns3::AdhocWifiMac");

    NetDeviceContainer devices =
        wifi.Install(
            phy,
            mac,
            uavs);

    phy.EnablePcapAll("results/phase2/uav-swarm");

    // --------------------------------------------------------
    // Internet stack + AODV
    // --------------------------------------------------------

    AodvHelper aodv;

    InternetStackHelper internet;

    internet.SetRoutingHelper(
        aodv);

    internet.Install(uavs);

    // --------------------------------------------------------
    // IPv4 addresses
    // --------------------------------------------------------

    Ipv4AddressHelper ipv4;

    ipv4.SetBase(
        "10.0.0.0",
        "255.255.255.0");

    Ipv4InterfaceContainer interfaces =
        ipv4.Assign(devices);

    // Neighbor output is retained for analysis/plot.py.  Rows are directed and include
    // all UAV pairs within the documented 150 m analysis neighborhood range.
    std::ofstream neighborFile("results/phase2/neighbors.csv");
    if (!neighborFile.is_open())
    {
        std::cerr << "[ERROR] Could not open results/phase2/neighbors.csv\n";
        return 1;
    }
    neighborFile << "time,uav_id,neighbor_id,distance_m,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps\n";

    std::unique_ptr<AirSimMobilityContext> airSimContext;
    if (g_useAirSim)
    {
        airSimContext = std::make_unique<AirSimMobilityContext>();
        airSimContext->nodes = uavs;
        airSimContext->bridge =
            std::make_unique<AirSimUdpBridge>(g_numUavs, g_airSimBindAddress, g_airSimPort);
        airSimContext->lastPosition.assign(g_numUavs, Vector(0.0, 0.0, 0.0));
        airSimContext->hasPosition.assign(g_numUavs, false);
        airSimContext->stale.assign(g_numUavs, false);
        airSimContext->log.open("results/phase2/airsim-mobility.csv");
        if (!airSimContext->log.is_open())
        {
            std::cerr << "[ERROR] Could not open results/phase2/airsim-mobility.csv\n";
            return 1;
        }
        airSimContext->log << "time,uav_id,vehicle_name,x,y,z,vx,vy,vz,airsim_timestamp,stale\n";
        std::string bridgeError;
        if (!airSimContext->bridge->Start(bridgeError))
        {
            std::cerr << "[AirSim][ERROR] Could not start UDP bridge: " << bridgeError << '\n';
            return 1;
        }
        std::cout << "[AirSim] Waiting for telemetry on " << g_airSimBindAddress << ':'
                  << g_airSimPort << " (update=" << g_airSimUpdateMs << " ms, stale="
                  << g_airSimStaleTimeout << " s, init=" << g_airSimInitTimeout << " s)\n";
        for (uint32_t i = 0; i < g_numUavs; ++i)
        {
            std::cout << "[AirSim] Drone" << i + 1 << " -> Node" << i << "\n";
        }
    }

    // --------------------------------------------------------
    // NetAnim
    // --------------------------------------------------------

    AnimationInterface animation(
        "results/phase2/uav-phase2.xml");

    animation.SetMobilityPollInterval(
        Seconds(0.2));

    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        animation.UpdateNodeDescription(
            uavs.Get(i),
            g_useAirSim ? "Drone" + std::to_string(i + 1) + " / UAV-" + std::to_string(i)
                        : "UAV-" + std::to_string(i));
    }

    // --------------------------------------------------------
    // Print IP addresses
    // --------------------------------------------------------

    std::cout
        << "\nUAV IP addresses:\n";

    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        std::cout
            << "UAV "
            << i
            << " -> "
            << interfaces.GetAddress(i)
            << "\n";
    }

    // --------------------------------------------------------
    // UDP server
    //
    // Last UAV receives traffic.
    // --------------------------------------------------------

    uint16_t port = 9000;

    UdpServerHelper server(port);

    ApplicationContainer serverApps =
        server.Install(
            uavs.Get(g_numUavs - 1));

    serverApps.Start(
        Seconds(1.0));

    serverApps.Stop(
        Seconds(g_simTime));

    // --------------------------------------------------------
    // UDP client
    //
    // UAV0 -> Last UAV
    // --------------------------------------------------------

    UdpClientHelper client(
        interfaces.GetAddress(
            g_numUavs - 1),
        port);

    client.SetAttribute(
        "MaxPackets",
        UintegerValue(100000));

    client.SetAttribute(
        "Interval",
        TimeValue(
            MilliSeconds(200)));

    client.SetAttribute(
        "PacketSize",
        UintegerValue(512));

    ApplicationContainer clientApps =
        client.Install(
            uavs.Get(0));

    // Give AODV time to initialize.
    clientApps.Start(
        Seconds(5.0));

    clientApps.Stop(
        Seconds(g_simTime - 1.0));

    // --------------------------------------------------------
    // FlowMonitor
    // --------------------------------------------------------

    FlowMonitorHelper flowmonHelper;

    Ptr<FlowMonitor> monitor =
        flowmonHelper.InstallAll();

    // --------------------------------------------------------
    // Change UAV velocities halfway through simulation
    // --------------------------------------------------------

    if (!g_useAirSim)
    {
        Simulator::Schedule(Seconds(g_mobilityChangeTime), &ChangeMobility, uavs);
    }
    else
    {
        Simulator::ScheduleNow(&PollAirSimMobility, airSimContext.get());
        Simulator::Schedule(Seconds(g_airSimInitTimeout), &CheckAirSimInitialization, airSimContext.get());
    }

    Simulator::Schedule(Seconds(1.0), &LogNeighbors, uavs, &neighborFile);

    // --------------------------------------------------------
    // Stop simulation
    // --------------------------------------------------------

    Simulator::Stop(
        Seconds(g_simTime));

    std::cout
        << "\n[INFO] Starting simulation...\n";

    Simulator::Run();

    if (airSimContext)
    {
        airSimContext->bridge->Stop();
        airSimContext->log.close();
    }
    neighborFile.close();

    // --------------------------------------------------------
    // FlowMonitor statistics
    // --------------------------------------------------------

    monitor->CheckForLostPackets();

    FlowMonitor::FlowStatsContainer stats =
        monitor->GetFlowStats();

    uint64_t totalTxPackets = 0;
    uint64_t totalRxPackets = 0;
    uint64_t totalLostPackets = 0;
    uint64_t totalRxBytes = 0;

    double totalDelaySeconds = 0.0;

    for (const auto& flow : stats)
    {
        const FlowMonitor::FlowStats& flowStats =
            flow.second;

        totalTxPackets +=
            flowStats.txPackets;

        totalRxPackets +=
            flowStats.rxPackets;

        totalLostPackets +=
            flowStats.lostPackets;

        totalRxBytes +=
            flowStats.rxBytes;

        totalDelaySeconds +=
            flowStats.delaySum.GetSeconds();
    }

    // --------------------------------------------------------
    // Calculate PDR
    // --------------------------------------------------------

    double pdr = 0.0;

    if (totalTxPackets > 0)
    {
        pdr =
            100.0 *
            static_cast<double>(totalRxPackets) /
            static_cast<double>(totalTxPackets);
    }

    // --------------------------------------------------------
    // Calculate packet loss
    // --------------------------------------------------------

    double packetLoss = 0.0;

    if (totalTxPackets > 0)
    {
        packetLoss =
            100.0 *
            static_cast<double>(
                totalTxPackets - totalRxPackets) /
            static_cast<double>(totalTxPackets);
    }

    // --------------------------------------------------------
    // Calculate average delay
    // --------------------------------------------------------

    double averageDelayMs = 0.0;

    if (totalRxPackets > 0)
    {
        averageDelayMs =
            1000.0 *
            totalDelaySeconds /
            static_cast<double>(totalRxPackets);
    }

    // --------------------------------------------------------
    // Calculate throughput
    // --------------------------------------------------------

    double trafficStart = 5.0;
    double trafficEnd = g_simTime - 1.0;

    double trafficDuration =
        trafficEnd - trafficStart;

    double throughputKbps = 0.0;

    if (trafficDuration > 0.0)
    {
        throughputKbps =
            (static_cast<double>(totalRxBytes) * 8.0) /
            (trafficDuration * 1000.0);
    }

    // --------------------------------------------------------
    // Write summary.csv
    //
    // run_phase2.sh creates results/phase2.
    // --------------------------------------------------------

    std::ofstream summaryFile(
        "results/phase2/summary.csv");

    if (!summaryFile.is_open())
    {
        std::cerr
            << "[ERROR] Could not open "
            << "results/phase2/summary.csv\n";

        Simulator::Destroy();

        return 1;
    }

    summaryFile
        << "metric,value\n";

    summaryFile
        << "test,mobile_aodv_multi_hop\n";

    summaryFile
        << "source_uav,0\n";

    summaryFile
        << "destination_uav,"
        << g_numUavs - 1
        << "\n";

    summaryFile
        << "uav_count,"
        << g_numUavs
        << "\n";

    summaryFile
        << "simulation_time_s,"
        << g_simTime
        << "\n";

    summaryFile
        << "initial_speed_mps,"
        << g_speed
        << "\n";

    summaryFile
        << "mobility_change_time_s,"
        << g_mobilityChangeTime
        << "\n";

    summaryFile
        << "communication_range_m,"
        << g_neighborRange
        << "\n";

    summaryFile
        << "uav_initial_spacing_m,100\n";

    summaryFile
        << "tx_packets,"
        << totalTxPackets
        << "\n";

    summaryFile
        << "rx_packets,"
        << totalRxPackets
        << "\n";

    summaryFile
        << "lost_packets,"
        << totalLostPackets
        << "\n";

    summaryFile
        << std::fixed
        << std::setprecision(4);

    summaryFile
        << "packet_delivery_ratio_percent,"
        << pdr
        << "\n";

    summaryFile
        << "packet_loss_percent,"
        << packetLoss
        << "\n";

    summaryFile
        << "average_delay_ms,"
        << averageDelayMs
        << "\n";

    summaryFile
        << "throughput_kbps,"
        << throughputKbps
        << "\n";

    summaryFile.close();

    // --------------------------------------------------------
    // Print results
    // --------------------------------------------------------

    std::cout
        << "\n";

    std::cout
        << "====================================================\n";

    std::cout
        << "              SIMULATION RESULTS\n";

    std::cout
        << "====================================================\n";

    std::cout
        << "Test             : Mobile AODV multi-hop\n";

    std::cout
        << "Source           : UAV0\n";

    std::cout
        << "Destination      : UAV"
        << g_numUavs - 1
        << "\n";

    std::cout
        << "Initial spacing  : 100 m\n";

    std::cout
        << "Neighbor range   : "
        << g_neighborRange
        << " m\n";

    std::cout
        << "Initial speed    : "
        << g_speed
        << " m/s\n";

    std::cout
        << "TX packets       : "
        << totalTxPackets
        << "\n";

    std::cout
        << "RX packets       : "
        << totalRxPackets
        << "\n";

    std::cout
        << "Lost packets     : "
        << totalLostPackets
        << "\n";

    std::cout
        << std::fixed
        << std::setprecision(2);

    std::cout
        << "PDR              : "
        << pdr
        << " %\n";

    std::cout
        << "Packet loss      : "
        << packetLoss
        << " %\n";

    std::cout
        << "Average delay    : "
        << averageDelayMs
        << " ms\n";

    std::cout
        << "Throughput       : "
        << throughputKbps
        << " Kbps\n";

    std::cout
        << "\n";

    std::cout
        << "Output files:\n";

    std::cout
        << "  results/phase2/summary.csv\n";

    std::cout
        << "  results/phase2/uav-phase2.xml\n";

    std::cout
        << "  results/phase2/neighbors.csv\n";

    if (g_useAirSim)
    {
        std::cout << "  results/phase2/airsim-mobility.csv\n";
    }

    std::cout
        << "====================================================\n";

    // --------------------------------------------------------
    // Destroy simulator
    // --------------------------------------------------------

    bool airSimInitializationTimedOut =
        airSimContext != nullptr && airSimContext->initializationTimedOut;
    Simulator::Destroy();

    return airSimInitializationTimedOut ? 2 : 0;
}
