#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <cmath>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WmnMultiHopMesh");

std::ofstream g_outFile;

void CollectStats(Ptr<FlowMonitor> monitor,
                  Ptr<Ipv4FlowClassifier> classifier,
                  double interval,
                  uint64_t capacity)
{
    static std::map<FlowId, uint64_t> lastTxBytes;
    static std::map<FlowId, uint64_t> lastRxBytes;
    static std::map<FlowId, uint32_t> lastTxPackets;
    static std::map<FlowId, uint32_t> lastRxPackets;

    auto stats = monitor->GetFlowStats();

    double timeMinutes = Simulator::Now().GetSeconds() / 60.0;
    int totalMinutes = (int)timeMinutes;
    int hour = (totalMinutes / 60) % 24;
    int day = (totalMinutes / 1440) % 7;

    double hourSin = sin(2.0 * M_PI * hour / 24.0);
    double hourCos = cos(2.0 * M_PI * hour / 24.0);
    double daySin  = sin(2.0 * M_PI * day / 7.0);
    double dayCos  = cos(2.0 * M_PI * day / 7.0);
    int isWeekend  = (day == 5 || day == 6) ? 1 : 0;

    for (auto const &flow : stats)
    {
        FlowId flowId = flow.first;
        auto const &fs = flow.second;
        auto tuple = classifier->FindFlow(flowId);

        uint32_t src = tuple.sourceAddress.Get() & 0xFF;
        uint32_t dst = tuple.destinationAddress.Get() & 0xFF;

        if (!lastTxBytes.count(flowId))
        {
            lastTxBytes[flowId] = 0;
            lastRxBytes[flowId] = 0;
            lastTxPackets[flowId] = 0;
            lastRxPackets[flowId] = 0;
        }

        uint64_t txBytes = fs.txBytes - lastTxBytes[flowId];
        uint64_t rxBytes = fs.rxBytes - lastRxBytes[flowId];
        uint32_t txPkts  = fs.txPackets - lastTxPackets[flowId];
        uint32_t rxPkts  = fs.rxPackets - lastRxPackets[flowId];

        double utilization = (txBytes * 8.0 * 100.0) / (interval * capacity);
        uint32_t lost = (txPkts > rxPkts) ? (txPkts - rxPkts) : 0;
        int hasLoss = lost > 0 ? 1 : 0;
        double deliveryRatio = txPkts > 0 ? (double)rxPkts / txPkts : 1.0;

        g_outFile
            << flowId << ","
            << (src - 1) << ","
            << (dst - 1) << ","
            << timeMinutes << ","
            << hour << ","
            << hourSin << ","
            << hourCos << ","
            << day << ","
            << daySin << ","
            << dayCos << ","
            << isWeekend << ","
            << utilization << ","
            << txPkts << ","
            << rxPkts << ","
            << lost << ","
            << hasLoss << ","
            << deliveryRatio << "\n";

        lastTxBytes[flowId] = fs.txBytes;
        lastRxBytes[flowId] = fs.rxBytes;
        lastTxPackets[flowId] = fs.txPackets;
        lastRxPackets[flowId] = fs.rxPackets;
    }

    Simulator::Schedule(Seconds(interval),
                        &CollectStats,
                        monitor,
                        classifier,
                        interval,
                        capacity);
}

int main(int argc, char *argv[])
{
    std::string trafficLevel = "medium";
    std::string outputFile = "wmn-data.csv";
    uint32_t runNumber = 1;

    CommandLine cmd;
    cmd.AddValue("traffic", "low | medium | high", trafficLevel);
    cmd.AddValue("output", "CSV output file", outputFile);
    cmd.AddValue("run", "Run number for RNG seed", runNumber);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(runNumber);

    uint32_t nNodes = 10;
    uint32_t gatewayNode = 9;
    double simulationTime = 1200.0;     // 20 minutes
    double collectionInterval = 60.0;   // 1 minute

    // Data rates for different traffic levels
    uint32_t baseRate;
    double onTimeMean, offTimeMean;
    
    if (trafficLevel == "low") {
        baseRate = 150000;      // 300 Kbps
        onTimeMean = 2.0;
        offTimeMean = 3.0;
    } else if (trafficLevel == "high") {
        baseRate = 450000;      // 900 Kbps
        onTimeMean = 4.0;
        offTimeMean = 1.0;
    } else {  // medium
        baseRate = 300000;      // 600 Kbps
        onTimeMean = 3.0;
        offTimeMean = 2.0;
    }

    std::cout << "=== WMN Multi-Hop Mesh Simulation ===" << std::endl;
    std::cout << "Traffic: " << trafficLevel << std::endl;
    std::cout << "Base rate: " << (baseRate / 1000.0) << " Kbps per flow" << std::endl;
    std::cout << "Run: " << runNumber << std::endl;
    std::cout << "Duration: " << (simulationTime / 60.0) << " minutes" << std::endl;

    // Create nodes
    NodeContainer nodes;
    nodes.Create(nNodes);

    // WiFi configuration - 802.11a
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("OfdmRate54Mbps"));

    // Simple channel model
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(25.0));
    phy.Set("TxPowerEnd", DoubleValue(25.0));

    // Ad-hoc MAC
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // Grid topology: 2 rows × 5 columns
    // N0  N1  N2  N3  N4
    // N5  N6  N7  N8  N9(Gateway)
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(30.0),  // 50m spacing
                                  "DeltaY", DoubleValue(30.0),
                                  "GridWidth", UintegerValue(5),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // Internet stack with global routing
    InternetStackHelper stack;
    stack.Install(nodes);

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    // Create flows: N0-N8 → Gateway (N9)
    uint16_t basePort = 9000;

    for (uint32_t i = 0; i < nNodes - 1; i++)
    {
        // PacketSink at gateway
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), basePort + i));
        ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(gatewayNode));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simulationTime));

        // OnOff application at each source node
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(interfaces.GetAddress(gatewayNode), basePort + i));

        uint32_t pktSize = 512 + (i * 50);  // Vary packet size: 512-912 bytes
        
        onoff.SetAttribute("PacketSize", UintegerValue(pktSize));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(baseRate)));
        
        // Vary on/off times per flow for diversity
        std::stringstream onTime, offTime;
        onTime << "ns3::ExponentialRandomVariable[Mean=" << (onTimeMean + i * 0.15) << "]";
        offTime << "ns3::ExponentialRandomVariable[Mean=" << (offTimeMean + i * 0.1) << "]";
        
        onoff.SetAttribute("OnTime", StringValue(onTime.str()));
        onoff.SetAttribute("OffTime", StringValue(offTime.str()));

        ApplicationContainer clientApp = onoff.Install(nodes.Get(i));
        clientApp.Start(Seconds(2.0 + i * 0.3));  // Stagger starts
        clientApp.Stop(Seconds(simulationTime - 1.0));
    }

    // FlowMonitor
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());

    // Open output file
    g_outFile.open(outputFile.c_str());
    g_outFile
        << "FlowID,SourceNode,DestNode,TimeMinutes,Hour,HourSin,HourCos,Day,"
        << "DaySin,DayCos,IsWeekend,FlowUtilization,TxPackets,RxPackets,"
        << "PacketsLost,HasPacketLoss,DeliveryRatio\n";

    uint64_t linkCapacity = 54000000;  // 54 Mbps

    // Schedule statistics collection
    Simulator::Schedule(Seconds(collectionInterval),
                        &CollectStats,
                        monitor,
                        classifier,
                        collectionInterval,
                        linkCapacity);

    std::cout << "\nStarting simulation..." << std::endl;
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    // Final statistics
    auto finalStats = monitor->GetFlowStats();
    uint64_t totalTx = 0, totalRx = 0;
    for (auto const &f : finalStats) {
        totalTx += f.second.txPackets;
        totalRx += f.second.rxPackets;
    }

    double lossRate = totalTx > 0 ? (100.0 * (totalTx - totalRx) / totalTx) : 0.0;

    std::cout << "\n=== Simulation Complete ===" << std::endl;
    std::cout << "Total TX: " << totalTx << std::endl;
    std::cout << "Total RX: " << totalRx << std::endl;
    std::cout << "Loss rate: " << lossRate << "%" << std::endl;
    std::cout << "Data written to: " << outputFile << std::endl;

    Simulator::Destroy();
    g_outFile.close();

    return 0;
}
