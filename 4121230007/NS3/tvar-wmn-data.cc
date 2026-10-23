#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/olsr-module.h"                          // CHANGED: was aodv-module.h
#include "ns3/propagation-loss-model.h"
#include <fstream>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TvarRegimeSwitchingOLSR");   // CHANGED: updated component name

std::map<uint32_t, uint64_t> previousTxPackets;
std::map<uint32_t, uint64_t> previousRxPackets;
std::map<uint32_t, uint64_t> previousTxBytes;
std::ofstream dataFile;
double currentTrafficRate = 200.0;

void CollectFlowStats(Ptr<FlowMonitor> monitor, FlowMonitorHelper* flowHelper, double timeWindow) {
    static uint32_t windowCount = 0;

    if (windowCount % 10 == 0) {
        std::cout << "[Sim] Window " << windowCount << " | Time: " << Simulator::Now().GetSeconds()
                  << "s | Traffic: " << currentTrafficRate << " kbps" << std::endl;
    }

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper->GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        FlowId flowId = i->first;
        FlowMonitor::FlowStats fs = i->second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        std::ostringstream dstOss;
        t.destinationAddress.Print(dstOss);
        if (dstOss.str() != "10.1.1.10") continue;

        std::ostringstream srcOss;
        t.sourceAddress.Print(srcOss);
        std::string srcIp = srcOss.str();
        uint32_t sourceNode = std::stoi(srcIp.substr(srcIp.find_last_of('.') + 1)) - 1;

        uint64_t txPacketsWindow = fs.txPackets - previousTxPackets[flowId];
        uint64_t rxPacketsWindow = fs.rxPackets - previousRxPackets[flowId];
        uint64_t txBytesWindow   = fs.txBytes   - previousTxBytes[flowId];

        previousTxPackets[flowId] = fs.txPackets;
        previousRxPackets[flowId] = fs.rxPackets;
        previousTxBytes[flowId]   = fs.txBytes;

        if (txPacketsWindow == 0) continue;
        if (rxPacketsWindow > txPacketsWindow) rxPacketsWindow = txPacketsWindow;

        uint64_t lostPackets = txPacketsWindow - rxPacketsWindow;
        double lossRate      = static_cast<double>(lostPackets) / txPacketsWindow;
        double utilization   = (txBytesWindow * 8.0) / (timeWindow * 5000000.0);

        dataFile << windowCount << "," << sourceNode << "," << "route_" << sourceNode << "_to_9" << ","
                 << txPacketsWindow << "," << lostPackets << "," << lossRate << ","
                 << (utilization > 1.0 ? 1.0 : utilization) << "," << currentTrafficRate << "\n";
    }

    dataFile.flush();
    windowCount++;
    Simulator::Schedule(Seconds(timeWindow), &CollectFlowStats, monitor, flowHelper, timeWindow);
}

void UpdateTrafficRate(ApplicationContainer* apps, double newRate) {
    currentTrafficRate = newRate;
    for (uint32_t i = 0; i < apps->GetN(); i++) {
        Ptr<OnOffApplication> onoff = DynamicCast<OnOffApplication>(apps->Get(i));
        if (onoff) onoff->SetAttribute("DataRate", DataRateValue(DataRate(newRate * 1000)));
    }
}

int main(int argc, char *argv[]) {
    RngSeedManager::SetSeed(time(NULL));
    RngSeedManager::SetRun(time(NULL));

    uint32_t nNodes  = 10;
    double warmup    = 150.0;               // CHANGED: 100 → 150s
                                            // OLSR requires longer warmup
                                            // for proactive topology tables
                                            // to fully converge
    double duration  = 1800.0;
    double totalTime = warmup + duration;
    double window    = 5.0;

    NodeContainer nodes;
    nodes.Create(nNodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                   "Exponent", DoubleValue(2.3));
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                   "MinX",       DoubleValue(0.0),
                                   "MinY",       DoubleValue(0.0),
                                   "DeltaX",     DoubleValue(100.0),
                                   "DeltaY",     DoubleValue(100.0),
                                   "GridWidth",  UintegerValue(4),
                                   "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    OlsrHelper olsr;                        // CHANGED: was AodvHelper aodv
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr);        // CHANGED: was aodv
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    uint16_t port = 9;
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sink.Install(nodes.Get(9));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(totalTime));

    ApplicationContainer sourceApps;
    for (uint32_t i = 0; i < nNodes - 1; i++) {
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(interfaces.GetAddress(9), port));
        onoff.SetAttribute("PacketSize", UintegerValue(1024));
        onoff.SetAttribute("DataRate",   StringValue("200kbps"));
        onoff.SetAttribute("OnTime",     StringValue("ns3::ExponentialRandomVariable[Mean=2.0]"));
        onoff.SetAttribute("OffTime",    StringValue("ns3::ExponentialRandomVariable[Mean=1.0]"));

        ApplicationContainer app = onoff.Install(nodes.Get(i));
        app.Start(Seconds(warmup + (i * 1.5)));
        app.Stop(Seconds(totalTime));
        sourceApps.Add(app);
    }

    // Regime switching traffic scheduler
    double simTime = warmup + 60.0;
    Ptr<UniformRandomVariable> rngLow  = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> rngHigh = CreateObject<UniformRandomVariable>();

    rngLow->SetAttribute("Min",  DoubleValue(300.0));
    rngLow->SetAttribute("Max",  DoubleValue(500.0));
    rngHigh->SetAttribute("Min", DoubleValue(1800.0));
    rngHigh->SetAttribute("Max", DoubleValue(2500.0));

    bool isSpike = false;
    while (simTime < totalTime - 60) {
        double nextRate = isSpike ? rngLow->GetValue() : rngHigh->GetValue();
        Simulator::Schedule(Seconds(simTime), &UpdateTrafficRate, &sourceApps, nextRate);
        simTime += 90.0;
        isSpike = !isSpike;
    }

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();

    // CHANGED: output filename updated to reflect OLSR protocol
    dataFile.open("mesh_regime_switching_olsr.csv");
    dataFile << "time_window,source_node,route_id,packets_sent,packets_lost,loss_rate,utilization,traffic_rate_kbps\n";

    Simulator::Schedule(Seconds(warmup + window), &CollectFlowStats,
                        monitor, &flowHelper, window);

    std::cout << "Starting OLSR Regime Switching Simulation..." << std::endl;
    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();

    dataFile.close();
    Simulator::Destroy();
    std::cout << "Simulation Complete. Results: mesh_regime_switching_olsr.csv" << std::endl;
    return 0;
}
