// Network topology
//
//    SRC
//     |<=== source network
//     A-----B
//      \   / \   all networks have cost 1, except
//       \ /  |   for the direct link from C to D, which
//        C  /    has cost 10
//        | /
//        |/
//        D
//        |<=== target network
//       DST
//
//
// A, B, C and D are RIPng routers.
// A and D are configured with static addresses.
// SRC and DST will exchange packets.
//
// After about 3 seconds, the topology is built, and Echo Reply will be received.
// After 40 seconds, the link between B and D will break, causing a route failure.
// After 44 seconds from the failure, the routers will recovery from the failure.
// Split Horizoning should affect the recovery time, but it is not. See the manual
// for an explanation of this effect.
//
// If "showPings" is enabled, the user will see:
// 1) if the ping has been acknowledged
// 2) if a Destination Unreachable has been received by the sender
// 3) nothing, when the Echo Request has been received by the destination but
//    the Echo Reply is unable to reach the sender.
// Examining the .pcap files with Wireshark can confirm this effect.

// Two types of Update Weight Algorithms included here

#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"

#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <vector>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RipNgSimpleRouting");

void
TearDownLink(Ptr<Node> nodeA, Ptr<Node> nodeB, uint32_t interfaceA, uint32_t interfaceB)
{
    nodeA->GetObject<Ipv6>()->SetDown(interfaceA);
    nodeB->GetObject<Ipv6>()->SetDown(interfaceB);
}

void
ChangeWeights(RipNgHelper& ripNgRouting,
              std::map<std::pair<uint32_t, uint32_t>, uint32_t>& weightMap,
              Ptr<Node> dst)
{
    // Get the destination node's position
    Ptr<MobilityModel> dstMobility = dst->GetObject<MobilityModel>();
    Vector dstPosition = dstMobility->GetPosition();

    // Create a map to store the shortest path distances between nodes
    std::map<uint32_t, std::map<uint32_t, double>> shortestPathDistances;

    // Initialize shortest path distances to infinity for all node pairs
    for (auto& pair : weightMap)
    {
        shortestPathDistances[pair.first.first][pair.first.second] =
            std::numeric_limits<double>::infinity();
        shortestPathDistances[pair.first.second][pair.first.first] =
            std::numeric_limits<double>::infinity();
    }

    // Implement Dijkstra's algorithm to calculate shortest path distances
    for (auto& pair : weightMap)
    {
        uint32_t sourceNode = pair.first.first;
        uint32_t destinationNode = pair.first.second;
        double weight = pair.second;

        // Initialize priority queue for Dijkstra's algorithm
        std::priority_queue<std::pair<double, uint32_t>,
                            std::vector<std::pair<double, uint32_t>>,
                            std::greater<std::pair<double, uint32_t>>>
            pq;

        // Initialize distance to source node as 0
        shortestPathDistances[sourceNode][sourceNode] = 0;
        pq.push({0, sourceNode});

        // Run Dijkstra's algorithm
        while (!pq.empty())
        {
            uint32_t currentNode = pq.top().second;
            pq.pop();

            // Get the current node's position
            Ptr<MobilityModel> currentMobility =
                NodeList::GetNode(currentNode)->GetObject<MobilityModel>();
            Vector currentPosition = currentMobility->GetPosition();

            // Check if the current node is the destination
            if (currentPosition == dstPosition)
            {
                // Update weights for all links to this node
                for (auto& adjacentPair : weightMap)
                {
                    uint32_t adjacentNode = adjacentPair.first.first == currentNode
                                                ? adjacentPair.first.second
                                                : adjacentPair.first.first;
                    uint32_t interface = adjacentPair.second;

                    double distance = shortestPathDistances[sourceNode][currentNode];
                    uint32_t weight = static_cast<uint32_t>(distance);

                    ripNgRouting.SetInterfaceMetric(NodeList::GetNode(adjacentNode),
                                                    interface,
                                                    weight);
                }
                break;
            }

            // Iterate through adjacent nodes of the current node
            for (auto& adjacentPair : weightMap)
            {
                uint32_t adjacentNode = adjacentPair.first.first == currentNode
                                            ? adjacentPair.first.second
                                            : adjacentPair.first.first;
                double edgeWeight = adjacentPair.second;

                // Relaxation step: Update shortest path distance if a shorter path is found
                if (shortestPathDistances[sourceNode][adjacentNode] >
                    shortestPathDistances[sourceNode][currentNode] + edgeWeight)
                {
                    shortestPathDistances[sourceNode][adjacentNode] =
                        shortestPathDistances[sourceNode][currentNode] + edgeWeight;
                    pq.push({shortestPathDistances[sourceNode][adjacentNode], adjacentNode});
                }
            }
        }
    }
}

int
main(int argc, char** argv)
{
    bool verbose = false;
    bool printRoutingTables = false;
    bool showPings = false;
    std::string SplitHorizon("PoisonReverse");

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "turn on log components", verbose);
    cmd.AddValue("printRoutingTables",
                 "Print routing tables at 30, 60 and 90 seconds",
                 printRoutingTables);
    cmd.AddValue("showPings", "Show Ping6 reception", showPings);
    cmd.AddValue("splitHorizonStrategy",
                 "Split Horizon strategy to use (NoSplitHorizon, SplitHorizon, PoisonReverse)",
                 SplitHorizon);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("RipNgSimpleRouting", LOG_LEVEL_INFO);
        LogComponentEnable("RipNg", LOG_LEVEL_ALL);
        // LogComponentEnable("Icmpv6L4Protocol", LOG_LEVEL_INFO);
        // LogComponentEnable("Ipv6Interface", LOG_LEVEL_ALL);
        // LogComponentEnable("Icmpv6L4Protocol", LOG_LEVEL_ALL);
        // LogComponentEnable("NdiscCache", LOG_LEVEL_ALL);
        // LogComponentEnable("Ping", LOG_LEVEL_ALL);
    }

    if (SplitHorizon == "NoSplitHorizon")
    {
        Config::SetDefault("ns3::RipNg::SplitHorizon", EnumValue(RipNg::NO_SPLIT_HORIZON));
    }
    else if (SplitHorizon == "SplitHorizon")
    {
        Config::SetDefault("ns3::RipNg::SplitHorizon", EnumValue(RipNg::SPLIT_HORIZON));
    }
    else
    {
        Config::SetDefault("ns3::RipNg::SplitHorizon", EnumValue(RipNg::POISON_REVERSE));
    }

    NS_LOG_INFO("Create nodes.");
    Ptr<Node> src = CreateObject<Node>();
    Names::Add("SrcNode", src);
    Ptr<Node> dst = CreateObject<Node>();
    Names::Add("DstNode", dst);
    Ptr<Node> a = CreateObject<Node>();
    Names::Add("RouterA", a);
    Ptr<Node> b = CreateObject<Node>();
    Names::Add("RouterB", b);
    Ptr<Node> c = CreateObject<Node>();
    Names::Add("RouterC", c);
    Ptr<Node> d = CreateObject<Node>();
    Names::Add("RouterD", d);

    // Create containers for node configurations
    NodeContainer net1(src, a);
    NodeContainer net2(a, b);
    NodeContainer net3(a, c);
    NodeContainer net4(b, c);
    NodeContainer net5(c, d);
    NodeContainer net6(b, d);
    NodeContainer net7(d, dst);
    NodeContainer routers(a, b, c, d);
    NodeContainer nodes(src, dst);

    // Create channels
    NS_LOG_INFO("Create channels.");
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(5000000));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    NetDeviceContainer ndc1 = csma.Install(net1);
    NetDeviceContainer ndc2 = csma.Install(net2);
    NetDeviceContainer ndc3 = csma.Install(net3);
    NetDeviceContainer ndc4 = csma.Install(net4);
    NetDeviceContainer ndc5 = csma.Install(net5);
    NetDeviceContainer ndc6 = csma.Install(net6);
    NetDeviceContainer ndc7 = csma.Install(net7);

    // Create mobility models and set positions for nodes
    Ptr<ConstantPositionMobilityModel> srcPosition = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> dstPosition = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> aPosition = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> bPosition = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> cPosition = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> dPosition = CreateObject<ConstantPositionMobilityModel>();

    // Set positions for the nodes
    srcPosition->SetPosition(Vector(0, 0, 0));   // Adjust coordinates as needed
    dstPosition->SetPosition(Vector(30, 30, 0)); // Adjust coordinates as needed
    aPosition->SetPosition(Vector(10, 10, 0));
    bPosition->SetPosition(Vector(20, 10, 0));
    cPosition->SetPosition(Vector(20, 20, 0));
    dPosition->SetPosition(Vector(10, 20, 0));

    // Assign mobility models to nodes
    Ptr<MobilityModel> srcMobility = srcPosition;
    Ptr<MobilityModel> dstMobility = dstPosition;
    Ptr<MobilityModel> aMobility = aPosition;
    Ptr<MobilityModel> bMobility = bPosition;
    Ptr<MobilityModel> cMobility = cPosition;
    Ptr<MobilityModel> dMobility = dPosition;

    // Install mobility models to nodes
    src->AggregateObject(srcMobility);
    dst->AggregateObject(dstMobility);
    a->AggregateObject(aMobility);
    b->AggregateObject(bMobility);
    c->AggregateObject(cMobility);
    d->AggregateObject(dMobility);

    NS_LOG_INFO("Create IPv6 and routing");
    RipNgHelper ripNgRouting;

    // Rule of thumb:
    // Interfaces are added sequentially, starting from 0
    // However, interface 0 is always the loopback...
    // ripNgRouting.ExcludeInterface(a, 1);
    // ripNgRouting.ExcludeInterface(d, 3);

    // ripNgRouting.SetInterfaceMetric(c, 3, 10);
    // ripNgRouting.SetInterfaceMetric(d, 1, 10);

    std::map<std::pair<uint32_t, uint32_t>, uint32_t> weightMap;
    weightMap[std::make_pair(a->GetId(), b->GetId())] = 1;
    weightMap[std::make_pair(a->GetId(), c->GetId())] = 2;
    weightMap[std::make_pair(b->GetId(), c->GetId())] = 3;
    weightMap[std::make_pair(b->GetId(), d->GetId())] = 4;
    weightMap[std::make_pair(c->GetId(), d->GetId())] = 5;

    for (auto& pair : weightMap)
    {
        uint32_t nodeId1 = pair.first.first;
        uint32_t nodeId2 = pair.first.second;
        uint32_t interface1 = pair.second;
        uint32_t interface2 = pair.second + 1;

        ripNgRouting.SetInterfaceMetric(NodeList::GetNode(nodeId1), interface1, pair.second);
        ripNgRouting.SetInterfaceMetric(NodeList::GetNode(nodeId2), interface2, pair.second);
    }

    Ipv6ListRoutingHelper listRH;
    listRH.Add(ripNgRouting, 0);
    Ipv6StaticRoutingHelper staticRh;
    listRH.Add(staticRh, 5);

    InternetStackHelper internetv6;
    internetv6.SetIpv4StackInstall(false);
    internetv6.SetRoutingHelper(listRH);
    internetv6.Install(routers);

    InternetStackHelper internetv6Nodes;
    internetv6Nodes.SetIpv4StackInstall(false);
    internetv6Nodes.Install(nodes);

    AnimationInterface anim("dij.xml");
    // anim.SetConstantPosition(a, 10, 10);
    // anim.SetConstantPosition(b, 20, 10);
    // anim.SetConstantPosition(c, 20, 20);
    // anim.SetConstantPosition(d, 10, 20);

    // anim.SetConstantPosition(src, 0, 0);
    // anim.SetConstantPosition(dst, 30, 30);

    anim.UpdateNodeDescription(src, "SrcNode");
    anim.UpdateNodeDescription(dst, "DstNode");
    anim.UpdateNodeDescription(a, "RouterA");
    anim.UpdateNodeDescription(b, "RouterB");
    anim.UpdateNodeDescription(c, "RouterC");
    anim.UpdateNodeDescription(d, "RouterD");

    // Assign addresses.
    // The source and destination networks have global addresses
    // The "core" network just needs link-local addresses for routing.
    // We assign global addresses to the routers as well to receive
    // ICMPv6 errors.
    NS_LOG_INFO("Assign IPv6 Addresses.");
    Ipv6AddressHelper ipv6;

    ipv6.SetBase(Ipv6Address("2002:1::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic1 = ipv6.Assign(ndc1);
    iic1.SetForwarding(1, true);
    iic1.SetDefaultRouteInAllNodes(1);

    ipv6.SetBase(Ipv6Address("2001:0:1::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic2 = ipv6.Assign(ndc2);
    iic2.SetForwarding(0, true);
    iic2.SetForwarding(1, true);

    ipv6.SetBase(Ipv6Address("2001:0:2::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic3 = ipv6.Assign(ndc3);
    iic3.SetForwarding(0, true);
    iic3.SetForwarding(1, true);

    ipv6.SetBase(Ipv6Address("2001:0:3::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic4 = ipv6.Assign(ndc4);
    iic4.SetForwarding(0, true);
    iic4.SetForwarding(1, true);

    ipv6.SetBase(Ipv6Address("2001:0:4::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic5 = ipv6.Assign(ndc5);
    iic5.SetForwarding(0, true);
    iic5.SetForwarding(1, true);

    ipv6.SetBase(Ipv6Address("2001:0:5::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic6 = ipv6.Assign(ndc6);
    iic6.SetForwarding(0, true);
    iic6.SetForwarding(1, true);

    ipv6.SetBase(Ipv6Address("2001:2::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer iic7 = ipv6.Assign(ndc7);
    iic7.SetForwarding(0, true);
    iic7.SetDefaultRouteInAllNodes(0);

    csma.EnablePcap("result_dij-DstNode-0", dst->GetId(), false, true);

    // Disable pcap on all other interfaces
    for (uint32_t i = 0; i < routers.GetN(); ++i)
    {
        Ptr<Node> router = routers.Get(i);
        if (router != dst)
        {
            csma.EnablePcapAll("result_dij", false);
        }
    }

    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        for (uint32_t j = 0; j < nodes.Get(i)->GetNDevices(); ++j)
        {
            Ptr<Node> node = nodes.Get(i);
            Ptr<NetDevice> device = node->GetDevice(j);

            // Retrieve the IPv6 address of the device
            Ipv6InterfaceAddress addr =
                iic1.GetAddress(j,
                                1); // Using iic1 for demonstration, replace with appropriate iicx

            // Convert IPv6 address to string
            std::ostringstream oss;
            oss << addr.GetAddress();

            // Set the IPv6 address as the node label
            anim.UpdateNodeDescription(node, oss.str());
        }
    }

    NS_LOG_INFO("Create Applications.");
    uint32_t packetSize = 1024;
    Time interPacketInterval = Seconds(1.0);
    PingHelper ping(iic7.GetAddress(1, 1));

    ping.SetAttribute("Interval", TimeValue(interPacketInterval));
    ping.SetAttribute("Size", UintegerValue(packetSize));
    if (showPings)
    {
        ping.SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::VERBOSE));
    }
    ApplicationContainer apps = ping.Install(src);
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(110.0));

    AsciiTraceHelper ascii;
    csma.EnableAsciiAll(ascii.CreateFileStream("result_dij.tr"));
    csma.EnablePcapAll("result_dij", true);

    // Simulator::Schedule(Seconds(5), &TearDownLink, b, d, 3, 2);
    Simulator::Schedule(Seconds(5.0),
                        &ChangeWeights,
                        std::ref(ripNgRouting),
                        std::ref(weightMap),
                        dst);
    Simulator::Schedule(Seconds(15.0),
                        &ChangeWeights,
                        std::ref(ripNgRouting),
                        std::ref(weightMap),
                        dst);
    Simulator::Schedule(Seconds(25), &TearDownLink, b, d, 3, 2);
    Simulator::Schedule(Seconds(30.0),
                        &ChangeWeights,
                        std::ref(ripNgRouting),
                        std::ref(weightMap),
                        dst);

    /* Now, do the actual simulation. */
    // Create and install the FlowMonitor before running the simulation
    Ptr<FlowMonitor> monitor;
    FlowMonitorHelper flowmon;
    monitor = flowmon.InstallAll();
    
    NS_LOG_INFO("Run Simulation.");
    Simulator::Stop(Seconds(40));
    Simulator::Run();

    NS_LOG_INFO("Run Simulation.");

    // Check if monitor is null before using it
    if (monitor)
    {
        monitor->CheckForLostPackets();
        Ptr<Ipv6FlowClassifier> classifier =
            DynamicCast<Ipv6FlowClassifier>(flowmon.GetClassifier());
        std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

        monitor->SerializeToXmlFile("dij_flow.xml", true, true);

        for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin();
             iter != stats.end();
             ++iter)
        {
            Ipv6FlowClassifier::FiveTuple flowKey =
                classifier->FindFlow(iter->first);      // Use iter instead of flow
            FlowMonitor::FlowStats flow = iter->second; // Retrieve flow stats using iter

            std::cout << "Flow: " << flowKey.sourceAddress << " -> " << flowKey.destinationAddress
                      << std::endl;
            std::cout << "  Tx Packets: " << flow.txPackets << std::endl;
            std::cout << "  Rx Packets: " << flow.rxPackets << std::endl;
            std::cout << "  Lost Packets: " << flow.lostPackets << std::endl;
            std::cout << "  Tx Bytes: " << flow.txBytes << std::endl;
            std::cout << "  Rx Bytes: " << flow.rxBytes << std::endl;
            std::cout << "  Delay Sum: " << flow.delaySum.GetSeconds() << " s" << std::endl;
            std::cout << "  Jitter Sum: " << flow.jitterSum.GetSeconds() << " s" << std::endl;
            std::cout << "  Mean Packet Size: " << (flow.txBytes / flow.txPackets) << " bytes"
                      << std::endl;
        }
    }
    else
    {
        NS_LOG_ERROR("Monitor is null. Unable to retrieve flow statistics.");
    }

    Simulator::Destroy();
    NS_LOG_INFO("Done.");

    return 0;
}