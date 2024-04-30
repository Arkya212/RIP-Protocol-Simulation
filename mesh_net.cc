#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ipv4-global-routing-helper.h"

using namespace ns3;
using namespace std;
// Custom routing class
class DVRouting : public Ipv4Routing {
public:
  DVRouting();

  // Methods for routing table management and packet forwarding
  void UpdateRoutingTable(const Ptr<Ipv4Route>& route) override;
  virtual bool RouteOutput(Ptr<Packet> p, const Ipv4Header& header, Ptr<NetDevice> out, bool judging, bool forwarding) override;

private:
  // Routing table data structure
  std::map<Ipv4Address, std::pair<Ipv4Address, int>> routingTable;
};

DVRouting::DVRouting() {}

void DVRouting::UpdateRoutingTable(const Ptr<Ipv4Route>& route) {
  Ipv4Address destination = route->GetDestination();
  Ipv4Address nexthop = route->GetNextHop();
  int hopCount = route->GetHopCount();

  // Update routing table with the new or better route (based on hop count)
  if (routingTable.count(destination) == 0 || hopCount < routingTable[destination].second) {
    routingTable[destination] = std::make_pair(nexthop, hopCount);
  }
}

bool DVRouting::RouteOutput(Ptr<Packet> p, const Ipv4Header& header, Ptr<NetDevice> out, bool judging, bool forwarding) {
  if (!forwarding) {
    return false;
  }

  Ipv4Address destination = header.GetDestination();

  // Check routing table for next hop
  if (routingTable.count(destination) > 0) {
    Ipv4Address nexthop = routingTable[destination].first;
    // Forward packet to the next hop interface
    return out->Send(p, nexthop);
  } else {
    // Destination not found in routing table (handle this case)
    NS_LOG_WARN ("Packet dropped: Destination not found");
    return false;
  }
}

// Point-to-point network scenario with DV routing
static void RunSimulation() {
  NodeContainer nodes;
  nodes.Create(2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
  pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

  NetDeviceContainer devices = pointToPoint.Install(nodes);

  Ptr<Ipv4ListRoutingProtocol> routingHelper = CreateObject<Ipv4ListRoutingProtocol>();
  routingHelper->AddRoutingProtocol(CreateObject<DVRouting>());

  InternetStackHelper internet;
  internet.SetRoutingHelper(routingHelper);
  internet.Install(nodes);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign(devices);

  UdpEchoServerHelper echoServer(9);
  ApplicationContainer serverApps = echoServer.Install(nodes.Get(1));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(10.0));

  UdpEchoClientHelper echoClient(interfaces.GetAddress(0), 9);
  echoClient.SetAttribute("MaxPackets", UintegerValue(10));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));
  ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(10.0));

  // No need for complex callback. Just print a message when a packet is received.
  NS_LOG_INFO ("Server (", interfaces.GetAddress(1), ") is ready to receive packets.");

  Simulator::Stop(Seconds(11.0));
  Simulator::Run();
  Simulator::Destroy();
}

int main(int argc, char* argv[]) {
  LogComponentEnable("DVRouting", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  RunSimulation();

  return 0;
}
