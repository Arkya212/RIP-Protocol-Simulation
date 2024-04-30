#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/callback.h"

// Add this line to register the CustomPacket component
NS_LOG_COMPONENT_DEFINE("CustomPacket");

using namespace ns3;

class CustomPacket : public ns3::Packet
{
public:
    CustomPacket() : ns3::Packet() {}

    uint32_t GetPacketId() const { return m_id; }
    void SetPacketId(uint32_t id) { m_id = id; }

    std::string GetPacketType() const { return m_type; }
    void SetPacketType(const std::string& type) { m_type = type; }

    ns3::Ipv4Address GetSourceAddress() const { return m_srcAddr; }
    void SetSourceAddress(ns3::Ipv4Address addr) { m_srcAddr = addr; }

    ns3::Ipv4Address GetDestinationAddress() const { return m_dstAddr; }
    void SetDestinationAddress(ns3::Ipv4Address addr) { m_dstAddr = addr; }

    std::string GetContent() const { return m_content; }
    void SetContent(const std::string& content) { m_content = content; }

private:
    uint32_t m_id;
    std::string m_type;
    ns3::Ipv4Address m_srcAddr;
    ns3::Ipv4Address m_dstAddr;
    std::string m_content;
};

int main(int argc, char* argv[])
{
    // Enable logging for CustomPacket
    LogComponentEnable("CustomPacket", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer nodes;
    nodes.Create(2);

    // Create point-to-point link
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer devices;
    devices = pointToPoint.Install(nodes);

    // Install internet stack
    InternetStackHelper internet;
    internet.Install(nodes);

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Create custom packet
    CustomPacket packet;
    packet.SetPacketId(1);
    packet.SetPacketType("Normal Packet");
    packet.SetSourceAddress(interfaces.GetAddress(0));
    packet.SetDestinationAddress(interfaces.GetAddress(1));
    packet.SetContent("Hello, NS3!");

    // Create sender application
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(interfaces.GetAddress(1), 9)); // Use UDP and port 9
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(1));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(10.0));

    // Create receiver application
OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(interfaces.GetAddress(1), 9)); // Use UDP and port 9
onoff.SetAttribute("PacketSize", UintegerValue(1024));
onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
onoff.SetAttribute("DataRate", StringValue("500kb/s"));

ApplicationContainer apps = onoff.Install(nodes.Get(0));
apps.Start(Seconds(1.0));
apps.Stop(Seconds(10.0));

// Receive and print messages
// Receive and print messages
Ptr<Socket> recvSocket = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
recvSocket->Bind(InetSocketAddress(interfaces.GetAddress(1), 9));

Address from; // Declare 'from' here

while (Simulator::Now() < Seconds(10.0))
{
    Ptr<Packet> packet = recvSocket->RecvFrom(from);
    if (packet) {
        std::cout << "Received message: " << packet->ToString() << std::endl;
    } else {
        Simulator::Stop(Seconds(10.0));
    }
}

Simulator::Destroy();



    // Create receiver application
    //OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(interfaces.GetAddress(1), 9)); // Use UDP and port 9
    //onoff.SetAttribute("PacketSize", UintegerValue(1024));
    //onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    //onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    //onoff.SetAttribute("DataRate", StringValue("500kb/s"));

    //ApplicationContainer apps = onoff.Install(nodes.Get(0));
    //apps.Start(Seconds(1.0));
    //apps.Stop(Seconds(10.0));

    // Run simulation
    //Simulator::Run();
    //Simulator::Destroy();

    // Display simulation status
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApp.Get(0));
    if (sink && sink->GetTotalRx() > 0)
    {
        std::cout << "Simulation successful. Packet received at receiver." << std::endl;
    }
    else
    {
        std::cout << "Simulation failed. No packets received at receiver." << std::endl;
    }

    return 0;
}
