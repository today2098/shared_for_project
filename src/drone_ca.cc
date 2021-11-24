/*
ドローンの衝突回避 (CA:Collision Avoidance) システム. 
*/

#include <bits/stdc++.h>
using namespace std;

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
using namespace ns3;

#define debug(x) std::cerr << "(L" << __LINE__ << ") " << #x << ": " << (x) << std::endl

NS_LOG_COMPONENT_DEFINE("Tmp");
string data_dir = "data/";

class NetSim {
    int numV;             // ノード数.
    Ptr<Node> *v;         // v[i]:=(i番目のノード).
    NodeContainer nodes;  // v[]のコンテナ.
    NetDeviceContainer devices;

    Vector basePosi;             // ベース座標.
    double dist;                 // ノード間距離 (m).
    double sensorRange;          // センサ範囲 (m).
    double velocity;             // 上昇速度 (m/s).
    vector<double> avoidTime;    // avoidTime[i]:=(v[i]が最後に回避した時間).
    vector<double> warningTime;  // warningTime[i]:=(v[i]が最後に警告した時間).
    vector<bool> catchWarning;   // cathWarning[i]:=(v[i]が警告を受信しているか否か).

    int numMV;             // 飛来物の数. 今は1個のみ!!!
    Ptr<Node> *mv;         // mv[i]:=(i番目の飛来物).
    NodeContainer movers;  // mv[]のコンテナ.

    vector<Vector> startMV;  // startMV[i]:=(mv[i]のスタート座標).
    vector<Vector> speedMV;  // speedMV[i]:=(mv[i]の速度).

    string phyMode;  // Wi-Fi物理層の伝送方式.

    bool connect;  // 警告システムの有無.
    int pktSize;
    int totalSent;      // システムの送信回数.
    int totalReceived;  // システムの受信回数.

    double interval;  // シミュレーション間隔 (s).
    double simStop;   // 終了時間 (s).
    bool verbose;

    InetSocketAddress SetSocketAddress(Ptr<Node> u, int port);
    InetSocketAddress SetBroadcastSocketAddress(Ptr<Node> u, int port);
    void ReceivePacket(Ptr<Socket> socket);
    void GenerateTraffic(Ptr<Socket> socket);
    void RunApplication(Ptr<Node> u, double deltatime);
    void ShowPosition(Ptr<Node> u);
    void ShowPositionC(Ptr<Node> u, double deltaTime);

public:
    // constructor.
    NetSim();

    void Configure(int argc, char *argv[]);  // command line options processing.
    void ConfigureSetDefault();              // configure default attributes.
    void CreateNetworkTopology();            // ノードを配置する.
    void CreateMovement();                   // 飛来物の軌道を作る.
    void ConfigureL2();                      // configure Data Link Layer.
    void ConfigureL3();                      // configure Network Layer.
    void ConfigureL4WithUDP();               // configure Transport Layer.
    void RunSimulation();
};

NetSim::NetSim() {
    numV = 25;  // 5*5.

    basePosi = Vector(100.0, 100.0, 10.0);
    dist = 50.0;
    sensorRange = 10.0;
    velocity = 4.0;
    avoidTime.assign(numV + 10, -1.0);
    warningTime.assign(numV + 10, -1.0);
    catchWarning.assign(numV + 10, false);

    numMV = 1;

    startMV = vector<Vector>({Vector(50.0, 150.0, 10.0)});
    speedMV = vector<Vector>({Vector(10.0, 0.0, 0.0)});

    phyMode = "DsssRate1Mbps";  // 直接拡散方式, 1Mbps.

    connect = true;  // システムの有無の比較検討!!!
    pktSize = 1024;
    totalSent = 0;
    totalReceived = 0;

    interval = 0.10;
    simStop = 30.0;
    verbose = false;
}

InetSocketAddress NetSim::SetSocketAddress(Ptr<Node> u, int port) {
    Ipv4InterfaceAddress adr = u->GetObject<Ipv4>()->GetAddress(1, 0);
    return InetSocketAddress(Ipv4Address(adr.GetLocal()), port);
}

InetSocketAddress NetSim::SetBroadcastSocketAddress(Ptr<Node> u, int port) {
    Ipv4InterfaceAddress adr = u->GetObject<Ipv4>()->GetAddress(1, 0);
    return InetSocketAddress(Ipv4Address(adr.GetBroadcast()), port);
}

void NetSim::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while((packet = socket->RecvFrom(from))) {
        if(packet->GetSize() > 0) {
            auto now = Simulator::Now().GetSeconds();
            int nodeId = socket->GetNode()->GetId();
            InetSocketAddress iaddr = InetSocketAddress::ConvertFrom(from);
            fprintf(stderr, "receive! (at: %.2lf, node: %d, size: %d, from: ", now, nodeId, packet->GetSize());
            cerr << "(" << iaddr.GetIpv4() << ", " << iaddr.GetPort() << "))" << endl;

            catchWarning[socket->GetNode()->GetId()] = true;
            totalReceived++;
        }
    }
}

void NetSim::GenerateTraffic(Ptr<Socket> socket) {
    socket->Send(Create<Packet>(pktSize));  // ???

    auto now = Simulator::Now().GetSeconds();
    int nodeId = socket->GetNode()->GetId();
    fprintf(stderr, "send! (at: %.2lf, node: %d)\n", now, nodeId);
    totalSent++;
}

void NetSim::ShowPosition(Ptr<Node> u) {
    int nodeId = u->GetId();
    auto mobilityModel = u->GetObject<MobilityModel>();
    auto posi = mobilityModel->GetPosition();
    auto speed = mobilityModel->GetVelocity();
    std::cerr << "At " << Simulator::Now().GetSeconds() << " node " << nodeId
              << ": Position(" << posi.x << ", " << posi.y << ", " << posi.z
              << ");   Speed(" << speed.x << ", " << speed.y << ", " << speed.z
              << ")" << std::endl;
}

void NetSim::ShowPositionC(Ptr<Node> u, double deltaTime) {
    ShowPosition(u);
    Simulator::Schedule(Seconds(deltaTime), &NetSim::ShowPositionC, this, u, deltaTime);
}

void NetSim::Configure(int argc, char *argv[]) {
    Time::SetResolution(Time::NS);

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "turn on all WifiNetDevice log components", verbose);
    cmd.Parse(argc, argv);

    cerr << "compleate NetSim::Configure()" << endl;
}

void NetSim::ConfigureSetDefault() {
    // Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
    // Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
    // Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    cerr << "compleate NetSim::ConfigureSetDefault()" << endl;
}

void NetSim::CreateNetworkTopology() {
    v = new Ptr<Node>[numV];
    for(int i = 0; i < numV; ++i) {
        v[i] = CreateObject<Node>();
        nodes.Add(v[i]);
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(basePosi.x),
                                  "MinY", DoubleValue(basePosi.y),
                                  "Z", DoubleValue(basePosi.z),
                                  "DeltaX", DoubleValue(dist),
                                  "DeltaY", DoubleValue(dist),
                                  "GridWidth", UintegerValue(5),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    // mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    cerr << "compleate NetSim::CreateNetworkTopology()" << endl;
}

void NetSim::CreateMovement() {
    mv = new Ptr<Node>[numMV];
    for(int i = 0; i < numMV; ++i) {
        mv[i] = CreateObject<Node>();
        movers.Add(mv[i]);
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(movers);
    for(int i = 0; i < numMV; ++i) {
        auto velocityMm = mv[i]->GetObject<ConstantVelocityMobilityModel>();
        velocityMm->SetPosition(startMV[i]);
        velocityMm->SetVelocity(speedMV[i]);
    }

    cerr << "compleate NetSim::CreateMovement()" << endl;
}

void NetSim::ConfigureL2() {
    // Wi-Fi関連の設定.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",        // 定常レート管理方式のWi-Fiマネージャー.
                                 "DataMode", StringValue(phyMode),      // データ通信速度.
                                 "ControlMode", StringValue(phyMode));  // 制御通信速度.

    // 無線チャンネルの設定.
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");  // 遅延モデル. 距離ベースの定常速度伝搬モデル.
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",       // 損失モデル. 対数距離伝搬損失モデル.
                                   "Exponent", DoubleValue(3.0),
                                   "ReferenceDistance", DoubleValue(1.0),
                                   "ReferenceLoss", DoubleValue(46.6777));

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.SetChannel(wifiChannel.Create());

    // Macレイヤーの設定.
    WifiMacHelper wifiMac;
    Ssid ssid = Ssid("MySSID");
    wifiMac.SetType("ns3::AdhocWifiMac",
                    "Ssid", SsidValue(ssid),
                    "QosSupported", BooleanValue(false));

    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    cerr << "compleate NetSim::ConfigureL2()" << endl;
}

void NetSim::ConfigureL3() {
    InternetStackHelper internet;
    AodvHelper aodv;  // use AODV routing protocol.
    internet.SetRoutingHelper(aodv);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifs = ipv4.Assign(devices);

    cerr << "compleate NetSim::ConfigureL3()" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void NetSim::ConfigureL4WithUDP() {// 通信部分!!!
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    // set receiver's socket.
    for(int i = 0; i < numV; ++i) {
        Ptr<Socket> destination = Socket::CreateSocket(v[i], tid);
        InetSocketAddress dstSocketAddr = SetSocketAddress(v[i], 2000);
        destination->Bind(dstSocketAddr);
        destination->SetRecvCallback(MakeCallback(&NetSim::ReceivePacket, this));
    }

    // set sender's socket.
    InetSocketAddress dstSocketAddr = SetSocketAddress(v[0], 2000);
    InetSocketAddress broadcastSocketAddr = SetBroadcastSocketAddress(v[0], 2000);
    debug(dstSocketAddr);
    debug(broadcastSocketAddr);
    for(int i = 0; i < numV; ++i) {
        Ptr<Socket> source = Socket::CreateSocket(v[i], tid);
        v[i]->AggregateObject(source);
        InetSocketAddress srcSocketAddr = SetSocketAddress(v[i], 3000);
        source->Bind(srcSocketAddr);
        source->SetAllowBroadcast(true);
        source->Connect(dstSocketAddr);  // send to v[0].
        //source->Connect(broadcastSocketAddr); // send to all. 上手くいかない.
    }

    cerr << "compleate NetSim::ConfigureL4WithUDP()" << endl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void NetSim::RunApplication(Ptr<Node> u, double deltaTime) {
    double now = Simulator::Now().GetSeconds();
    int nodeId = u->GetId();
    auto waypointMm = u->GetObject<WaypointMobilityModel>();
    auto posi = waypointMm->GetPosition();
    // auto speed = waypointMm->GetVelocity();

    auto warning = [&]() -> void {
        if(connect) GenerateTraffic(u->GetObject<Socket>());
        warningTime[nodeId] = now;
    };
    auto avoid = [&]() -> void {
        fprintf(stderr, "avoid! (at: %.2lf, node: %d, dist: %.2lf)\n", now, nodeId, dist);
        waypointMm->EndMobility();
        waypointMm->AddWaypoint(Waypoint(Seconds(now), posi));
        waypointMm->AddWaypoint(Waypoint(Seconds(now + abs(20.0 - posi.z) / velocity), Vector(posi.x, posi.y, 20.0)));
        avoidTime[nodeId] = now;
        catchWarning[nodeId] = false;
    };
    auto recover = [&]() -> void {
        fprintf(stderr, "recover! (at: %.2lf, node: %d, dist: %.2lf)\n", now, nodeId, dist);
        waypointMm->EndMobility();
        waypointMm->AddWaypoint(Waypoint(Seconds(now), posi));
        waypointMm->AddWaypoint(Waypoint(Seconds(now + abs(10.0 - posi.z) / velocity), Vector(posi.x, posi.y, 10.0)));
        avoidTime[nodeId] = -1.0;
    };

    double minDist = 1e9;
    for(int i = 0; i < numMV; ++i) {
        auto mvPosi = mv[i]->GetObject<MobilityModel>()->GetPosition();
        minDist = min(minDist, CalculateDistance(posi, mvPosi));
    }

    if(catchWarning[nodeId]) {
        if(avoidTime[nodeId] == -1.0) avoid();
    } else if(minDist <= sensorRange) {
        if(warningTime[nodeId] == -1.0) warning();
        if(avoidTime[nodeId] == -1.0) avoid();
    } else if(avoidTime[nodeId] != -1.0 and avoidTime[nodeId] <= now - 10.0) {
        recover();
    }

    //GenerateTraffic(u->GetObject<Socket>());
    Simulator::Schedule(Seconds(deltaTime), &NetSim::RunApplication, this, u, deltaTime);
}

void NetSim::RunSimulation() {
    Simulator::Stop(Seconds(simStop));

    for(int i = 0; i < numV; ++i) RunApplication(v[i], 0.01);
    for(int i = 5; i < 10; ++i) ShowPositionC(v[i], 0.5);
    for(int i = 0; i < numMV; ++i) ShowPositionC(mv[i], 0.5);

    Simulator::Run();
    Simulator::Destroy();

    debug(totalSent);
    debug(totalReceived);
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
    NetSim sim;
    sim.Configure(argc, argv);
    sim.ConfigureSetDefault();
    sim.CreateNetworkTopology();
    sim.CreateMovement();
    sim.ConfigureL2();
    sim.ConfigureL3();
    sim.ConfigureL4WithUDP();

    // string xf = data_dir + "tmp.xml";
    // AnimationInterface anim(xf);
    // anim.EnablePacketMetadata(true);

    sim.RunSimulation();

    return 0;
}
