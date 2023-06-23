/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
*   SIGNET lab.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <ns3/mmwave-sidelink-spectrum-phy.h>
#include <ns3/mmwave-vehicular-5g-net-device.h>
#include <ns3/mmwave-vehicular-5g-helper.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/mobility-module.h>
#include <ns3/applications-module.h>
#include <ns3/config.h>
#include <ns3/isotropic-antenna-model.h>
#include <ns3/spectrum-helper.h>
#include <ns3/mmwave-spectrum-value-helper.h>
#include <ns3/applications-module.h>
#include <ns3/internet-module.h>
#include <ns3/buildings-module.h>
#include <ns3/command-line.h>
#include <ns3/node-list.h>

#include <ns3/mmwave-point-to-point-epc-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/global-value.h>
#include <ns3/internet-module.h>
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/netanim-module.h>
#include <ns3/sqlite-output.h>
#include <ns3/stats-helper.h>
#include <ns3/parameters-config.h>
#include <ns3/core-module.h>

NS_LOG_COMPONENT_DEFINE ("Vehicular5G");

using namespace ns3;
using namespace millicar;

bool
AreOverlapping(Box a, Box b)
{
    return !((a.xMin > b.xMax) || (b.xMin > a.xMax) || (a.yMin > b.yMax) || (b.yMin > a.yMax));
}

bool
OverlapWithAnyPrevious(Box box, std::list<Box> m_previousBlocks)
{
    for (std::list<Box>::iterator it = m_previousBlocks.begin(); it != m_previousBlocks.end(); ++it)
    {
        if (AreOverlapping(*it, box))
        {
            return true;
        }
    }
    return false;
}

std::pair<Box, std::list<Box>>
GenerateBuildingBounds(double xArea,
                       double yArea,
                       double maxBuildSize,
                       std::list<Box> m_previousBlocks)
{
    Ptr<UniformRandomVariable> xMinBuilding = CreateObject<UniformRandomVariable>();
    xMinBuilding->SetAttribute("Min", DoubleValue(30));
    xMinBuilding->SetAttribute("Max", DoubleValue(xArea));

    NS_LOG_UNCOND("min " << 0 << " max " << xArea);

    Ptr<UniformRandomVariable> yMinBuilding = CreateObject<UniformRandomVariable>();
    yMinBuilding->SetAttribute("Min", DoubleValue(0));
    yMinBuilding->SetAttribute("Max", DoubleValue(yArea));

    NS_LOG_UNCOND("min " << 0 << " max " << yArea);

    Box box;
    uint32_t attempt = 0;
    do
    {
        NS_ASSERT_MSG(attempt < 100,
                      "Too many failed attempts to position non-overlapping buildings. Maybe area "
                      "too small or too many buildings?");
        box.xMin = xMinBuilding->GetValue();

        Ptr<UniformRandomVariable> xMaxBuilding = CreateObject<UniformRandomVariable>();
        xMaxBuilding->SetAttribute("Min", DoubleValue(box.xMin));
        xMaxBuilding->SetAttribute("Max", DoubleValue(box.xMin + maxBuildSize));
        box.xMax = xMaxBuilding->GetValue();

        box.yMin = yMinBuilding->GetValue();

        Ptr<UniformRandomVariable> yMaxBuilding = CreateObject<UniformRandomVariable>();
        yMaxBuilding->SetAttribute("Min", DoubleValue(box.yMin));
        yMaxBuilding->SetAttribute("Max", DoubleValue(box.yMin + maxBuildSize));
        box.yMax = yMaxBuilding->GetValue();

        ++attempt;
    } while (OverlapWithAnyPrevious(box, m_previousBlocks));

    NS_LOG_UNCOND("Building in coordinates (" << box.xMin << " , " << box.yMin << ") and ("
                                              << box.xMax << " , " << box.yMax
                                              << ") accepted after " << attempt << " attempts");
    m_previousBlocks.push_back(box);
    std::pair<Box, std::list<Box>> pairReturn = std::make_pair(box, m_previousBlocks);
    return pairReturn;
}

void PrintGnuplottableNodeListToFile (std::string filename);
void PrintGnuplottableUeListToFile(std::string filename);
void PrintGnuplottableEnbListToFile(std::string filename);
void PrintGnuplottableBuildingListToFile(std::string filename);

uint32_t g_rxPackets; // total number of received packets
uint32_t g_txPackets; // total number of transmitted packets

Time g_firstReceived; // timestamp of the first time a packet is received
Time g_lastReceived; // timestamp of the last received packet

static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
 g_rxPackets++;
 SeqTsHeader header;
//  SeqTsSizeHeader header;

 p->PeekHeader(header);
 uint64_t packetId = p->GetUid();

 *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds() << std::endl;

 if (g_rxPackets > 1)
 {

   g_lastReceived = Simulator::Now();
 }
 else
 {
   g_firstReceived = Simulator::Now();
 }
}

static ns3::GlobalValue g_maxXAxis(
    "maxXAxis",
    "The maximum X coordinate for the area in which to deploy the buildings",
    ns3::DoubleValue(150),
    ns3::MakeDoubleChecker<double>());
static ns3::GlobalValue g_maxYAxis(
    "maxYAxis",
    "The maximum Y coordinate for the area in which to deploy the buildings",
    ns3::DoubleValue(40),
    ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_numBuildingsBetweenMmWaveEnb(
    "numBlocks",
    "Number of buildings between MmWave eNB 1 and 2",
    ns3::UintegerValue(1),
    ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_e2TermIp ("e2TermIp", "The IP address of the RIC E2 termination",
                                    ns3::StringValue ("10.0.2.10"), ns3::MakeStringChecker ());

int main (int argc, char *argv[])
{
  // LogComponentEnableAll (LOG_PREFIX_ALL);
  // LogComponentEnable ("MmWaveMillicarHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveMillicarUeNetDevice", LOG_LEVEL_ALL);
  
  // LogComponentEnable ("Vehicular5G", LOG_LEVEL_ALL);

  // LogComponentEnable ("MmWaveSidelinkMac", LOG_LOGIC);
  // LogComponentEnable ("MmWaveSidelinkPhy", LOG_LEVEL_ALL); // LOG_DEBUG
  // LogComponentEnable ("MmWaveSidelinkSpectrumPhy", LOG_LEVEL_ALL);
  
  // LogComponentEnable ("E2Termination", LOG_LEVEL_ALL); 
  // LogComponentEnable ("MmWaveComponentCarrier", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveComponentCarrierEnb", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveMacScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable ("Socket", LOG_LEVEL_ALL);
  // LogComponentEnable ("PacketSink", LOG_LEVEL_ALL);
  // LogComponentEnable ("BulkSendApplication", LOG_LEVEL_ALL);
  // LogComponentEnable ("ApplicationContainer", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("LtePdcp", LOG_LEVEL_ALL);
    
  // This script creates two nodes moving at 20 m/s, placed at a distance of 10 m.
  // These nodes exchange packets through a UDP application,
  // and they communicate using a wireless channel.

  mmwave::ParametersConfig::EnableTraces();

  // system parameters
  std::list<Box> m_previousBlocks;
  bool harqEnabled = true;
  double bandwidth = 1e8; // bandwidth in Hz
  double frequency = 28e9; // the carrier frequency
  uint32_t numerology = 3; // the numerology

  // applications
  uint32_t packetSize = 1024; // UDP packet size in bytes
  uint32_t startTime = 50; // application start time in milliseconds
  uint32_t endTime = 2000; // application end time in milliseconds
  uint32_t interPacketInterval = 30; // interpacket interval in microseconds

  // mobility
  double speed = 10; // speed of the vehicles m/s
  double intraGroupDistance = 10; // distance between two vehicles belonging to the same group
  
  std::string scenario = "V2V-Urban";
  
  UintegerValue uintegerValue;
  DoubleValue doubleValue;
  StringValue stringValue;
  GlobalValue::GetValueByName ("e2TermIp", stringValue);
  std::string e2TermIp = stringValue.Get ();
  GlobalValue::GetValueByName("numBlocks", uintegerValue);
  uint32_t numBlocks = uintegerValue.Get();
  GlobalValue::GetValueByName("maxXAxis", doubleValue);
  double maxXAxis = doubleValue.Get();
  GlobalValue::GetValueByName("maxYAxis", doubleValue);
  double maxYAxis = doubleValue.Get();

  CommandLine cmd;
  cmd.AddValue ("bandwidth", "used bandwidth", bandwidth);
  cmd.AddValue ("iip", "inter packet interval, in microseconds", interPacketInterval);
  cmd.AddValue ("intraGroupDistance", "distance between two vehicles belonging to the same group, y-coord", intraGroupDistance);
  cmd.AddValue ("numerology", "set the numerology to use at the physical layer", numerology);
  cmd.AddValue ("frequency", "set the carrier frequency", frequency);
  cmd.AddValue ("scenario", "set the vehicular scenario", scenario);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveMillicarHelper::E2ModeNr", BooleanValue (false));

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (28));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));
  
  Config::SetDefault ("ns3::MmWaveMillicarHelper::BandwidthMillicar", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::NumerologyMillicar", UintegerValue (numerology));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::ChannelModelTypeMillicar", StringValue (scenario));
  
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (10)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (10)));

  Config::SetDefault ("ns3::MmWaveMillicarUeNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::SchedulingPatternOptionMillicar", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (500*1024));

  Config::SetDefault("ns3::MmWaveMillicarHelper::UseIdealRrc", BooleanValue(true));

  Config::SetDefault ("ns3::MmWaveMillicarHelper::E2TermIp", StringValue (e2TermIp));

  SQLiteOutput db("traces.db");

  mmwave::SinrReportStats sinrReportStats;
  mmwave::SinrReportStats allPairssinrReportStats;

  sinrReportStats.SetDb(&db, "sinrReportStats");
  allPairssinrReportStats.SetDb(&db, "allPairsReportStats");

  // create the nodes
  NodeContainer ueNodes;
  ueNodes.Create (8);
  // 5g part
  NodeContainer allEnbNodes, mmWaveEnbNodes, lteEnbNodes;
  allEnbNodes.Create(2);
  mmWaveEnbNodes.Add(allEnbNodes.Get(0));
  lteEnbNodes.Add(allEnbNodes.Get(1));

  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  MobilityHelper uemobility, uemobilityGroup1, uemobilityGroup2;

  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup1 = CreateObject<UniformDiscPositionAllocator> ();

  double rho = 20;
  double center_x = 55;
  double center_y = 65;

  uePositionAlloc->SetX (center_x);
  uePositionAlloc->SetY (center_y);
  uePositionAlloc->SetRho (rho);
  
  Ptr<UniformRandomVariable> speedRandomVariable = CreateObject<UniformRandomVariable> ();
  speedRandomVariable->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable->SetAttribute ("Max", DoubleValue (14.0));

  uemobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable), "Bounds",
                              RectangleValue (Rectangle (center_x-rho, center_x+rho, 
                              center_y-rho, center_y+rho)));
  
  // uemobilityGroup2 = uemobility;
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);

  // Install Mobility Model
  // Positions
  Vector mmw1Position = Vector(50, 70, 30);
  Vector mmw2Position = Vector(100, 70, 30);
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
  enbPositionAlloc->Add(mmw1Position);
  enbPositionAlloc->Add(mmw2Position);
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(enbPositionAlloc);
  enbmobility.Install(allEnbNodes);

  std::vector<Ptr<Building>> buildingVector;

  double maxBuildingSize = 20;

  // creating fixed position buildings
  Ptr<Building> building1, building2;
  building1 = Create<Building>();
  building2 = Create<Building>();
  Box box1, box2;
  box1.xMin = 10; box1.xMax = 40; box1.yMin = 10; box1.yMax = 40;
  box2.xMin = 50; box2.xMax = 80; box2.yMin = 10; box2.yMax = 40;
  double buildingHeight = 30;

  building1->SetBoundaries(Box(box1.xMin, box1.xMax, box1.yMin, box1.yMax, 0.0, buildingHeight));
  building2->SetBoundaries(Box(box2.xMin, box2.xMax, box2.yMin, box2.yMax, 0.0, buildingHeight));
  buildingVector.push_back(building1);
  buildingVector.push_back(building2);

  // constant position nodes
  MobilityHelper relayNodeMobility;
  relayNodeMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  relayNodeMobility.Install(ueNodes.Get (0));
  relayNodeMobility.Install(ueNodes.Get (3));
  // for simplicity we make fixed position for source and relaying node
  // source
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (45,20,1.5));
  // relay node
  ueNodes.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (45,52,1.5));

  MobilityHelper costVelocityMobility;
  costVelocityMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  costVelocityMobility.Install(ueNodes.Get (4));
  // destination node movement - is between buildings and 
  // at the beginning has los then it moves along y axis going in nLOS
  // at this instant the relay node is used as intermediate
  ueNodes.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (41, 50,  1.5));
  ueNodes.Get (4)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  // create and configure the helper
  Ptr<mmwave::MmWaveMillicarHelper> helper = CreateObject<mmwave::MmWaveMillicarHelper> ();
  helper->SetNumerologyMillicar (3);

  // configure 5g part 
  // helper->SetPathlossModelType("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  // helper->SetChannelConditionModelType("ns3::BuildingsChannelConditionModel");

  // set the number of antennas for both UEs and eNBs
  // helper->SetUePhasedArrayModelAttribute("NumColumns", UintegerValue(4));
  // helper->SetUePhasedArrayModelAttribute("NumRows", UintegerValue(4));
  // helper->SetEnbPhasedArrayModelAttribute("NumColumns", UintegerValue(8));
  // helper->SetEnbPhasedArrayModelAttribute("NumRows", UintegerValue(8));

  Ptr<mmwave::MmWavePointToPointEpcHelper> epcHelper = CreateObject<mmwave::MmWavePointToPointEpcHelper>();
  helper->SetEpcHelper(epcHelper);
  // helper->SetHarqEnabled(harqEnabled);
  helper->Initialize();

  NetDeviceContainer mmWaveEnbDevs = helper->InstallEnbDevice (mmWaveEnbNodes);
  // NetDeviceContainer lteEnbDevs = helper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer lteEnbDevs = helper->InstallEnbDevice (lteEnbNodes);
  NetDeviceContainer ueNetDev = helper->InstallUeDevice (ueNodes);

  Ptr<mmwave::MmWaveEnbNetDevice> firstEnbDev = DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(0));
  firstEnbDev->RegisterToMillicarUeTraces();
  helper->RegisterMillicarDevicesToEnb(ueNetDev, firstEnbDev);

  // Install the TCP/IP stack in the two nodes
  InternetStackHelper internet;
  internet.Install (ueNodes);
  // ipv4 stack
  // internet.Install (ueNodesGroup1);
  // internet.Install (ueNodesGroup2);

  Ipv4AddressHelper ipv4;

// Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode();
  Ptr<Node> mme = epcHelper->GetMmeNode();
  MobilityHelper otherNodesMobility;
  otherNodesMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  otherNodesMobility.Install(pgw);
  otherNodesMobility.Install(mme);
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  // InternetStackHelper internet;
  internet.Install(remoteHostContainer);
  otherNodesMobility.Install(remoteHost);

  double _sgw_x = 70;
  double _sgw_y = 70;
  double shift_x = 0;
  double shift_y = 0;
  uint32_t dist_coeff = 1;
  pgw->GetObject<MobilityModel>()->SetPosition(Vector((_sgw_x+ shift_x), _sgw_y+ shift_y, 0));
  mme->GetObject<MobilityModel>()->SetPosition(Vector((dist_coeff * 85 + shift_x), (dist_coeff * 75 + shift_y), 0));
  remoteHost->GetObject<MobilityModel>()->SetPosition(Vector((dist_coeff * 75+ shift_x), (dist_coeff * 75+ shift_y), 0));

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
  ipv4.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4.Assign(internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  // Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
  
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ueNodesIpv4InterfaceContainer = ipv4.Assign (ueNetDev);
  
  // Mandatory to install buildings helper even if there are no buildings, 
  // otherwise V2V-Urban scenario does not work
  BuildingsHelper::Install (allEnbNodes);
  BuildingsHelper::Install (ueNodes);

  // Need to pair the devices in order to create a correspondence between transmitter and receiver
  // and to populate the < IP addr, RNTI > map.
  helper->PairDevicesMillicar(ueNetDev);

  // Set the routing table
//   Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (ueNodes.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  NS_LOG_DEBUG("IPv4 Address node 0: " << ueNodes.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1: " << ueNodes.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject <mmwave::MmWaveAmc> (helper->GetConfigurationParametersMillicar());

  // setup the applications
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (0xFFFFFFFF));
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MicroSeconds (interPacketInterval)));
  Config::SetDefault ("ns3::UdpClient::PacketSize", UintegerValue (packetSize));

  // Add X2 interfaces
  // helper->AddX2Interface(lteEnbNodes, mmWaveEnbNodes);

  // Manual attachment
  NetDeviceContainer allNetDevs;
  allNetDevs.Add(mmWaveEnbDevs);
  allNetDevs.Add(lteEnbDevs);
  // helper->AttachToClosestEnb(ueNetDev, mmWaveEnbDevs);
  helper->AttachToClosestEnb(ueNetDev, allNetDevs);

  // create the applications
  uint32_t port = 4000;
  uint16_t otherPort = 50100;
  
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> _stream = asciiTraceHelper.CreateFileStream ("simple-one-stats.txt");
  // modified
  ApplicationContainer packetSinkApps;
  for (int _ind = 0; _ind < ueNodes.GetN(); ++_ind){
    //_ind+4
    Ptr<mmwave::MmWaveMillicarUeNetDevice> ueNrUeNetDev = ueNetDev.Get(_ind)->GetObject<mmwave::MmWaveMillicarUeNetDevice>();
    Vector _pos = ueNodes.Get (_ind)->GetObject<MobilityModel> ()->GetPosition ();
    NS_LOG_DEBUG("Packet sink  with address " << ueNodesIpv4InterfaceContainer.GetAddress(_ind)<< 
    " " << (otherPort + 2 * _ind) << " in node with rnti " << ueNrUeNetDev->GetRnti()<<
    " in position (" << _pos.x << ", " << _pos.y << ")" );


    // UdpClientHelper client (ueNodes.Get (_ind)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port + _ind);
    Address sinkLocalAddress(InetSocketAddress(ueNodesIpv4InterfaceContainer.GetAddress(_ind), otherPort + 2 * _ind));
    PacketSinkHelper sinkHelper (TcpSocketFactory::GetTypeId().GetName(), sinkLocalAddress);
    sinkHelper.SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
    sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    
    ApplicationContainer sinkHelperApplicationContainer = sinkHelper.Install (ueNodes.Get (_ind)); // _ind+4

    // install to remaining 5 nodes
    // apps.Add(client.Install (ueNodes.Get (_ind+4)));
    // _ind+4
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("./rx-n"  + std::to_string(ueNodes.Get(_ind)->GetId()) + ".txt");
    *stream->GetStream () << "time" << "\t" << "rnti" << "\t" << "packetsize" 
    << "\t" << "seqnum" << "\t" << "sendingtime" << "\t" << "x" << "\t" << "y" << std::endl;
    sinkHelperApplicationContainer.Get(0)->TraceConnectWithoutContext ("RxWithSeqTsSize", 
            MakeBoundCallback (&mmwave::ParametersConfig::RxTcp, stream, ueNodes.Get (_ind), // _ind+4
            ueNodesIpv4InterfaceContainer.Get(_ind).first)); // _ind+4
    packetSinkApps.Add(sinkHelperApplicationContainer); 
  }
  // end modification
  // set the application start/end time
  // apps.Start (MilliSeconds (startTime));

  uint32_t maxBytes = 10000000;
  std::vector<uint32_t> efsSize(uint32_t(ueNodes.GetN()), maxBytes);
  std::vector<Ptr<OutputStreamWrapper>> efsStreamFiles;
  ApplicationContainer bulkApps;
  for (int _ind = 0; _ind < ueNodes.GetN()/2; ++_ind){  
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream("./tx-n" + std::to_string(ueNodes.Get(_ind)->GetId()) + ".txt");
    efsStreamFiles.push_back(stream);
    // adding the header to the file
    Ptr<mmwave::MmWaveMillicarUeNetDevice> ueNrUeNetDev = ueNetDev.Get(_ind)->GetObject<mmwave::MmWaveMillicarUeNetDevice>();
    NS_LOG_DEBUG("Creating bulk in node with dest address " << ueNodesIpv4InterfaceContainer.GetAddress(_ind+4)<< 
    " " << (otherPort + 2 * (_ind+4)) << " rnti " << ueNrUeNetDev->GetRnti());
    // Address localAddress();
    BulkSendHelper ftp(TcpSocketFactory::GetTypeId().GetName(), InetSocketAddress(ueNodesIpv4InterfaceContainer.GetAddress(_ind+4), (otherPort + 2 * (_ind+4))));
    ftp.SetAttribute("MaxBytes", UintegerValue(efsSize.at(_ind)));
    ftp.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer bulkSenderApplicationContainer = ftp.Install(ueNodes.Get(_ind));
    Ptr<BulkSendApplication> bulkSenderApplication = bulkSenderApplicationContainer.Get(0)->GetObject<BulkSendApplication>();
    mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    bulkSenderStruct->bulkSenderApplication = bulkSenderApplication;
    bulkSenderStruct->servertcpIpv4 = ueNodesIpv4InterfaceContainer.Get(_ind).first;
    bulkSenderStruct->stream = stream;
    bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    *stream->GetStream () << "rnti" << "\t" << "imsi" << "\t" << "nodeid" << "\t"  << "cellid" << "\t" 
     << "remaining" << "\t" << "sent" << "\t" << "availTx" << "\t" << "time" << "\t" << "x" << "\t" 
     << "y"<< "\t" << "z" << std::endl;
    bulkSenderApplication->TraceConnectWithoutContext("Tx", 
        MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinator, 
        bulkSenderStruct , DynamicCast<LteEnbNetDevice>(lteEnbNodes.Get(0))));
    bulkApps.Add(bulkSenderApplicationContainer);

    // NS_LOG_DEBUG("Destination address " << ueNodes.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ()); 
  }
  
  packetSinkApps.Start (MilliSeconds (0));
  packetSinkApps.Stop (MilliSeconds (endTime));
  bulkApps.Start (MilliSeconds (0.0));
  bulkApps.Stop (MilliSeconds (endTime));
  NS_LOG_DEBUG("Creating sink apps");
  // end modification

  // traces ueNetDev
  for (auto ueDeviceIt = ueNetDev.Begin();ueDeviceIt != ueNetDev.End(); ++ueDeviceIt){
    Ptr<millicar::MmWaveSidelinkPhy> uePhy = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(*ueDeviceIt)->GetPhyMillicar();
    uePhy->TraceConnectWithoutContext("SlSinrReport",
											   MakeBoundCallback(&mmwave::EfStatsHelper::SinrReportCallback, &sinrReportStats));
    uePhy->TraceConnectWithoutContext("NotifyMillicarPairsSinr",
											   MakeBoundCallback(&mmwave::EfStatsHelper::AllPeersSinrReportCallback, &allPairssinrReportStats));
    Ptr<mmwave::MmWaveMillicarUeNetDevice> ueDev = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(*ueDeviceIt);
    // through this relay all traffic between 1-5 should pass through 6
    // NS_LOG_DEBUG("Setting test relay");
    
  }
  
  Ptr<mmwave::MmWaveMillicarUeNetDevice> relayDev = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(ueNetDev.Get(3));
  Simulator::Schedule (Seconds (0.5), &mmwave::MmWaveMillicarUeNetDevice::TestRelay,
                                              relayDev, 1, 5, 4);

  

  // PrintGnuplottableNodeListToFile ("scenario.txt");
  PrintGnuplottableBuildingListToFile("buildings.txt");
  PrintGnuplottableUeListToFile("ues.txt");
  PrintGnuplottableEnbListToFile("enbs.txt");
  // Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  // lteHelper->Initialize ();
  // lteHelper->EnablePhyTraces ();
  // lteHelper->EnableMacTraces ();

  // params.outputDir + params.simTag + 
  AnimationInterface *anim = new AnimationInterface("./elephant-flow-animation.xml");

  for (uint32_t i = 0; i < mmWaveEnbNodes.GetN(); ++i)
  {
    anim->UpdateNodeDescription(mmWaveEnbNodes.Get(i), "GNB"); // Optional
    anim->UpdateNodeColor(mmWaveEnbNodes.Get(i), 255, 255, 0); // Optional
  }

  for (uint32_t i = 0; i < lteEnbNodes.GetN(); ++i)
  {
    anim->UpdateNodeDescription(lteEnbNodes.Get(i), "LTE"); // Optional
    anim->UpdateNodeColor(lteEnbNodes.Get(i), 255, 255, 0); // Optional
  }
  // Updating the ue into blue color
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    anim->UpdateNodeDescription(ueNodes.Get(i), "UE"); // Optional
    anim->UpdateNodeColor(ueNodes.Get(i), 0, 255, 0);  // Optional
    anim->UpdateNodeSize(ueNodes.Get(i), 3, 3);
  }

  for (uint32_t i = 0; i < remoteHostContainer.GetN(); ++i)
  {
    anim->UpdateNodeDescription(remoteHostContainer.Get(i), "RH"); // Optional
    anim->UpdateNodeColor(remoteHostContainer.Get(i), 0, 0, 255);  // Optional
  }
  anim->UpdateNodeDescription(pgw, "PGW");
  anim->UpdateNodeColor(pgw, 0, 255, 255);

  anim->UpdateNodeDescription(mme, "MME");
  anim->UpdateNodeColor(mme, 200, 255, 255);

  // stop ip since we need to see only the layout
  // anim->SetStopTime(MilliSeconds(50));
  
  Simulator::Stop (MilliSeconds (endTime + 2000));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "----------- Statistics -----------" << std::endl;
  std::cout << "Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  std::cout << "Packets received:\t" << g_rxPackets << std::endl;
  std::cout << "Average Throughput:\t" << (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6 << " Mbps" << std::endl;

  return 0;
}

void
PrintGnuplottableNodeListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  outFile << "set xrange [-200:200]; set yrange [-200:200]" << std::endl;
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<mmwave::MmWaveMillicarUeNetDevice> vdev = node->GetDevice (j)->GetObject <mmwave::MmWaveMillicarUeNetDevice> ();
          if (vdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << vdev->GetMacMillicar ()->GetRnti ()
                      << "\" at "<< pos.x << "," << pos.y << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 7 ps 0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;

              // Simulator::Schedule (Seconds (1), &PrintHelper::UpdateGnuplottableNodeListToFile, filename, node);
            }
        }
    }
    
  uint32_t index = 0;
  for (BuildingList::Iterator it = BuildingList::Begin (); it != BuildingList::End (); ++it)
    {
      ++index;
      Box box = (*it)->GetBoundaries ();
      outFile << "set object " << index
              << " rect from " << box.xMin  << "," << box.yMin
              << " to "   << box.xMax  << "," << box.yMax
              << std::endl;
    }
}

void
PrintGnuplottableUeListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteUeNetDevice> uedev = node->GetDevice(j)->GetObject<LteUeNetDevice>();
            Ptr<mmwave::MmWaveUeNetDevice> mmuedev = node->GetDevice(j)->GetObject<mmwave::MmWaveUeNetDevice>();
            Ptr<mmwave::McUeNetDevice> mcuedev = node->GetDevice(j)->GetObject<mmwave::McUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << uedev->GetImsi() << "\" at " << pos.x << "," << pos.y
                        << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                           "0.3 lc rgb \"black\" offset 0,0"
                        << std::endl;
            }
            else if (mmuedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << mmuedev->GetImsi() << "\" at " << pos.x << "," << pos.y
                        << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                           "0.3 lc rgb \"black\" offset 0,0"
                        << std::endl;
            }
            else if (mcuedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << mcuedev->GetImsi() << "\" at " << pos.x << "," << pos.y
                        << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                           "0.3 lc rgb \"black\" offset 0,0"
                        << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteEnbNetDevice> enbdev = node->GetDevice(j)->GetObject<LteEnbNetDevice>();
            Ptr<mmwave::MmWaveEnbNetDevice> mmdev = node->GetDevice(j)->GetObject<mmwave::MmWaveEnbNetDevice>();
            if (enbdev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << enbdev->GetCellId() << "\" at " << pos.x << ","
                        << pos.y
                        << " left font \"Helvetica,8\" textcolor rgb \"blue\" front  point pt 4 ps "
                           "0.3 lc rgb \"blue\" offset 0,0"
                        << std::endl;
            }
            else if (mmdev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << mmdev->GetCellId() << "\" at " << pos.x << "," << pos.y
                        << " left font \"Helvetica,8\" textcolor rgb \"red\" front  point pt 4 ps "
                           "0.3 lc rgb \"red\" offset 0,0"
                        << std::endl;
            }
        }
    }
}

void
PrintGnuplottableBuildingListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    uint32_t index = 0;
    for (BuildingList::Iterator it = BuildingList::Begin(); it != BuildingList::End(); ++it)
    {
        ++index;
        Box box = (*it)->GetBoundaries();
        outFile << "set object " << index << " rect from " << box.xMin << "," << box.yMin << " to "
                << box.xMax << "," << box.yMax << " front fs empty " << std::endl;
    }
}
