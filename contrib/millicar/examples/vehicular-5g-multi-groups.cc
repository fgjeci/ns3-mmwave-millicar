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
#include <sys/stat.h>
#include <ns3/send-packet-stats.h>

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
                                    ns3::StringValue ("10.0.3.10"), ns3::MakeStringChecker ());

int main (int argc, char *argv[])
{   
  // This script creates two nodes moving at 20 m/s, placed at a distance of 10 m.
  // These nodes exchange packets through a UDP application,
  // and they communicate using a wireless channel.

  mmwave::ParametersConfig::EnableTraces();

  // system parameters
  bool harqEnabled = true;
  double bandwidth = 1e8; // bandwidth in Hz
  double frequency = 28e9; // the carrier frequency
  uint32_t numerology = 3; // the numerology

  // applications
  uint32_t packetSize = 1024; // UDP packet size in bytes
  uint32_t startTime = 50; // application start time in milliseconds
  uint32_t endTime = 10000; // application end time in milliseconds
  uint32_t interPacketInterval = 100; // interpacket interval in microseconds

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

  std::string tracesPath = "./";
  double relayTime = 0;
  double txpower = 0;
  std::string ltePlmnId = "111";
  uint16_t e2startingPort = 38470;
  bool isXappEnabled = false;

  CommandLine cmd;
  cmd.AddValue ("bandwidth", "used bandwidth", bandwidth);
  cmd.AddValue ("iip", "inter packet interval, in microseconds", interPacketInterval);
  cmd.AddValue ("intraGroupDistance", "distance between two vehicles belonging to the same group, y-coord", intraGroupDistance);
  cmd.AddValue ("numerology", "set the numerology to use at the physical layer", numerology);
  cmd.AddValue ("frequency", "set the carrier frequency", frequency);
  cmd.AddValue ("scenario", "set the vehicular scenario", scenario);
  cmd.AddValue ("tracesPath", "set the path where logs are stored", tracesPath);
  cmd.AddValue ("relayTime", "set the time when to schedule the relay", relayTime);
  cmd.AddValue ("transmitPowerdBm", "Transmit power of ues", txpower);
  cmd.AddValue ("ltePlmnId", "Lte plmn id, shall be used to distinguish simulation", ltePlmnId);
  cmd.AddValue ("e2StartingPort", "starting port number for the gnb e2 termination; destination is same", e2startingPort);
  cmd.AddValue ("isXappEnabled", "Define if the simulation has the support of Xapp", isXappEnabled);  
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveMillicarHelper::E2ModeNr", BooleanValue (isXappEnabled));

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

  Config::SetDefault ("ns3::MmWaveMillicarHelper::UseIdealRrc", BooleanValue(true));

  // setting plmn id 
  Config::SetDefault ("ns3::MmWaveMillicarHelper::E2TermIp", StringValue (e2TermIp));
  Config::SetDefault ("ns3::MmWaveSidelinkPhy::TxPower", DoubleValue (txpower));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::PlmnId", StringValue (ltePlmnId));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::PlmnId", StringValue (ltePlmnId));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::E2LocalPort", UintegerValue (e2startingPort));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (0.01));
  

  if (tracesPath.find("/", 0) == std::string::npos){
    // there is only the name as sim tag
    tracesPath = "./" + tracesPath;
  }

  if (tracesPath.rfind("/", 0) != tracesPath.size()-1){
    // there is only the name as sim tag
    tracesPath = tracesPath + "/";
  }

  // create dir if it does not exist
  struct stat simTagDirInfo;
  if (stat((tracesPath).c_str(), &simTagDirInfo) != 0){
    const int dir_err = mkdir((tracesPath).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
    {
      exit(1);
    }
  }

  // set traces path for enb
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::TracesPath", StringValue (tracesPath));
  Config::SetDefault ("ns3::MmWaveMillicarHelper::TracesPath", StringValue (tracesPath));

  SQLiteOutput db(tracesPath+ "traces.db");
  mmwave::SinrReportStats sinrReportStats;
  mmwave::SinrReportStats allPairssinrReportStats;
  mmwave::SendPacketStats sendPacketStats;
  mmwave::SendPacketStats relayPacketStats;

  sinrReportStats.SetDb(&db, "sinrReportStats");
  allPairssinrReportStats.SetDb(&db, "allPairsReportStats");
  sendPacketStats.SetDb(&db, "sendPacketsReportStats");
  relayPacketStats.SetDb(&db, "relayPacketsReportStats");

  // create the nodes
  NodeContainer ueNodes, ueNodesGroup1, ueNodesGroup2, ueNodesGroup3, ueNodesGroup4, ueNodesGroup5;
  ueNodes.Create (8);
  ueNodesGroup1.Create(8);
  ueNodesGroup2.Create(8);
  ueNodesGroup3.Create(8);
  ueNodesGroup4.Create(8);
  ueNodesGroup5.Create(8);
  // 5g part
  NodeContainer allEnbNodes, mmWaveEnbNodes, lteEnbNodes;
  allEnbNodes.Create(2);
  mmWaveEnbNodes.Add(allEnbNodes.Get(0));
  // lteEnbNodes.Add(allEnbNodes.Get(1));

  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  MobilityHelper uemobility, uemobilityGroup1, uemobilityGroup2, uemobilityGroup3, uemobilityGroup4, uemobilityGroup5;

  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup1 = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup2 = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup3 = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup4 = CreateObject<UniformDiscPositionAllocator> ();
  Ptr<UniformDiscPositionAllocator> uePositionAllocGroup5 = CreateObject<UniformDiscPositionAllocator> ();

  double rho = 20;
  double center_x = 55;
  double center_y = 60;

  uePositionAlloc->SetX (center_x);
  uePositionAlloc->SetY (center_y);
  uePositionAlloc->SetZ(1.5);
  uePositionAlloc->SetRho (rho);
  
  Ptr<UniformRandomVariable> speedRandomVariable = CreateObject<UniformRandomVariable> ();
  speedRandomVariable->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable->SetAttribute ("Max", DoubleValue (4.0));

  uemobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable), "Bounds",
                              RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              center_y-3*rho, center_y+3*rho)));
  
  // uemobilityGroup2 = uemobility;
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);

  
  rho = 20;
  center_x = 90;
  center_y = 60;
  uePositionAllocGroup1->SetX(center_x);
  uePositionAllocGroup1->SetY(center_y);
  uePositionAllocGroup1->SetZ(1.5);
  uePositionAllocGroup1->SetRho(rho);

  Ptr<UniformRandomVariable> speedRandomVariable1 = CreateObject<UniformRandomVariable> ();
  speedRandomVariable1->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable1->SetAttribute ("Max", DoubleValue (4.0));

  uemobilityGroup1.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable1), "Bounds",
                              RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              center_y-3*rho, center_y+3*rho)));

  uemobilityGroup1.SetPositionAllocator(uePositionAllocGroup1);
  uemobilityGroup1.Install (ueNodesGroup1);

  // group 2 
  rho = 20;
  center_x = 10;
  center_y = 60;
  uePositionAllocGroup2->SetX(center_x);
  uePositionAllocGroup2->SetY(center_y);
  uePositionAllocGroup2->SetZ(1.5);
  uePositionAllocGroup2->SetRho(rho);

  Ptr<UniformRandomVariable> speedRandomVariable2 = CreateObject<UniformRandomVariable> ();
  speedRandomVariable2->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable2->SetAttribute ("Max", DoubleValue (4.0));

  uemobilityGroup2.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable2), "Bounds",
                              RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              center_y-3*rho, center_y+3*rho)));

  uemobilityGroup2.SetPositionAllocator(uePositionAllocGroup2);
  uemobilityGroup2.Install (ueNodesGroup2);

  // group 3

  rho = 20;
  center_x = 40;
  center_y = 60;
  uePositionAllocGroup3->SetX(center_x);
  uePositionAllocGroup3->SetY(center_y);
  uePositionAllocGroup3->SetZ(1.5);
  uePositionAllocGroup3->SetRho(rho);

  Ptr<UniformRandomVariable> speedRandomVariable3 = CreateObject<UniformRandomVariable> ();
  speedRandomVariable3->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable3->SetAttribute ("Max", DoubleValue (4.0));

  uemobilityGroup3.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable3), "Bounds",
                              RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              center_y-3*rho, center_y+3*rho)));

  uemobilityGroup3.SetPositionAllocator(uePositionAllocGroup3);
  uemobilityGroup3.Install (ueNodesGroup3);

  // group 4
  rho = 5;
  center_x = 45;
  center_y = 90;
  Ptr<RandomBoxPositionAllocator> randomBoxPosition4 = CreateObject<RandomBoxPositionAllocator>();
  Ptr<UniformRandomVariable> randomBoxPositionX4 = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> randomBoxPositionY4 = CreateObject<UniformRandomVariable>();
  Ptr<ConstantRandomVariable> constBoxPositionZ4 = CreateObject<ConstantRandomVariable>();
  constBoxPositionZ4->SetAttribute("Constant", DoubleValue(1.5));
  randomBoxPositionX4->SetAttribute ("Min", DoubleValue (40));
  randomBoxPositionX4->SetAttribute ("Max", DoubleValue (50));
  randomBoxPositionY4->SetAttribute ("Min", DoubleValue (50));
  randomBoxPositionY4->SetAttribute ("Max", DoubleValue (100));
  randomBoxPosition4->SetX(randomBoxPositionX4);
  randomBoxPosition4->SetY(randomBoxPositionY4);
  randomBoxPosition4->SetZ(constBoxPositionZ4);

  // randomBoxPosition->SetX();
  uePositionAllocGroup4->SetX(center_x);
  uePositionAllocGroup4->SetY(center_y);
  uePositionAllocGroup4->SetZ(1.5);
  uePositionAllocGroup4->SetRho(rho);

  Ptr<UniformRandomVariable> speedRandomVariable4 = CreateObject<UniformRandomVariable> ();
  speedRandomVariable4->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable4->SetAttribute ("Max", DoubleValue (4.0));

  uemobilityGroup4.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable4), "Bounds",
                              // RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              // center_y-3*rho, center_y+3*rho))
                              RectangleValue (Rectangle (20, 70, 40, 120))
                              );

  // uemobilityGroup4.SetPositionAllocator(uePositionAllocGroup4);
  uemobilityGroup4.SetPositionAllocator(randomBoxPosition4);
  uemobilityGroup4.Install (ueNodesGroup4);
  
  // group 5
  rho = 5;
  center_x = 45;
  center_y = 20;

  Ptr<RandomBoxPositionAllocator> randomBoxPosition5 = CreateObject<RandomBoxPositionAllocator>();
  Ptr<UniformRandomVariable> randomBoxPositionX5 = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> randomBoxPositionY5 = CreateObject<UniformRandomVariable>();
  Ptr<ConstantRandomVariable> constBoxPositionZ5 = CreateObject<ConstantRandomVariable>();
  constBoxPositionZ5->SetAttribute("Constant", DoubleValue(1.5));
  randomBoxPositionX5->SetAttribute ("Min", DoubleValue (40));
  randomBoxPositionX5->SetAttribute ("Max", DoubleValue (50));
  randomBoxPositionY5->SetAttribute ("Min", DoubleValue (0));
  randomBoxPositionY5->SetAttribute ("Max", DoubleValue (70));
  randomBoxPosition5->SetX(randomBoxPositionX5);
  randomBoxPosition5->SetY(randomBoxPositionY5);
  randomBoxPosition5->SetZ(constBoxPositionZ5);

  uePositionAllocGroup5->SetX(center_x);
  uePositionAllocGroup5->SetY(center_y);
  uePositionAllocGroup5->SetZ(1.5);
  uePositionAllocGroup5->SetRho(rho);

  Ptr<UniformRandomVariable> speedRandomVariable5 = CreateObject<UniformRandomVariable> ();
  speedRandomVariable5->SetAttribute ("Min", DoubleValue (2.0));
  speedRandomVariable5->SetAttribute ("Max", DoubleValue (4.0));

  uemobilityGroup5.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                              PointerValue (speedRandomVariable5), "Bounds",
                              // RectangleValue (Rectangle (center_x-3*rho, center_x+3*rho, 
                              // center_y-3*rho, center_y+3*rho))
                              RectangleValue (Rectangle (20, 70, 0, 80))
                              );

  // uemobilityGroup5.SetPositionAllocator(uePositionAllocGroup5);
  uemobilityGroup5.SetPositionAllocator(randomBoxPosition5);
  uemobilityGroup5.Install (ueNodesGroup5);

  // setting the nodes with blockage
  MobilityHelper costVelocityMobility;
  costVelocityMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  costVelocityMobility.Install(ueNodes.Get (0));
  costVelocityMobility.Install(ueNodesGroup1.Get (0));
  costVelocityMobility.Install(ueNodesGroup2.Get (0));
  costVelocityMobility.Install(ueNodesGroup3.Get (0));
  costVelocityMobility.Install(ueNodesGroup4.Get (0));
  costVelocityMobility.Install(ueNodesGroup5.Get (0));
  
  // setting constant position intermediate nodes
  // constant position nodes
  MobilityHelper relayNodeMobility;
  relayNodeMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  relayNodeMobility.Install(ueNodes.Get (1));
  relayNodeMobility.Install(ueNodesGroup1.Get (1));
  relayNodeMobility.Install(ueNodesGroup2.Get (1));
  relayNodeMobility.Install(ueNodesGroup3.Get (1));
  relayNodeMobility.Install(ueNodesGroup4.Get (1));
  relayNodeMobility.Install(ueNodesGroup5.Get (1));

  // source
  // ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (90,90,1.5));
  // ueNodesGroup1.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (45,80,1.5));
  // ueNodesGroup2.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (45,40,1.5));
  // ueNodesGroup3.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (5,40,1.5));
  // ueNodesGroup4.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (45,110,1.5));
  // ueNodesGroup5.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (45,5,1.5));
  // setting speed
  // ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));
  // ueNodesGroup1.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));
  // ueNodesGroup2.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, -speed, 0));
  // ueNodesGroup3.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, -speed, 0));
  // ueNodesGroup4.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));
  // ueNodesGroup5.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));
  // destination
  // ueNodes.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (65,65,1.5));
  // ueNodesGroup1.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (80,75,1.5));
  // ueNodesGroup2.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (25,45,1.5));
  // ueNodesGroup3.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (25,55,1.5));
  // ueNodesGroup4.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (45,90,1.5));
  // ueNodesGroup5.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (45,20,1.5));
  // intermediate 
  // ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (90,65,1.5));
  // ueNodesGroup1.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (45,60,1.5));
  // ueNodesGroup2.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (45,45,1.5));
  // ueNodesGroup3.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (5,55,1.5));
  // ueNodesGroup4.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (45,105,1.5));
  // ueNodesGroup5.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (45,10,1.5));
  
  // relay


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
  Ptr<Building> building1, building2, building3, building4;
  building1 = Create<Building>();
  building2 = Create<Building>();
  building3 = Create<Building>();
  building4 = Create<Building>();
  Box box1, box2, box3, box4;
  box1.xMin = 10; box1.xMax = 40; box1.yMin = 10; box1.yMax = 40;
  box2.xMin = 50; box2.xMax = 80; box2.yMin = 10; box2.yMax = 40;
  box3.xMin = 10; box3.xMax = 40; box3.yMin = 80; box3.yMax = 100;
  box4.xMin = 50; box4.xMax = 80; box4.yMin = 80; box4.yMax = 100;
  double buildingHeight = 30;

  building1->SetBoundaries(Box(box1.xMin, box1.xMax, box1.yMin, box1.yMax, 0.0, buildingHeight));
  building2->SetBoundaries(Box(box2.xMin, box2.xMax, box2.yMin, box2.yMax, 0.0, buildingHeight));
  building3->SetBoundaries(Box(box3.xMin, box3.xMax, box3.yMin, box3.yMax, 0.0, buildingHeight));
  building4->SetBoundaries(Box(box4.xMin, box4.xMax, box4.yMin, box4.yMax, 0.0, buildingHeight));
  building1->SetBuildingType(Building::Residential);
  building2->SetBuildingType(Building::Residential);
  building3->SetBuildingType(Building::Residential);
  building4->SetBuildingType(Building::Residential);
  building1->SetExtWallsType(Building::ConcreteWithWindows);
  building2->SetExtWallsType(Building::ConcreteWithWindows);
  building3->SetExtWallsType(Building::ConcreteWithWindows);
  building4->SetExtWallsType(Building::ConcreteWithWindows);
  buildingVector.push_back(building1);
  buildingVector.push_back(building2);
  buildingVector.push_back(building3);
  buildingVector.push_back(building4);

  // Mandatory to install buildings helper even if there are no buildings, 
  // otherwise V2V-Urban scenario does not work
  BuildingsHelper::Install (allEnbNodes);
  BuildingsHelper::Install (ueNodes);
  BuildingsHelper::Install (ueNodesGroup1);
  BuildingsHelper::Install (ueNodesGroup2);
  BuildingsHelper::Install (ueNodesGroup3);
  BuildingsHelper::Install (ueNodesGroup4);
  BuildingsHelper::Install (ueNodesGroup5);

  
  // create and configure the helper
  Ptr<mmwave::MmWaveMillicarHelper> helper = CreateObject<mmwave::MmWaveMillicarHelper> ();
  helper->SetNumerologyMillicar (3);

  // configure 5g part 
  // helper->SetPathlossModelType("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  // helper->SetChannelConditionModelType("ns3::BuildingsChannelConditionModel");

  Ptr<mmwave::MmWavePointToPointEpcHelper> epcHelper = CreateObject<mmwave::MmWavePointToPointEpcHelper>();
  helper->SetEpcHelper(epcHelper);
  // helper->SetHarqEnabled(harqEnabled);
  helper->Initialize();

  NetDeviceContainer mmWaveEnbDevs = helper->InstallEnbDevice (mmWaveEnbNodes);
  // NetDeviceContainer lteEnbDevs = helper->InstallLteEnbDevice (lteEnbNodes);
  // NetDeviceContainer lteEnbDevs = helper->InstallEnbDevice (lteEnbNodes);
  NetDeviceContainer ueNetDev = helper->InstallUeDevice (ueNodes);
  // other groups
  NetDeviceContainer ueNetDevGroup1 = helper->InstallUeDevice (ueNodesGroup1);
  NetDeviceContainer ueNetDevGroup2 = helper->InstallUeDevice (ueNodesGroup2);
  NetDeviceContainer ueNetDevGroup3 = helper->InstallUeDevice (ueNodesGroup3);
  NetDeviceContainer ueNetDevGroup4 = helper->InstallUeDevice (ueNodesGroup4);
  NetDeviceContainer ueNetDevGroup5 = helper->InstallUeDevice (ueNodesGroup5);
  

  Ptr<mmwave::MmWaveEnbNetDevice> firstEnbDev = DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(0));
  firstEnbDev->RegisterToMillicarUeTraces();
  helper->RegisterMillicarDevicesToEnb(ueNetDev, firstEnbDev);

  helper->RegisterMillicarDevicesToEnb(ueNetDevGroup1, firstEnbDev);
  helper->RegisterMillicarDevicesToEnb(ueNetDevGroup2, firstEnbDev);
  helper->RegisterMillicarDevicesToEnb(ueNetDevGroup3, firstEnbDev);
  helper->RegisterMillicarDevicesToEnb(ueNetDevGroup4, firstEnbDev);
  helper->RegisterMillicarDevicesToEnb(ueNetDevGroup5, firstEnbDev);

  // test message creation
  // firstEnbDev->TestMessageCreation();
  // NS_FATAL_ERROR ("Error");

  // Install the TCP/IP stack in the two nodes
  InternetStackHelper internet;
  internet.Install (ueNodes);
  // ipv4 stack
  internet.Install (ueNodesGroup1);
  internet.Install (ueNodesGroup2);
  internet.Install (ueNodesGroup3);
  internet.Install (ueNodesGroup4);
  internet.Install (ueNodesGroup5);

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
  Ipv4InterfaceContainer ueNodesGroup1Ipv4InterfaceContainer = ipv4.Assign (ueNetDevGroup1);
  Ipv4InterfaceContainer ueNodesGroup2Ipv4InterfaceContainer = ipv4.Assign (ueNetDevGroup2);
  Ipv4InterfaceContainer ueNodesGroup3Ipv4InterfaceContainer = ipv4.Assign (ueNetDevGroup3);
  Ipv4InterfaceContainer ueNodesGroup4Ipv4InterfaceContainer = ipv4.Assign (ueNetDevGroup4);
  Ipv4InterfaceContainer ueNodesGroup5Ipv4InterfaceContainer = ipv4.Assign (ueNetDevGroup5);

  // Need to pair the devices in order to create a correspondence between transmitter and receiver
  // and to populate the < IP addr, RNTI > map.
  helper->PairDevicesMillicar(ueNetDev);

  helper->PairDevicesMillicar(ueNetDevGroup1);
  helper->PairDevicesMillicar(ueNetDevGroup2);
  helper->PairDevicesMillicar(ueNetDevGroup3);
  helper->PairDevicesMillicar(ueNetDevGroup4);
  helper->PairDevicesMillicar(ueNetDevGroup5);

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
  // allNetDevs.Add(lteEnbDevs);
  // helper->AttachToClosestEnb(ueNetDev, mmWaveEnbDevs);
  helper->AttachToClosestEnb(ueNetDev, allNetDevs);
  helper->AttachToClosestEnb(ueNetDevGroup1, allNetDevs);
  helper->AttachToClosestEnb(ueNetDevGroup2, allNetDevs);
  helper->AttachToClosestEnb(ueNetDevGroup3, allNetDevs);
  helper->AttachToClosestEnb(ueNetDevGroup4, allNetDevs);
  helper->AttachToClosestEnb(ueNetDevGroup5, allNetDevs);

  // create the applications
  uint32_t port = 4000;
  uint16_t otherPort = 50100;
  
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> _stream = asciiTraceHelper.CreateFileStream (tracesPath+"simple-one-stats.txt");
  *_stream->GetStream () << "rxtime" << "\t" << "rnti" << "\t" << "size" 
                        << "\t" << "seqnum" << "\t" << "txtime"
                        << "\t" << "x" << "\t" << "y" << std::endl;

  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream(tracesPath+"tx-upds.txt");
  *stream->GetStream () << "rnti" << "\t" << "imsi" << "\t" << "nodeid" << "\t"  << "cellid" << "\t" 
     << "seqNum" << "\t" << "sent" << "\t" << "txTimestamp" << "\t" << "time"
     << "\t" << "x" << "\t" << "y" << "\t" << "z" << std::endl;
    NS_LOG_DEBUG("Create bulk sender struct");
    
  // modified
  ApplicationContainer packetSinkApps, echoApps;
  uint32_t maxBytes = 10000000;
  std::vector<uint32_t> efsSize(uint32_t(ueNodes.GetN()), maxBytes);
  std::vector<Ptr<OutputStreamWrapper>> efsStreamFiles;
  ApplicationContainer bulkApps, updApps;

  for (int _ind = 0; _ind < ueNodes.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 1; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodes.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    packetSinkApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodes.Get (_ind), // _ind+4
            ueNodesIpv4InterfaceContainer.Get(_ind).first));
    
  }

  for (int _ind = 0; _ind < ueNodes.GetN()/2; ++_ind){
  // for (int _ind = 0; _ind < 1; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodes.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodes.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodes.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    bulkApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesIpv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))
    //     // ,&sendPacketStats
    //     ));
  }

  for (int _ind = 0; _ind < ueNodesGroup1.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 0; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodesGroup1.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    echoApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodesGroup1.Get (_ind), // _ind+4
            ueNodesGroup1Ipv4InterfaceContainer.Get(_ind).first));
  }


  for (int _ind = 0; _ind < ueNodesGroup1.GetN()/2; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodesGroup1.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodesGroup1.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodesGroup1.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    updApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesGroup1Ipv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))));
  }

  // group 2

  for (int _ind = 0; _ind < ueNodesGroup2.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 0; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodesGroup2.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    echoApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodesGroup2.Get (_ind), // _ind+4
            ueNodesGroup2Ipv4InterfaceContainer.Get(_ind).first));
  }


  for (int _ind = 0; _ind < ueNodesGroup2.GetN()/2; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodesGroup2.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodesGroup2.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodesGroup2.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    updApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesGroup2Ipv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))));
  }

  // // group3 

  for (int _ind = 0; _ind < ueNodesGroup3.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 0; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodesGroup3.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    echoApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodesGroup3.Get (_ind), // _ind+4
            ueNodesGroup3Ipv4InterfaceContainer.Get(_ind).first));
  }


  for (int _ind = 0; _ind < ueNodesGroup3.GetN()/2; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodesGroup3.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodesGroup3.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodesGroup3.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    updApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesGroup3Ipv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))));
  }

  // // group 4

  for (int _ind = 0; _ind < ueNodesGroup4.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 0; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodesGroup4.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    echoApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodesGroup4.Get (_ind), // _ind+4
            ueNodesGroup4Ipv4InterfaceContainer.Get(_ind).first));
  }


  for (int _ind = 0; _ind < ueNodesGroup4.GetN()/2; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodesGroup4.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodesGroup4.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodesGroup4.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    updApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesGroup4Ipv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))));
  }

  // // group 5

  for (int _ind = 0; _ind < ueNodesGroup5.GetN(); ++_ind){
  // for (int _ind = 0; _ind < 0; ++_ind){
    UdpEchoServerHelper server (port);
    ApplicationContainer echoApp = server.Install (ueNodesGroup5.Get (_ind));
    Ptr<UdpEchoServer> udpEchoServerApp = echoApp.Get(0)->GetObject<UdpEchoServer>();
    echoApps.Add(echoApp);
    udpEchoServerApp->TraceConnectWithoutContext ("RxWithAddresses", 
        MakeBoundCallback (&mmwave::ParametersConfig::RxUdp, _stream, ueNodesGroup5.Get (_ind), // _ind+4
            ueNodesGroup5Ipv4InterfaceContainer.Get(_ind).first));
  }


  for (int _ind = 0; _ind < ueNodesGroup5.GetN()/2; ++_ind){
    NS_LOG_DEBUG("The address of ue " <<  (_ind+4) << ueNodesGroup5.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
    UdpClientHelper client (ueNodesGroup5.Get (_ind+4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
    ApplicationContainer udpApp = client.Install (ueNodesGroup5.Get (_ind));
    Ptr<UdpClient> udpEchoClientApp = udpApp.Get(0)->GetObject<UdpClient>();
    updApps.Add(udpApp);
    
    // mmwave::BulkSenderStruct_t *bulkSenderStruct = new mmwave::BulkSenderStruct_t();
    // bulkSenderStruct->bulkSenderApplication = udpEchoClientApp;
    // bulkSenderStruct->servertcpIpv4 = ueNodesGroup5Ipv4InterfaceContainer.Get(_ind).first;
    // bulkSenderStruct->stream = stream;
    // bulkSenderStruct->pointerToSize = &efsSize.at(_ind);
    // udpEchoClientApp->TraceConnectWithoutContext("TxWithSeqTsSize", 
    //     MakeBoundCallback(&mmwave::ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress, 
    //     bulkSenderStruct , DynamicCast<mmwave::MmWaveEnbNetDevice>(mmWaveEnbNodes.Get(0))));
  }
  
  updApps.Start (MilliSeconds (50));
  updApps.Stop (MilliSeconds (endTime));
  echoApps.Start (MilliSeconds (50));
  echoApps.Stop (MilliSeconds (endTime));
  packetSinkApps.Start (MilliSeconds (0));
  packetSinkApps.Stop (MilliSeconds (endTime));
  bulkApps.Start (MilliSeconds (0));
  bulkApps.Stop (MilliSeconds (endTime));
  NS_LOG_DEBUG("Creating sink apps");
  // end modification

  NetDeviceContainer allUeDevices;
  allUeDevices.Add(ueNetDev);
  allUeDevices.Add(ueNetDevGroup1);
  allUeDevices.Add(ueNetDevGroup2);
  allUeDevices.Add(ueNetDevGroup3);
  allUeDevices.Add(ueNetDevGroup4);
  allUeDevices.Add(ueNetDevGroup5);

  // traces ueNetDev
  for (auto ueDeviceIt = allUeDevices.Begin();ueDeviceIt != allUeDevices.End(); ++ueDeviceIt){
    Ptr<millicar::MmWaveSidelinkPhy> uePhy = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(*ueDeviceIt)->GetPhyMillicar();
    uePhy->TraceConnectWithoutContext("SlSinrReport",
											   MakeBoundCallback(&mmwave::EfStatsHelper::SinrReportCallback, &sinrReportStats));
    uePhy->TraceConnectWithoutContext("NotifyMillicarPairsSinr",
											   MakeBoundCallback(&mmwave::EfStatsHelper::AllPeersSinrReportCallback, &allPairssinrReportStats));
    Ptr<mmwave::MmWaveMillicarUeNetDevice> ueDev = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(*ueDeviceIt);
    
    ueDev->TraceConnectWithoutContext("SendPacketReport",
											   MakeBoundCallback(&mmwave::EfStatsHelper::SendPacketReportCallback, &sendPacketStats));
    
    ueDev->TraceConnectWithoutContext("RelayPacketReport",
											   MakeBoundCallback(&mmwave::EfStatsHelper::RelayPacketReportCallback, &relayPacketStats));
    // through this relay all traffic between 1-5 should pass through 6
    // NS_LOG_DEBUG("Setting test relay");
    // if (relayTime>0){
    //   NS_LOG_DEBUG("Schedule relay at time " << 0.38 << " seconds");
    //   Simulator::Schedule (Seconds (0.38), &mmwave::MmWaveMillicarUeNetDevice::TestRelay,
    //                                           ueDev, 1, 5, 4);
    // }
    
  }

  // helper->EnableTraces();
  // PrintGnuplottableNodeListToFile ("scenario.txt");
  PrintGnuplottableBuildingListToFile(tracesPath+"buildings.txt");
  PrintGnuplottableUeListToFile(tracesPath+"ues.txt");
  PrintGnuplottableEnbListToFile(tracesPath+"enbs.txt");
  // Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  // lteHelper->Initialize ();
  // lteHelper->EnablePhyTraces ();
  // lteHelper->EnableMacTraces ();

  // params.outputDir + params.simTag + 
  AnimationInterface *anim = new AnimationInterface(tracesPath+"elephant-flow-animation.xml");

  for (uint32_t i = 0; i < mmWaveEnbNodes.GetN(); ++i)
  {
    anim->UpdateNodeDescription(mmWaveEnbNodes.Get(i), "GNB"); // Optional
    anim->UpdateNodeColor(mmWaveEnbNodes.Get(i), 255, 255, 0); // Optional
  }

  // for (uint32_t i = 0; i < lteEnbNodes.GetN(); ++i)
  // {
  //   anim->UpdateNodeDescription(lteEnbNodes.Get(i), "LTE"); // Optional
  //   anim->UpdateNodeColor(lteEnbNodes.Get(i), 255, 255, 0); // Optional
  // }
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
  
  Simulator::Stop (MilliSeconds (endTime + 1000));
  Simulator::Run ();
  Simulator::Destroy ();

  sendPacketStats.EmptyCache();
  relayPacketStats.EmptyCache();
  allPairssinrReportStats.EmptyCache();
  sinrReportStats.EmptyCache();

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