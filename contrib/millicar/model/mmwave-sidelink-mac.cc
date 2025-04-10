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

#define NS_LOG_APPEND_CONTEXT                                            \
do                                                                     \
  {                                                                    \
    std::clog << Simulator::Now() << " [ Rnti " << GetRnti() << "] ";  \
  }                                                                    \
while (false);

#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mmwave-amc.h"
#include "ns3/lte-mac-sap.h"
#include "ns3/lte-radio-bearer-tag.h"
#include "mmwave-sidelink-mac.h"
#include "ns3/mmwave-mac-sched-sap.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include <ns3/mobility-model.h>
#include <ns3/mmwave-ue-net-device.h>
#include <ns3/net-device.h>
#include <ns3/node.h>
#include "mmwave-packet-relay-tag.h"
#include "mmwave-vehicular-5g-net-device.h"
#include <set>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveSidelinkMac");

namespace millicar {

MacSidelinkMemberPhySapUser::MacSidelinkMemberPhySapUser (Ptr<MmWaveSidelinkMac> mac)
  : m_mac (mac)
{

}

void
MacSidelinkMemberPhySapUser::ReceivePhyPdu (Ptr<Packet> p)
{
  m_mac->DoReceivePhyPdu (p);
}

void
MacSidelinkMemberPhySapUser::SlotIndication (mmwave::SfnSf timingInfo)
{
  m_mac->DoSlotIndication (timingInfo);
}

void
MacSidelinkMemberPhySapUser::SlSinrReport (const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize)
{
  m_mac->DoSlSinrReport (sinr, rnti, numSym, tbSize);
}

//-----------------------------------------------------------------------

RlcSidelinkMemberMacSapProvider::RlcSidelinkMemberMacSapProvider (Ptr<MmWaveSidelinkMac> mac)
  : m_mac (mac)
{

}

void
RlcSidelinkMemberMacSapProvider::TransmitPdu (TransmitPduParameters params)
{
  m_mac->DoTransmitPdu (params);
}

void
RlcSidelinkMemberMacSapProvider::ReportBufferStatus (ReportBufferStatusParameters params)
{
  m_mac->DoReportBufferStatus (params);
}

//-----------------------------------------------------------------------

NS_OBJECT_ENSURE_REGISTERED (MmWaveSidelinkMac);

TypeId
MmWaveSidelinkMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MmWaveSidelinkMac")
    .SetParent<Object> ()
    .AddAttribute ("Mcs",
                   "If AMC is not used, specify a fixed MCS value.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&MmWaveSidelinkMac::m_mcs),
                   MakeUintegerChecker<uint8_t> (0, 28))
    .AddAttribute ("UseAmc",
                   "Set to true to use adaptive modulation and coding.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&MmWaveSidelinkMac::m_useAmc),
                   MakeBooleanChecker ())
    .AddTraceSource ("SchedulingInfo",
                     "Information regarding the scheduling.",
                     MakeTraceSourceAccessor (&MmWaveSidelinkMac::m_schedulingTrace),
                     "ns3::millicar::MmWaveSidelinkMac::SlSchedulingTracedCallback")
    
    .AddAttribute("Device",
                  "The current NetDevice",
                  PointerValue(0),
                  MakePointerAccessor(&MmWaveSidelinkMac::SetDevice,
                                      &MmWaveSidelinkMac::GetDevice),
                  MakePointerChecker<NetDevice>())
    // modified
    // add a trace for the buffer status
    .AddTraceSource ("RlcBufferStatus",
                    "The Rlc buffer status coming from RLC.",
                    MakeTraceSourceAccessor (&MmWaveSidelinkMac::m_rlcBufferStatusTrace),
                    "ns3::millicar::MmWaveSidelinkMac::RlcBufferStatusTracedCallback")   
    .AddTraceSource ("DecentralizedRelaySnr",
                    "The relay snr for the decentralized case.",
                    MakeTraceSourceAccessor (&MmWaveSidelinkMac::m_decentralizedRelaySnrTrace),
                    "ns3::millicar::MmWaveSidelinkMac::DecentralizedRelaySnrTracedCallback") 
    .AddTraceSource ("RelayPacketLatency",
                    "The trace to measure the latency of the packet.",
                    MakeTraceSourceAccessor (&MmWaveSidelinkMac::m_relayLatency),
                    "ns3::millicar::MmWaveSidelinkMac::RelayPacketLatencyTracedCallback")  
    .AddAttribute("DecentralizedRelaySnrdB",
                  "The value of SNR for which a relay is triggered in the decentralized architecture",
                  DoubleValue (5.0),
                  MakeDoubleAccessor(&MmWaveSidelinkMac::m_decentralizedRelaySnr),
                  MakeDoubleChecker<double> ())
    .AddAttribute("HasDecentralizedRelay",
                  "A boolean indicating whether a decentralized relay approach is implemented",
                  BooleanValue (false),
                  MakeBooleanAccessor(&MmWaveSidelinkMac::m_hasDecentralizedRelay),
                  MakeBooleanChecker ())
    .AddAttribute ("TracesPath",
                  "The path where to store the path. ",
                  StringValue ("./"),
                  MakeStringAccessor (&MmWaveSidelinkMac::m_tracesPath),
                  MakeStringChecker ())
    
;
  return tid;
}

MmWaveSidelinkMac::MmWaveSidelinkMac (Ptr<mmwave::MmWavePhyMacCommon> pmc)
{
  NS_LOG_FUNCTION (this);

  m_phyMacConfig = pmc;

  // initialize the RNTI to 0
  m_rnti = 0;

  // create the PHY SAP USER
  m_phySapUser = new MacSidelinkMemberPhySapUser (this);

  // create the MAC SAP PROVIDER
  m_macSapProvider = new RlcSidelinkMemberMacSapProvider(this);

  // create the mmwave::MmWaveAmc instance
  m_amc = CreateObject <mmwave::MmWaveAmc> (m_phyMacConfig);

  // initialize the scheduling patter
  std::vector<uint16_t> pattern (m_phyMacConfig->GetSlotsPerSubframe (), 0);
  m_sfAllocInfo = pattern;
}

MmWaveSidelinkMac::~MmWaveSidelinkMac (void)
{
  NS_LOG_FUNCTION (this);
}

void
MmWaveSidelinkMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_phySapUser;
  Object::DoDispose ();
}

void
MmWaveSidelinkMac::DoSlotIndication (mmwave::SfnSf timingInfo)
{
  // NS_LOG_FUNCTION (this);
  m_frame = timingInfo.m_frameNum;
  m_subframe = timingInfo.m_sfNum;
  m_slotNum = timingInfo.m_slotNum;

  // NS_LOG_DEBUG("Slot indication " << m_rnti);

  // modified
  // m_relayPaths[9][13]=14;
  // m_relayPaths[1][5]=6;
  // end modification

  NS_ASSERT_MSG (m_rnti != 0, "First set the RNTI");
  NS_ASSERT_MSG (!m_sfAllocInfo.empty (), "First set the scheduling pattern");
  if(m_sfAllocInfo [timingInfo.m_slotNum] == m_rnti) // check if this slot is associated to the user who required it
  {
    // check if there should be relays in decentalized mode
    if (m_hasDecentralizedRelay){
      UpdateDecentralizedAllRelayPaths(timingInfo);
    }
    //
    mmwave::SlotAllocInfo allocationInfo = ScheduleResources (timingInfo);
    // NS_LOG_DEBUG("Schedule finished");

    // associate slot alloc info and pdu
    for (auto it = allocationInfo.m_ttiAllocInfo.begin(); it != allocationInfo.m_ttiAllocInfo.end (); it++)
    {
      // retrieve the tx buffer corresponding to the assigned destination
      auto txBuffer = m_txBufferMap.find (it->m_rnti); // the destination RNTI

      if (txBuffer == m_txBufferMap.end () || txBuffer->second.empty ())
      {
        // discard the tranmission opportunity and go to the next transmission
        continue;
      }

      // modified
      // NS_LOG_DEBUG("Peekeing the packet tag");
      MmWaveMacPacketRelayTag packetRelayTag;
      txBuffer->second.front ().pdu->PeekPacketTag(packetRelayTag);
      // uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
      // uint16_t sourceRnti = peekPacketRelayTag.GetSourceRnti();

      // packetRelayTag.SetDestinationRnti(it->m_rnti);
      // packetRelayTag.SetSourceRnti(GetRnti());

      uint16_t intermediateDest = UINT16_MAX;
      // check if rntiDest is in relay path; if it is, then we have to change the destination
      // for the path
      auto relayPathThisRntiIt =  m_relayPaths.find(GetRnti());
      if (relayPathThisRntiIt != m_relayPaths.end()){
        auto relayPathDestIt = relayPathThisRntiIt->second.find(it->m_rnti);
        if (relayPathDestIt != relayPathThisRntiIt->second.end()){
          // means there is a relay path, thus the destRnti is not the former
          // but the intermediate node
          intermediateDest = relayPathDestIt->second;
        }
      }
      
      // only from the source can be triggered a relay
      // the seconds part is to avoid to do a relay in intermediate nodes
      // this avoids a chain relay or loop relays
      if ((intermediateDest != UINT16_MAX) && (packetRelayTag.GetSourceRnti() == GetRnti())){
        // as the original
        // NS_LOG_LOGIC("Changing relay path from rnti " << it->m_rnti << " to " << intermediateDest);
        it->m_rnti = intermediateDest;
      }

      // add intermediate rnti
      // packetRelayTag.SetIntermediateRnti(intermediateDest);
      // add the packet relay tag to distinguish from normal packet
      // txBuffer->second.front ().pdu->AddPacketTag(packetRelayTag);

      // end modification

      m_relayLatency(txBuffer->second.front ().pdu, GetRnti(), 
                    packetRelayTag.GetSourceRnti(), 
                    packetRelayTag.GetDestinationRnti(), 3);

      // otherwise, forward the packet to the PHY
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      pb->AddPacket (txBuffer->second.front ().pdu);
      m_phySapProvider->AddTransportBlock (pb, *it);
      txBuffer->second.pop_front ();

      // report the buffer status and real value of tx buffer

      // uint32_t bufferSizeBytes = 0;
      // for (auto txBufferIt = txBuffer->second.begin(); txBufferIt!=txBuffer->second.end(); ++txBufferIt){
      //   bufferSizeBytes+=txBufferIt->pdu->GetSize();
      // }
      
      // uint32_t bufferStatusSize = 0;
      // auto bufferStatusMapIt = m_bufferStatusReportMap.find(txBuffer->second.front().lcid);
      // if (bufferStatusMapIt!= m_bufferStatusReportMap.end()){
      //   bufferStatusSize = bufferStatusMapIt->second.txQueueSize;
      // }
      

      // m_bufferStatusRealBufferTrace(GetRnti(), packetRelayTag.GetSourceRnti(), 
      //                               packetRelayTag.GetDestinationRnti(), 
      //                               bufferSizeBytes, bufferStatusSize);
    }

    // uint32_t numSymAlloc = 0;
    // uint32_t symStart = 0;

    // for (auto ttiAllocInfoIt = allocationInfo.m_ttiAllocInfo.begin(); ttiAllocInfoIt!=allocationInfo.m_ttiAllocInfo.end(); ++ ttiAllocInfoIt){
    //   numSymAlloc+=ttiAllocInfoIt->m_dci.m_numSym;
    // }
    

    // scheduling Relay Resources

    mmwave::SlotAllocInfo relayAllocationInfo = ScheduleRelayResources (timingInfo, allocationInfo.m_numSymAlloc, allocationInfo.m_numSymAlloc);

    // associate slot alloc info and pdu
    for (auto it = relayAllocationInfo.m_ttiAllocInfo.begin(); it != relayAllocationInfo.m_ttiAllocInfo.end (); it++)
    {
      // retrieve the tx buffer corresponding to the assigned destination
      auto txBuffer = m_txBufferMapRelay.find (it->m_rnti); // the destination RNTI

      if (txBuffer == m_txBufferMapRelay.end () || txBuffer->second.empty ())
      {
        // discard the tranmission opportunity and go to the next transmission
        continue;
      }

      // NS_LOG_UNCOND("Tx buffer search " << it->m_rnti 
      //               << " p " << (txBuffer == m_txBufferMapRelay.end ())
      //               << " buffer empty " << txBuffer->second.empty ());

      // modified
      // NS_LOG_DEBUG("Peekeing the packet tag");
      MmWaveMacPacketRelayTag packetRelayTag;
      txBuffer->second.front ().pdu->PeekPacketTag(packetRelayTag);
      // uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
      // uint16_t sourceRnti = peekPacketRelayTag.GetSourceRnti();

      m_relayLatency(txBuffer->second.front ().pdu, GetRnti(), 
                    packetRelayTag.GetSourceRnti(), 
                    packetRelayTag.GetDestinationRnti(), 4);

      // otherwise, forward the packet to the PHY
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      pb->AddPacket (txBuffer->second.front ().pdu);
      m_phySapProvider->AddTransportBlock (pb, *it);
      txBuffer->second.pop_front ();

    }
    // NS_LOG_DEBUG("Schedule finished");
  }
  else if (m_sfAllocInfo[timingInfo.m_slotNum] != 0) // if the slot is assigned to another device, prepare for reception
  {
    // NS_LOG_INFO ("Prepare for reception from rnti " << m_sfAllocInfo[timingInfo.m_slotNum]);
    m_phySapProvider->PrepareForReception (m_sfAllocInfo[timingInfo.m_slotNum]);
  }
  else // the slot is not assigned to any user
  {
    // NS_LOG_INFO ("Empty slot");
  }

}

void
MmWaveSidelinkMac::SetDevice(Ptr<NetDevice> d)
{
    m_netDevice = d;
}

Ptr<NetDevice>
MmWaveSidelinkMac::GetDevice() const
{
    return m_netDevice;
}

void 
MmWaveSidelinkMac::PrintRelay(){
  for(auto it = m_relayPaths.begin(); it != m_relayPaths.end(); ++it)
  {
    for (auto secondIt = it->second.begin(); secondIt != it->second.end(); ++secondIt){
      std::cout << it->first << " " << secondIt->first << " " << secondIt->second << "\n";
    }
  }
}

// modified
void 
MmWaveSidelinkMac::AddRelayPath(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti){
  // NS_LOG_FUNCTION(this << localRnti << destRnti<<intermediateRnti);
  NS_LOG_LOGIC("Adding relay path from rnti " << localRnti << " to " << destRnti << " through " << intermediateRnti);
  m_relayPaths[localRnti][destRnti] = intermediateRnti;
  m_relayPaths[destRnti][localRnti] = intermediateRnti;
}

void 
MmWaveSidelinkMac::UpdateDecentralizedRelayPath(mmwave::SfnSf timingInfo, uint16_t rntiDest, double directLinkSnrdb){
  // NS_LOG_FUNCTION (this);
  // std::pair<const uint64_t, double> _pair = GetBestRelayNeighbor();
  std::pair<const uint64_t, double> _pair = m_phySapProvider->GetBestRelayNeighbor();
  m_relayPaths[GetRnti()][rntiDest] = (uint16_t)_pair.first;
  // here we update the phy layer as well
  m_phySapProvider->UpdateDecentralizedFullRelayPath(rntiDest, (uint16_t)_pair.first);

  // updating the relay path in the device
  Ptr<mmwave::MmWaveMillicarUeNetDevice> device = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(m_netDevice);
  if (device!=nullptr){
    device->DecentralizedAddRelayPath(GetRnti(), rntiDest, (uint16_t)_pair.first);
  }
  // create a trace to track the activation of relay
  // local rnti, dest rnti, intermediate rnti, direct snr, best snr
  m_decentralizedRelaySnrTrace(timingInfo, GetRnti(), rntiDest, (uint16_t)_pair.first, directLinkSnrdb, _pair.second);

  // storing the info in ric control command file
  std::ofstream csv {};
  std::string m_ricControlReceivedFilename = m_tracesPath + "ric-control-messages.txt";
  csv.open (m_ricControlReceivedFilename.c_str (),  std::ios_base::app);
  if (!csv.is_open ())
  {
    NS_FATAL_ERROR ("Can't open file " << m_ricControlReceivedFilename.c_str ());
  }

  std::string to_print = std::to_string(Simulator::Now().GetSeconds ()) + "," +
                                    std::to_string(GetRnti()) + "," +
                                    std::to_string(rntiDest) + "," +
                                    std::to_string(_pair.first) + "\n";
  csv << to_print;
  
  csv.close();
}

void 
MmWaveSidelinkMac::UpdateDecentralizedAllRelayPaths(mmwave::SfnSf timingInfo){
  // NS_LOG_FUNCTION (this);
  // we update the relay paths before the scheduling for all the rnti 
  // in the bsr
  // TODO: Update relay path in the device
  Ptr<mmwave::MmWaveMillicarUeNetDevice> device = DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(m_netDevice);
  if (device==nullptr){
    // we do not add a relay
    return;
  }
  for (auto bsrIt = m_bufferStatusReportMap.begin (); bsrIt!=m_bufferStatusReportMap.end(); ++bsrIt){
    uint16_t rntiDest = bsrIt->second.rnti;
    // local rnti, dest rnti, intermediate rnti = 0, direct snr = 0, best snr
    // double directLinkSnr =  m_phySapProvider->GetDirectLinkSignalStrength(rntiDest);
    double directLinkSnr = (double)UINT32_MAX;
    auto rntiSinrLinkIt = m_rntiSinrActiveLinks.find(rntiDest);
    if (rntiSinrLinkIt!=m_rntiSinrActiveLinks.end()){
      directLinkSnr = rntiSinrLinkIt->second;
    }
    double directLinkSnrdb = 10 * std::log10 (directLinkSnr);
    // check if a relay is needed for this destination rnti
    // NS_LOG_DEBUG("Direct link snr " << directLinkSnr);
    if (directLinkSnrdb<=m_decentralizedRelaySnr){
      UpdateDecentralizedRelayPath(timingInfo, rntiDest, directLinkSnrdb);
    }else{
      // mean we do not need a relay, thus we go to the main path
      // find if there exist an entry in the relay path
      auto localRntiRelayPathIt = m_relayPaths.find (GetRnti());
      if (localRntiRelayPathIt != m_relayPaths.end()){
        // find dest rnti 
        auto destRntiRelayPathIt = localRntiRelayPathIt->second.find (rntiDest);
        if (destRntiRelayPathIt != localRntiRelayPathIt->second.end()){
          // delete the entry for the reference destination
          localRntiRelayPathIt->second.erase(destRntiRelayPathIt);
        } 
      }
      // remove the same relay path from the physical layer
      m_phySapProvider->DecentralizedRemoveRelayPath(GetRnti(), rntiDest);
      // Remove relay path in the device
      device->DecentralizedRemoveRelayPath(GetRnti(), rntiDest);
      // add the entry in the trace: 
      m_decentralizedRelaySnrTrace(timingInfo, GetRnti(), rntiDest, UINT16_MAX, directLinkSnrdb, 0);
    }
  }
}

// end modification

// this only for relay vehicles
mmwave::SlotAllocInfo
MmWaveSidelinkMac::ScheduleRelayResources (mmwave::SfnSf timingInfo, uint32_t regTrafficUsedSym, uint8_t firstSymStartAvailable)
{
  // NS_LOG_UNCOND("Schedule relay " << m_rnti << " " << timingInfo.m_frameNum);
  mmwave::SlotAllocInfo allocationInfo; // stores all the allocation decisions
  allocationInfo.m_sfnSf = timingInfo;
  allocationInfo.m_numSymAlloc = 0;
  // std::set<uint8_t> consideredLcids; 

  if (m_bufferStatusReportMapRelay.size () == 0)
  {
    return allocationInfo;
  }

  // compute the total number of available symbols
  uint32_t availableSymbols = m_phyMacConfig->GetSymbPerSlot () - regTrafficUsedSym;

  NS_ASSERT_MSG (regTrafficUsedSym <= m_phyMacConfig->GetSymbPerSlot (), "More symbols assigned than available");

  // distribute the symbols among different relay equally
  uint32_t availableSymbolsPerLc = availableSymbols / m_bufferStatusReportMapRelay.size ();

  NS_LOG_UNCOND( " rnti " << m_rnti <<
                " available symbols " << availableSymbols << 
                // " per lc " << availableSymbolsPerLc <<
                " buffer size " << m_bufferStatusReportMapRelay.size ());

  // TODO start from the last served lc + 1
  auto bsrIt = m_bufferStatusReportMapRelay.begin ();

  uint8_t symStart = firstSymStartAvailable; // indicates the next available symbol in the slot
  
  // serve the active logical channels with a Round Robin approach
  while (availableSymbols > 0 && m_bufferStatusReportMapRelay.size () > 0){
    // consideredLcids.insert(bsrIt->first);
    uint16_t rntiDest = bsrIt->second.rnti; // the RNTI of the destination node
    uint8_t mcs = GetMcs (rntiDest); // select the MCS

    // compute the number of bits for this LC
    uint32_t availableBytesPerLc = m_amc->CalculateTbSize(mcs, availableSymbolsPerLc);
    uint32_t availableBytes = m_amc->CalculateTbSize(mcs, availableSymbols);

    uint32_t requiredBytes = 0;
    uint32_t assignedBytes = 0;
    uint32_t assignedSymbols = 0;
    uint32_t remainingSymbolsPerLc = availableSymbolsPerLc;
    bool firstPacketConsidered = false;

    NS_LOG_UNCOND(" rnti " << rntiDest << " mcs " << (uint32_t)mcs << 
                  " packet sizes " << bsrIt->second.txPacketSizes.size() <<
                  " total size " << bsrIt->second.txQueueSize
                  );

    // iterate over individual packets
    std::list<uint32_t>::iterator txPacketSizesIt=bsrIt->second.txPacketSizes.begin();
    for(; txPacketSizesIt != bsrIt->second.txPacketSizes.end(); ){ // ++txPacketSizesIt
      
      NS_LOG_UNCOND(" " << (*txPacketSizesIt));
      // we have to calculate the used symbols per packet size
      // uint32_t requiredBytes = (bsrIt->second.txQueueSize + bsrIt->second.retxQueueSize + bsrIt->second.statusPduSize);
      requiredBytes = (*txPacketSizesIt);

      // assign a number of bits which is less or equal to the available bits
      
      if (requiredBytes <= availableBytesPerLc)
      {
        assignedBytes = requiredBytes;
      }
      else
      {
        // assignedBytes = availableBytesPerLc;
        // means we have a packet which is bigger to availableBytesPerLc
        // we use symbols from other lc
        // we do so to avoid traffic congestion due to packet size
        // though we do this if there is a big packet in the front of the buffer
        if(!firstPacketConsidered){
          // means a packet has not been scheduled
          // check if there is avaialable symbols from the other side
          // we want to make sure there is not a packet that blocks the traffic
          if (requiredBytes<=availableBytes){
            // if the packet is big enough, we can take from other's symbols to send the packet
            assignedBytes = requiredBytes;
          }else{

            break;
          }
        }else{
          // we do not continue with other packets in the buffer
          break;
        }
      }

      firstPacketConsidered = true;

      // compute the number of symbols assigned to this LC
      assignedSymbols = m_amc->GetMinNumSymForTbSize (assignedBytes, mcs);

      // NS_LOG_UNCOND("Assigned bytes " << assignedBytes << " assign symbols " << assignedSymbols);

      mmwave::TtiAllocInfo info;
      info.m_ttiIdx = timingInfo.m_slotNum; // the TB will be sent in this slot
      info.m_rnti = rntiDest; // the RNTI of the destination node
      info.m_dci.m_rnti = m_rnti; // my RNTI
      info.m_dci.m_numSym = assignedSymbols; // the number of symbols required to tx the packet
      info.m_dci.m_symStart = symStart; // index of the first available symbol
      info.m_dci.m_mcs = mcs;
      info.m_dci.m_tbSize = assignedBytes; // the TB size in bytes
      info.m_ttiType = mmwave::TtiAllocInfo::TddTtiType::DATA; // the TB carries data

      allocationInfo.m_ttiAllocInfo.push_back (info);
      allocationInfo.m_numSymAlloc += assignedSymbols;

      // fire the scheduling trace
      SlSchedulingCallback traceInfo;
      traceInfo.frame = timingInfo.m_frameNum;
      traceInfo.subframe = timingInfo.m_sfNum;
      traceInfo.slotNum = timingInfo.m_slotNum;
      traceInfo.symStart = symStart;
      traceInfo.numSym = assignedSymbols;
      traceInfo.mcs = mcs;
      traceInfo.tbSize = assignedBytes;
      traceInfo.txRnti = m_rnti;

      m_schedulingTrace (info.m_rnti, traceInfo);

      // end modification
      // NS_LOG_DEBUG("Update bsr ");

      bsrIt = UpdateRelayUserBufferStatusReport (bsrIt->second.lcid, assignedBytes);

      // update the number of available symbols
      availableSymbols -= assignedSymbols;
      remainingSymbolsPerLc -= assignedSymbols;
      availableBytesPerLc = m_amc->CalculateTbSize(mcs, remainingSymbolsPerLc);

      // update the available bytes
      availableBytes = m_amc->CalculateTbSize(mcs, availableSymbols);


      if (availableSymbols < availableSymbolsPerLc)
      {
        availableSymbolsPerLc = availableSymbols;
      }

      // update index to the next available symbol
      symStart = symStart + assignedSymbols;

      // ++txPacketSizesIt;
      // go to next packet in front
      txPacketSizesIt=bsrIt->second.txPacketSizes.begin();
    }
    bsrIt = UpdateRelayBufferStatusReport (bsrIt->second.lcid, 0);

    // if the iterator reached the end of the map, start again
    if (bsrIt == m_bufferStatusReportMapRelay.end ())
    {
      // there is no need to get in the begin, rather we just 
      // get out of loop by returning the allocations
      // bsrIt = m_bufferStatusReportMapRelay.begin ();
      return allocationInfo;
    }
    // find 
  }

}

std::map<uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator
MmWaveSidelinkMac::UpdateRelayUserBufferStatusReport (uint8_t lcid, uint32_t assignedBytes){
  // NS_LOG_FUNCTION(this << (uint32_t)lcid);
  
  // find the corresponding entry in the map
  auto bsrIt = m_bufferStatusReportMapRelay.find (lcid);

  NS_ASSERT_MSG (bsrIt != m_bufferStatusReportMapRelay.end (), "m_bufferStatusReportMapRelay does not contain the required entry");

  if (bsrIt->second.txQueueSize > assignedBytes)
  {
    bsrIt->second.txQueueSize -= assignedBytes;
    assignedBytes = 0;  
  }
  else
  {
    assignedBytes -= bsrIt->second.txQueueSize;
    bsrIt->second.txQueueSize = 0;
  }

  NS_LOG_UNCOND("UpdateRelayUser " << (uint32_t)lcid << " bytes " << assignedBytes << 
                " tx queue " << bsrIt->second.txQueueSize << 
                " front packet size " << bsrIt->second.txPacketSizes.front()<< 
                " rnti " << m_rnti);

  // remove the packet from list
  bsrIt->second.txPacketSizes.pop_front();

  return bsrIt;
}

std::map<uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator
MmWaveSidelinkMac::UpdateRelayBufferStatusReport (uint8_t lcid, uint32_t assignedBytes){
  // NS_LOG_FUNCTION(this << (uint32_t)lcid);
  // find the corresponding entry in the map
  auto bsrIt = m_bufferStatusReportMapRelay.find (lcid);

  NS_ASSERT_MSG (bsrIt != m_bufferStatusReportMapRelay.end (), "m_bufferStatusReportMapRelay does not contain the required entry");

  // delete the entry in the map if no further resources are needed
  if ( (bsrIt->second.txQueueSize == 0) || (bsrIt->second.txPacketSizes.size() == 0) )
  {
    // erasing also from relay lcid and rnti map
    // NS_LOG_DEBUG("Rem lcid " << (uint32_t)bsrIt->second.lcid 
    //             << " rnti " << bsrIt->second.rnti);
    auto relayLcidRntiIt = m_relayLcidRntiMap.find(bsrIt->second.lcid);
    if (relayLcidRntiIt != m_relayLcidRntiMap.end()){
      m_relayLcidRntiMap.erase(relayLcidRntiIt);
    }

    NS_LOG_UNCOND("Erasing lcid " << (uint32_t)lcid << " rnti " << m_rnti);

    bsrIt = m_bufferStatusReportMapRelay.erase (bsrIt);
  }
  else
  {
    bsrIt++;
  }

  return bsrIt;
}


mmwave::SlotAllocInfo
MmWaveSidelinkMac::ScheduleResources (mmwave::SfnSf timingInfo)
{
  mmwave::SlotAllocInfo allocationInfo; // stores all the allocation decisions
  allocationInfo.m_sfnSf = timingInfo;
  allocationInfo.m_numSymAlloc = 0;

  // NS_LOG_DEBUG("m_bufferStatusReportMap.size () =\t" << m_bufferStatusReportMap.size ());
  // if there are no active channels return an empty vector
  if (m_bufferStatusReportMap.size () == 0)
  {
    return allocationInfo;
  }

  // compute the total number of available symbols
  uint32_t availableSymbols = m_phyMacConfig->GetSymbPerSlot ();

  // NS_LOG_DEBUG("availableSymbols =\t" << availableSymbols);

  // compute the number of available symbols per logical channel
  // NOTE the number of available symbols per LC is rounded down due to the cast
  // to int
  uint32_t availableSymbolsPerLc = availableSymbols / m_bufferStatusReportMap.size ();

  uint32_t maxSymbolsRelay = 14;

  // NS_LOG_DEBUG("availableSymbolsPerLc =\t" << availableSymbolsPerLc);

  // TODO start from the last served lc + 1
  auto bsrIt = m_bufferStatusReportMap.begin ();

  uint8_t symStart = 0; // indicates the next available symbol in the slot

  // serve the active logical channels with a Round Robin approach
  while (availableSymbols > 0 && m_bufferStatusReportMap.size () > 0)
  {
    uint16_t rntiDest = bsrIt->second.rnti; // the RNTI of the destination node

    // modified
    uint16_t intermediateDest = UINT16_MAX;
    // check if rntiDest is in relay path; if it is, then we have to change the destination
    // for the path
    auto relayPathThisRntiIt =  m_relayPaths.find(GetRnti());
    if (relayPathThisRntiIt != m_relayPaths.end()){
      auto relayPathDestIt = relayPathThisRntiIt->second.find(rntiDest);
      if (relayPathDestIt != relayPathThisRntiIt->second.end()){
        // means there is a relay path, thus the destRnti is not the former
        // but the intermediate node
        intermediateDest = relayPathDestIt->second;
      }
    }
    // end modification
    // original
    // uint8_t mcs = GetMcs (rntiDest); // select the MCS
    // NS_LOG_DEBUG("rnti " << rntiDest << " mcs = " << uint16_t(mcs));
    // end original
    // modified
    // here we have to use intermediate node if it is in relay paths
    // we have to check as well we are not in the intermediate node
    // if we are, we should refer to the mcs of the dest rnti
    // the second condition indicates we have relay traffic, thus we 
    // should use te mcs of the destination, rather that of intermediate node
    uint8_t mcs = 0;
    bool isRelay = false;
    if ((intermediateDest== UINT16_MAX)  || (m_relayLcidRntiMap.find(bsrIt->second.lcid) == m_relayLcidRntiMap.end())){ //
      // as the original
      mcs = GetMcs (rntiDest); // select the MCS
      isRelay = true;
      // NS_LOG_DEBUG("rnti " << rntiDest << " mcs = " << uint16_t(mcs));
    }else{
      // we are in a relay path, thus use the intermediate rnti
      mcs = GetMcs (intermediateDest); // select the MCS
      // mcs = 0;
      // NS_LOG_LOGIC("intermediate rnti " << intermediateDest << " mcs = " << uint16_t(mcs));
    }
    
    // end modification
    // compute the number of bits for this LC
    uint32_t availableBytesPerLc = m_amc->CalculateTbSize(mcs, availableSymbolsPerLc);

    uint32_t maxTxBytesRelay = m_amc->CalculateTbSize(mcs, maxSymbolsRelay);

    // compute the number of bits required by this LC
    uint32_t requiredBytes = (bsrIt->second.txQueueSize + bsrIt->second.retxQueueSize + bsrIt->second.statusPduSize);

    // assign a number of bits which is less or equal to the available bits
    uint32_t assignedBytes = 0;
    // original
    // if (requiredBytes <= availableBytesPerLc)
    // {
    //   assignedBytes = requiredBytes;
    // }
    // else
    // {
    //   assignedBytes = availableBytesPerLc;
    // }
    // end original
    // modified
    // set a limit for the relay traffic
    // if (isRelay){
    //   if (availableBytesPerLc < maxTxBytesRelay){

    //   }
    // }else{
    if (requiredBytes <= availableBytesPerLc)
    {
      if (availableBytesPerLc <= maxTxBytesRelay){
        assignedBytes = requiredBytes;
      }
      else{
        assignedBytes = maxTxBytesRelay;
      }
    }
    else
    {
      if (availableBytesPerLc <= maxTxBytesRelay){
        assignedBytes = availableBytesPerLc;
      }else{
        assignedBytes = maxTxBytesRelay;
      }
    }
    // }
    // end modification
    



    // compute the number of symbols assigned to this LC
    uint32_t assignedSymbols = m_amc->GetMinNumSymForTbSize (assignedBytes, mcs);

    //if (assignedSymbols <= availableSymbols) // TODO check if needed
    //{
    // create the TtiAllocInfo object
    mmwave::TtiAllocInfo info;
    info.m_ttiIdx = timingInfo.m_slotNum; // the TB will be sent in this slot
    // original
    // we leave it as it is, since the packets are already sent by the rlc to mac; 
    // thus mac has already the packtes in txbuffer. We change the rnti when
    // the data are passed to phy layer
    info.m_rnti = rntiDest; // the RNTI of the destination node
    // end original
    // modified
    // if (intermediateDest == UINT16_MAX){
    //   // as the original
    //   info.m_rnti = rntiDest;
    // }else{ 
    //   info.m_rnti = intermediateDest;
    // }
    // end modification
    info.m_dci.m_rnti = m_rnti; // my RNTI
    info.m_dci.m_numSym = assignedSymbols; // the number of symbols required to tx the packet
    info.m_dci.m_symStart = symStart; // index of the first available symbol
    info.m_dci.m_mcs = mcs;
    info.m_dci.m_tbSize = assignedBytes; // the TB size in bytes
    info.m_ttiType = mmwave::TtiAllocInfo::TddTtiType::DATA; // the TB carries data

    // NS_LOG_DEBUG("info.m_dci.m_tbSize =\t" << info.m_dci.m_tbSize);

    allocationInfo.m_ttiAllocInfo.push_back (info);
    allocationInfo.m_numSymAlloc += assignedSymbols;

    // fire the scheduling trace
    SlSchedulingCallback traceInfo;
    traceInfo.frame = timingInfo.m_frameNum;
    traceInfo.subframe = timingInfo.m_sfNum;
    traceInfo.slotNum = timingInfo.m_slotNum;
    traceInfo.symStart = symStart;
    traceInfo.numSym = assignedSymbols;
    traceInfo.mcs = mcs;
    traceInfo.tbSize = assignedBytes;
    traceInfo.txRnti = m_rnti;
    // original
    // traceInfo.rxRnti = rntiDest;
    // end original
    // modified
    // here we change as we want to have info on the real scheduling
    // since the data are scheduled for the intermediate rnti now
    if (intermediateDest == UINT16_MAX){
      // as the original
      info.m_rnti = rntiDest;
    }else{
      info.m_rnti = intermediateDest;
    }
    // end modification
    
    m_schedulingTrace (info.m_rnti, traceInfo);

    // notify the RLC
    // NS_LOG_DEBUG("Notifying rlc: local " << GetRnti() << " inter " << intermediateDest 
    //           << " dest " << rntiDest << " lcid " << (uint32_t)bsrIt->second.lcid);
    // original
    // LteMacSapUser* macSapUser = m_lcidToMacSap.find (bsrIt->second.lcid)->second;
    // modified
    LteMacSapUser* macSapUser;
    // if the lcid does not belong to the relay, we use the local rlc
    bool lcidIsInRelayMap = m_relayLcidRntiMap.find(bsrIt->second.lcid) != m_relayLcidRntiMap.end();
    if (!lcidIsInRelayMap){
      macSapUser = m_lcidToMacSap.find (bsrIt->second.lcid)->second;
    }
    // end modification
    LteMacSapUser::TxOpportunityParameters params;
    params.bytes = assignedBytes;  // the number of bytes to transmit
    params.layer = 0;  // the layer of transmission (MIMO) (NOT USED)
    params.harqId = 0; // the HARQ ID (NOT USED)
    params.componentCarrierId = 0; // the component carrier id (NOT USED)
    // this we leave it unchanged, since we don't want to mess with rlc
    // though it doesn't change anything changing rntiDest in these params
    params.rnti = rntiDest; // the C-RNTI identifying the destination
    params.lcid = bsrIt->second.lcid; // the logical channel id
    // original
    // macSapUser->NotifyTxOpportunity (params);
    // modified
    // we only notify the rlc of new tx opportunity if it is traffic
    // being generated by this node
    // if the traffic is relay traffic, we do not have to inform the rlc
    if (!lcidIsInRelayMap){
      // NS_LOG_DEBUG("Notifying rlc: local " << GetRnti());
      macSapUser->NotifyTxOpportunity (params);
    }
    // end modification
    // NS_LOG_DEBUG("Update bsr ");
    // update the entry in the m_bufferStatusReportMap (delete it if no
    // further resources are needed)
    bsrIt = UpdateBufferStatusReport (bsrIt->second.lcid, assignedBytes);

    // update the number of available symbols
    availableSymbols -= assignedSymbols;

    // update the availableSymbolsPerLc (if needed)
    if (availableSymbols < availableSymbolsPerLc)
    {
      availableSymbolsPerLc = availableSymbols;
    }

    // update index to the next available symbol
    symStart = symStart + assignedSymbols;

    // if the iterator reached the end of the map, start again
    if (bsrIt == m_bufferStatusReportMap.end ())
    {
      bsrIt = m_bufferStatusReportMap.begin ();
    }

  }
  return allocationInfo;
}

std::map<uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator
MmWaveSidelinkMac::UpdateBufferStatusReport (uint8_t lcid, uint32_t assignedBytes)
{
  // NS_LOG_FUNCTION(this << (uint32_t)lcid);
  // find the corresponding entry in the map
  auto bsrIt = m_bufferStatusReportMap.find (lcid);

  NS_ASSERT_MSG (bsrIt != m_bufferStatusReportMap.end (), "m_bufferStatusReportMap does not contain the required entry");

  // NOTE RLC transmits PDUs in the following priority order:
  // 1) STATUS PDUs
  // 2) retransmissions
  // 3) regular PDUs
  if (bsrIt->second.statusPduSize > assignedBytes)
  {
    bsrIt->second.statusPduSize -= assignedBytes;
    assignedBytes = 0;
  }
  else
  {
    assignedBytes -= bsrIt->second.statusPduSize;
    bsrIt->second.statusPduSize = 0;
  }

  if (bsrIt->second.retxQueueSize > assignedBytes)
  {
    bsrIt->second.retxQueueSize -= assignedBytes;
    assignedBytes = 0;
  }
  else
  {
    assignedBytes -= bsrIt->second.retxQueueSize;
    bsrIt->second.retxQueueSize = 0;
  }

  if (bsrIt->second.txQueueSize > assignedBytes)
  {
    bsrIt->second.txQueueSize -= assignedBytes;
    assignedBytes = 0;
  }
  else
  {
    assignedBytes -= bsrIt->second.txQueueSize;
    bsrIt->second.txQueueSize = 0;
  }

  // delete the entry in the map if no further resources are needed
  if (bsrIt->second.statusPduSize == 0 && bsrIt->second.retxQueueSize == 0  && bsrIt->second.txQueueSize == 0)
  {
    // erasing also from relay lcid and rnti map
    // NS_LOG_DEBUG("Rem lcid " << (uint32_t)bsrIt->second.lcid 
    //             << " rnti " << bsrIt->second.rnti);
    // auto relayLcidRntiIt = m_relayLcidRntiMap.find(bsrIt->second.lcid);
    // if (relayLcidRntiIt != m_relayLcidRntiMap.end()){
    //   m_relayLcidRntiMap.erase(relayLcidRntiIt);
    // }

    bsrIt = m_bufferStatusReportMap.erase (bsrIt);
  }
  else
  {
    bsrIt++;
  }

  return bsrIt;
}

void
MmWaveSidelinkMac::DoReportBufferStatus (LteMacSapProvider::ReportBufferStatusParameters params)
{
  NS_LOG_FUNCTION (this);

  auto bsrIt = m_bufferStatusReportMap.find (params.lcid);
  if (bsrIt != m_bufferStatusReportMap.end ())
  {
    bsrIt->second = params;
    // NS_LOG_DEBUG("Update buffer status report for LCID " << uint32_t(params.lcid));
  }
  else
  {
    m_bufferStatusReportMap.insert (std::make_pair (params.lcid, params));
    // NS_LOG_DEBUG("Insert buffer status report for LCID " << uint32_t(params.lcid));
  }

  // test e2

  // modified
  mmwave::MmWaveMacSchedSapProvider::SchedDlRlcBufferReqParameters schedParams;
  schedParams.m_logicalChannelIdentity = params.lcid;
  schedParams.m_rlcRetransmissionHolDelay = params.retxQueueHolDelay;
  schedParams.m_rlcRetransmissionQueueSize = params.retxQueueSize;
  schedParams.m_rlcStatusPduSize = params.statusPduSize;
  schedParams.m_rlcTransmissionQueueHolDelay = params.txQueueHolDelay;
  schedParams.m_rlcTransmissionQueueSize = params.txQueueSize;
  schedParams.m_rnti = params.rnti;

  schedParams.m_txPacketSizes = params.txPacketSizes;
  schedParams.m_txPacketDelays = params.txPacketDelays;
  schedParams.m_retxPacketSizes = params.retxPacketSizes;
  schedParams.m_retxPacketDelays = params.retxPacketDelays;
  schedParams.m_arrivalRate = params.arrivalRate;

  // modified 
  // insert the buffer status in the trace
  m_rlcBufferStatusTrace(mmwave::SfnSf(m_frame, m_subframe, m_slotNum), m_rnti, schedParams);
  // end modification
  // end modification

  
}

void
MmWaveSidelinkMac::DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params)
{
  NS_LOG_FUNCTION (this);
  LteRadioBearerTag tag (params.rnti, params.lcid, params.layer);
  params.pdu->AddPacketTag (tag);

  MmWaveMacPacketRelayTag packetRelayTag;
  packetRelayTag.SetDestinationRnti(params.rnti);
  packetRelayTag.SetSourceRnti(GetRnti());
  params.pdu->AddPacketTag(packetRelayTag);
  m_relayLatency(params.pdu, GetRnti(), GetRnti(), params.rnti, 1);

  //insert the packet at the end of the buffer
  // NS_LOG_DEBUG("Add packet local " << GetRnti() 
  //               << " for RNTI " << params.rnti << 
  //               " LCID " << uint32_t(params.lcid));

  auto it = m_txBufferMap.find (params.rnti);
  if (it == m_txBufferMap.end ())
  {
    std::list<LteMacSapProvider::TransmitPduParameters> txBuffer;
    txBuffer.push_back (params);
    m_txBufferMap.insert (std::make_pair (params.rnti, txBuffer));
  }
  else
  {
    it->second.push_back (params);
  }

  // with the macPduMap
  // auto macPduIt = m_macPduMap.find(params.rnti);
  // if (macPduIt == m_macPduMap.end ())
  // {
  //   MacPduInfo macPduInfo(pduSfn,
  //                           ttiAllocInfo.m_dci.m_tbSize,
  //                           rlcPduInfo.size(),
  //                           dciElem);
  //   macPduIt->
  // }else{

  // }
}
// original
// void
// MmWaveSidelinkMac::DoReceivePhyPdu (Ptr<Packet> p)
// {
//   NS_LOG_FUNCTION(this << p);
//   LteMacSapUser::ReceivePduParameters rxPduParams;
//   LteRadioBearerTag tag;
//   p->PeekPacketTag (tag);
//   // pick the right lcid associated to this communication. As discussed, this can be done via a dedicated SidelinkBearerTag
//   rxPduParams.p = p;
//   rxPduParams.rnti = tag.GetRnti ();
//   rxPduParams.lcid = tag.GetLcid ();
//   // NS_LOG_DEBUG ("Received a packet "  << rxPduParams.rnti << " " << (uint16_t)rxPduParams.lcid);
//   LteMacSapUser* macSapUser = m_lcidToMacSap.find(rxPduParams.lcid)->second;
//   macSapUser->ReceivePdu (rxPduParams);
// }
// end original
// modified
void
MmWaveSidelinkMac::DoReceivePhyPdu (Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this << p);
  // check if is relay
  Ptr<Packet> packet = p->Copy();
  // get the relay tag
  MmWaveMacPacketRelayTag peekPacketRelayTag;
  bool hasRelayTag = p->RemovePacketTag(peekPacketRelayTag);
  uint16_t destinationRnti = peekPacketRelayTag.GetDestinationRnti();
  uint16_t sourceRnti = peekPacketRelayTag.GetSourceRnti();
  // uint16_t intermediateRnti = peekPacketRelayTag.GetIntermediateRnti();

  // if (packetRelayTag.GetIntermediateRnti()!=UINT16_MAX){
  //   NS_LOG_UNCOND("Dest " << destinationRnti << " source " << sourceRnti
  //             << " inter " << intermediateRnti << " local " << GetRnti()
  //             << " packet size " << p->GetSize());
  // }

  // it means we have a relay packet
  // first we trigger the device callback to store the relay packet
  // m_forwardUpCallback(packet);
  // and send directly to the destination
  LteMacSapUser::ReceivePduParameters rxPduParams;
  LteRadioBearerTag tag;
  p->PeekPacketTag (tag);
  // pick the right lcid associated to this communication. As discussed, this can be done via a dedicated SidelinkBearerTag
  rxPduParams.p = p;
  rxPduParams.rnti = tag.GetRnti ();
  rxPduParams.lcid = tag.GetLcid ();
  // NS_LOG_DEBUG ("Received a packet " << rxPduParams.rnti << " " << (uint16_t)rxPduParams.lcid);
  LteMacSapUser* macSapUser = m_lcidToMacSap.find(rxPduParams.lcid)->second;
  // we send the packet to upper layers anyway
  // in case of a relay, it will only store in the traces
  macSapUser->ReceivePdu (rxPduParams);


  m_relayLatency(packet, GetRnti(), sourceRnti, destinationRnti, 2);

  // intermediate node
  if (destinationRnti!=GetRnti()){
    // NS_LOG_DEBUG("Relay in inter " << GetRnti() 
    //             // << " - " << intermediateRnti 
    //             // << " source " << sourceRnti 
    //             << " dest " << destinationRnti
    //             << " lcid " << (uint32_t)tag.GetLcid());
    
    // Ptr<Packet> packet = p->Copy();
    // MmWaveMacPacketRelayTag packetRelayTag;
    // packet->RemovePacketTag(packetRelayTag);
    
    // LteRadioBearerTag tagRelay;
    // packet->PeekPacketTag(tagRelay);
    LteMacSapProvider::TransmitPduParameters txPduParams;
    // txPduParams.rnti = tagRelay.GetRnti();
    // add relay tag to the packet
    // packet->AddPacketTag(packetRelayTag);
    txPduParams.pdu = packet;
    txPduParams.rnti = destinationRnti;
    txPduParams.lcid = tag.GetLcid();
    txPduParams.layer = tag.GetLayer();
    // shouldn't change anything, just reput it in the buffer
    // first check if there is an antry with the rnti in the buffer
    // NS_LOG_UNCOND("Tx buffer rnti " << destinationRnti << " p.size " << packet->GetSize());
    auto it = m_txBufferMapRelay.find (destinationRnti);// the destination RNTI
    if (it == m_txBufferMapRelay.end ())
    {
      std::list<LteMacSapProvider::TransmitPduParameters> txBuffer;
      txBuffer.push_back (txPduParams);
      m_txBufferMapRelay.insert (std::make_pair (destinationRnti, txBuffer));
    }
    else
    {
      it->second.push_back (txPduParams);
    }

    // update the buffer status report map
    
    auto bsrIt = m_bufferStatusReportMapRelay.find (tag.GetLcid());
    if (bsrIt != m_bufferStatusReportMapRelay.end ())
    {
      // add tx queue size of the relay if exists
      bsrIt->second.txQueueSize+=packet->GetSize();
      // insert packet size -> will be used to schedule resources
      bsrIt->second.txPacketSizes.push_back(packet->GetSize());
    }else{
      // if does not exist 
      LteMacSapProvider::ReportBufferStatusParameters params;
      params.rnti = destinationRnti;
      params.lcid = tag.GetLcid();
      params.txQueueSize = packet->GetSize();
      // insert packet size -> will be used to schedule resources
      params.txPacketSizes.push_back(packet->GetSize());
      params.txQueueHolDelay = 0;
      params.retxQueueSize = 0;
      params.retxQueueHolDelay = 0;
      params.statusPduSize = 0;
      m_bufferStatusReportMapRelay.insert (std::make_pair (tag.GetLcid(), params));
    }
    // inser in the map of lcid-s 
    if (m_relayLcidRntiMap.find(tag.GetLcid()) == m_relayLcidRntiMap.end()){
      m_relayLcidRntiMap.insert(std::make_pair (tag.GetLcid(), destinationRnti));
    }
  }
}

// end modification

MmWaveSidelinkPhySapUser*
MmWaveSidelinkMac::GetPhySapUser () const
{
  NS_LOG_FUNCTION (this);
  return m_phySapUser;
}

void
MmWaveSidelinkMac::SetPhySapProvider (MmWaveSidelinkPhySapProvider* sap)
{
  NS_LOG_FUNCTION (this);
  m_phySapProvider = sap;
}

LteMacSapProvider*
MmWaveSidelinkMac::GetMacSapProvider () const
{
  NS_LOG_FUNCTION (this);
  return m_macSapProvider;
}

void
MmWaveSidelinkMac::SetRnti (uint16_t rnti)
{
  m_rnti = rnti;
}

uint16_t
MmWaveSidelinkMac::GetRnti () const
{
  return m_rnti;
}

void
MmWaveSidelinkMac::SetSfAllocationInfo (std::vector<uint16_t> pattern)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (pattern.size () == m_phyMacConfig->GetSlotsPerSubframe (), "The number of pattern elements must be equal to the number of slots per subframe");
  m_sfAllocInfo = pattern;
}

void
MmWaveSidelinkMac::SetForwardUpCallback (Callback <void, Ptr<Packet> > cb)
{
  m_forwardUpCallback = cb;
}

// modified
void 
MmWaveSidelinkMac::SetSendE2MessageCallback (Callback <void, E2AP_PDU* > cb){
  m_sendE2MessageCallback = cb;
}
// end modification

void
MmWaveSidelinkMac::DoSlSinrReport (const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize)
{
  NS_LOG_FUNCTION (this);

  // check if the m_slCqiReported already contains the CQI history for the device
  // with RNTI = rnti. If so, add the new CQI report, otherwise create a new
  // entry.
  uint8_t mcs; // the selected MCS will be stored in this variable
  if (m_slCqiReported.find (rnti) != m_slCqiReported.end () )
  {
    m_slCqiReported.at (rnti).push_back (m_amc->CreateCqiFeedbackWbTdma (sinr, mcs));
  }
  else
  {
    std::vector<int> cqiTemp;
    cqiTemp.push_back (m_amc->CreateCqiFeedbackWbTdma (sinr, mcs));
    m_slCqiReported.insert (std::make_pair(rnti, cqiTemp));
  }
  m_rntiSinrActiveLinks[rnti] = Sum(sinr)/(sinr.GetSpectrumModel ()->GetNumBands ()); 
}

uint8_t
MmWaveSidelinkMac::GetMcs (uint16_t rnti)
{
  // NS_LOG_FUNCTION (this);

  uint8_t mcs; // the selected MCS
  if (m_useAmc)
  {
    // if AMC is used, select the MCS based on the CQI history
    if (m_slCqiReported.find (rnti) != m_slCqiReported.end ())
    {
      std::vector <int> cqi = m_slCqiReported.find (rnti)->second;
      mcs = m_amc->GetMcsFromCqi(cqi.back ());
    }
    else
    {
      mcs = 0; // if the CQI history not found for this device, use the minimum MCS value
    }
  }
  else
  {
    // if AMC is not used, use a fixed MCS value
    mcs = m_mcs;
  }
  // TODO: update the method to refresh CQI reports and delete older reports
  return mcs;
}

void
MmWaveSidelinkMac::AddMacSapUser (uint8_t lcid, LteMacSapUser* macSapUser)
{
  NS_LOG_FUNCTION (this);
  m_lcidToMacSap.insert(std::make_pair(lcid, macSapUser));
}

} // mmwave namespace

} // ns3 namespace
