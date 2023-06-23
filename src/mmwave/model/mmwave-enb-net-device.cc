/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
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
*
*   Author: Marco Miozzo <marco.miozzo@cttc.es>
*           Nicola Baldo  <nbaldo@cttc.es>
*
*   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
*                         Sourjya Dutta <sdutta@nyu.edu>
*                         Russell Ford <russell.ford@nyu.edu>
*                         Menglei Zhang <menglei@nyu.edu>
*
*       Modified by: Tommaso Zugno <tommasozugno@gmail.com>
*                                Integration of Carrier Aggregation
*/


#include <ns3/llc-snap-header.h>
#include <ns3/simulator.h>
#include <ns3/callback.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include "mmwave-net-device.h"
#include <ns3/packet-burst.h>
#include <ns3/uinteger.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/pointer.h>
#include <ns3/enum.h>
#include <ns3/uinteger.h>
#include <ns3/double.h>
#include "mmwave-enb-net-device.h"
#include "mmwave-ue-net-device.h"
#include <ns3/lte-enb-rrc.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/mmwave-component-carrier-enb.h>
#include <ns3/config.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-um-lowlat.h>
#include <ns3/lte-rlc-am.h>

#include <ns3/mmwave-indication-message-helper.h>

#include <ns3/tinyxml2.h>

#include "encode_e2apv1.hpp" 

#include <ns3/mmwave-vehicular-5g-net-device.h>

extern "C" {
#include "E2SM-KPM-IndicationHeader-Format1.h"
#include "E2SM-KPM-IndicationMessage-Format1.h"
#include "GlobalE2node-ID.h"
#include "GlobalE2node-gNB-ID.h"
#include "GlobalE2node-eNB-ID.h"
#include "GlobalE2node-ng-eNB-ID.h"
#include "GlobalE2node-en-gNB-ID.h"
#include "NRCGI.h"
#include "PM-Containers-Item.h"
#include "PF-Container.h"
#include "PF-ContainerListItem.h"
#include "OCUUP-PF-Container.h"
#include "OCUCP-PF-Container.h"
#include "ODU-PF-Container.h"
#include "CellResourceReportListItem.h"
#include "ServedPlmnPerCellListItem.h"
#include "PlmnID-Item.h"
#include "PLMN-Identity.h"
#include "EPC-DU-PM-Container.h"
#include "EPC-CUUP-PM-Format.h"
#include "PerQCIReportListItem.h"
#include "PerQCIReportListItemFormat.h"
#include "NRCellIdentity.h"
#include "MeasurementTypeName.h"
#include "PM-Info-Item.h"
#include "PerUE-PM-Item.h"
#include "OCTET_STRING.h"
#include "e2ap_asn1c_codec.h"
}

// #include <ns3/mpi-interface.h>
#include <ns3/node-list.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveEnbNetDevice");

namespace mmwave {


NS_OBJECT_ENSURE_REGISTERED (MmWaveEnbNetDevice);

/**
* KPM Subscription Request callback.
* This function is triggered whenever a RIC Subscription Request for 
* the KPM RAN Function is received.
*
* \param pdu request message
*/
void 
MmWaveEnbNetDevice::KpmSubscriptionCallback (E2AP_PDU_t* sub_req_pdu)
{
  NS_LOG_DEBUG ("\nReceived RIC Subscription Request, cellId= " << m_cellId << "\n");

  E2Termination::RicSubscriptionRequest_rval_s params = m_e2term->ProcessRicSubscriptionRequest (sub_req_pdu);
  NS_LOG_DEBUG ("requestorId " << +params.requestorId << 
                 ", instanceId " << +params.instanceId << 
                 ", ranFuncionId " << +params.ranFuncionId << 
                 ", actionId " << +params.actionId);  

  if (!m_isReportingEnabled)
  {
    // BuildAndSendReportMessage (params);
    BuildAndSendMillicarReportMessage(params);
    m_isReportingEnabled = true; 
  }
    
}

TypeId MmWaveEnbNetDevice::GetTypeId ()
{
  static TypeId
    tid =
    TypeId ("ns3::MmWaveEnbNetDevice")
    .SetParent<MmWaveNetDevice> ()
    .AddConstructor<MmWaveEnbNetDevice> ()
    .AddAttribute ("LteEnbComponentCarrierManager",
                   "The ComponentCarrierManager associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::m_componentCarrierManager),
                   MakePointerChecker <LteEnbComponentCarrierManager> ())
    .AddAttribute ("LteEnbRrc",
                   "The RRC layer associated with the ENB",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::m_rrc),
                   MakePointerChecker <LteEnbRrc> ())
    .AddAttribute ("E2Termination",
                   "The E2 termination object associated to this node",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::SetE2Termination,
                                                &MmWaveEnbNetDevice::GetE2Termination),
                   MakePointerChecker <E2Termination> ())
    .AddAttribute ("CellId",
                   "Cell Identifier",
                   UintegerValue (0),
                   MakeUintegerAccessor (&MmWaveEnbNetDevice::m_cellId),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("BasicCellId",
                   "Basic cell ID. This is needed to properly loop over neighbors.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MmWaveEnbNetDevice::m_basicCellId),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("E2PdcpCalculator",
                   "The PDCP calculator object for E2 reporting",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::m_e2PdcpStatsCalculator),
                   MakePointerChecker <MmWaveBearerStatsCalculator> ())
    .AddAttribute ("E2Periodicity",
                   "Periodicity of E2 reporting (value in seconds)",
                   DoubleValue (0.05),
                   MakeDoubleAccessor (&MmWaveEnbNetDevice::m_e2Periodicity),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("E2RlcCalculator",
                   "The RLC calculator object for E2 reporting",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::m_e2RlcStatsCalculator),
                   MakePointerChecker <MmWaveBearerStatsCalculator> ())    
    .AddAttribute ("E2DuCalculator",
                   "The DU calculator object for E2 reporting",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveEnbNetDevice::m_e2DuCalculator),
                   MakePointerChecker <MmWavePhyTrace> ())
    .AddAttribute ("EnableCuUpReport",
                   "If true, send CuUpReport",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MmWaveEnbNetDevice::m_sendCuUp),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableCuCpReport",
                   "If true, send CuCpReport",
                   BooleanValue (true),
                   MakeBooleanAccessor (&MmWaveEnbNetDevice::m_sendCuCp),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableDuReport",
                   "If true, send DuReport",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MmWaveEnbNetDevice::m_sendDu),
                   MakeBooleanChecker ())
    .AddAttribute ("ReducedPmValues",
                   "If true, send only a subset of pmValues",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MmWaveEnbNetDevice::m_reducedPmValues),
                   MakeBooleanChecker ())
   .AddAttribute ("EnableE2FileLogging",
                   "If true, force E2 indication generation and write E2 fields in csv file",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MmWaveEnbNetDevice::m_forceE2FileLogging),
                   MakeBooleanChecker ())
    .AddTraceSource("RicControlMessage",
                    "Control messages arriving from xapp", 
                    MakeTraceSourceAccessor (&MmWaveEnbNetDevice::m_ricControlMessageTrace),
                     "ns3::millicar::MmWaveEnbNetDevice::RicControlMessageTracedCallback"
                    ) 
  ;
  return tid;
}

MmWaveEnbNetDevice::MmWaveEnbNetDevice ()
//:m_cellId(0),
// m_Bandwidth (72),
// m_Earfcn(1),
  : m_componentCarrierManager (0),
    m_isConfigured (false),
    m_isReportingEnabled (false),
    m_reducedPmValues (false),
    m_forceE2FileLogging (false),
    m_cuUpFileName (),
    m_cuCpFileName (),
    m_duFileName ()
{
  NS_LOG_FUNCTION (this);
}


MmWaveEnbNetDevice::~MmWaveEnbNetDevice ()
{
  NS_LOG_FUNCTION (this);
}

void
MmWaveEnbNetDevice::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_isConstructed = true;
  UpdateConfig ();
  NS_LOG_DEBUG("Cc configured " << m_ccMap.size());
  for (auto it = m_ccMap.begin (); it != m_ccMap.end (); ++it)
    {
      NS_LOG_DEBUG("Entering ..");
      NS_LOG_DEBUG("Pointer " << it->second);
      it->second->Initialize ();
      NS_LOG_DEBUG("Pointer " << it->second);
    }
  NS_LOG_DEBUG("RRc initializing..");
  m_rrc->Initialize ();
  
  m_componentCarrierManager->Initialize ();

  if (m_sendCuCp == true)
  {
    // connect to callback
    // Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/NotifyMmWaveSinr",
    // MakeBoundCallback (&MmWaveEnbNetDevice::RegisterNewSinrReadingCallback, this));
  }
}

void
MmWaveEnbNetDevice::RegisterToMillicarUeTraces(){
  // connect to mac sidelink trace 
    // connect to callback
    bool _retValue = Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveSidelinkPhy/SlSinrReport",
    MakeBoundCallback (&MmWaveEnbNetDevice::RegisterSlSinrReportReadingCallback, this));

    NS_LOG_DEBUG("Registered to sl phy sinr " << _retValue);

    // connect to peer sinr report
    bool _retPeerSinrReportValue = Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveSidelinkPhy/NotifyMillicarPairsSinr",
    MakeBoundCallback (&MmWaveEnbNetDevice::RegisterPeerDevicesSinrReportReadingCallback, this));
}

void
MmWaveEnbNetDevice::DoDispose ()
{
  NS_LOG_FUNCTION (this);

  m_rrc->Dispose ();
  m_rrc = 0;

  m_componentCarrierManager->Dispose ();
  m_componentCarrierManager = 0;
  // MmWaveComponentCarrierEnb::DoDispose() will call DoDispose
  // of its PHY, MAC, FFR and scheduler instance
  for (uint32_t i = 0; i < m_ccMap.size (); i++)
    {
      m_ccMap.at (i)->Dispose ();
      m_ccMap.at (i) = 0;
    }

  MmWaveNetDevice::DoDispose ();
}

void
MmWaveEnbNetDevice::RegisterNewSinrReadingCallback(Ptr<MmWaveEnbNetDevice> netDev, std::string context, uint64_t imsi, uint16_t cellId, long double sinr)
{
  netDev->RegisterNewSinrReading(imsi, cellId, sinr);
}

void
MmWaveEnbNetDevice::RegisterNewSinrReading(uint64_t imsi, uint16_t cellId, long double sinr)
{
  // check if the imsi is connected to this DU
  auto ueMap = m_rrc->GetUeMap();
  bool imsiFound = false;

  for (auto ue : ueMap)
  {
    if (ue.second->GetImsi() == imsi)
    {
      imsiFound = true;
      break;
    }
  }

  if (imsiFound)
  {
    // we only need to save the last value, so we erase if exists already a value nd save the new one
    m_l3sinrMap[imsi][cellId] = sinr;
    NS_LOG_LOGIC (Simulator::Now ().GetSeconds ()
                  << " enbdev " << m_cellId << " UE " << imsi << " report for " << cellId
                  << " SINR " << m_l3sinrMap[imsi][cellId]);
  }
}


// end modification
void
MmWaveEnbNetDevice::RegisterSlSinrReportReadingCallback(Ptr<MmWaveEnbNetDevice> netDev, std::string context, uint16_t localRnti, uint16_t destRnti, uint8_t numSym, uint32_t tbSize, double avgSinr) // , double positionX, double positionY
{
  netDev->RegisterSlSinrReportReading(localRnti, destRnti, numSym, tbSize, avgSinr); // , positionX, positionY
}

void
MmWaveEnbNetDevice::RegisterSlSinrReportReading(uint16_t localRnti, uint16_t destRnti, uint8_t numSym, uint32_t tbSize, double avgSinr) // , double positionX, double positionY
{
  // NS_LOG_FUNCTION(this);
  m_l3sinrMillicarMap[localRnti][destRnti].mcs = numSym;
  m_l3sinrMillicarMap[localRnti][destRnti].sinr = avgSinr;

  // m_l3sinrMillicarMap[localRnti][destRnti].positionX = positionX;
  // m_l3sinrMillicarMap[localRnti][destRnti].positionY = positionY;
  
  // NS_LOG_LOGIC (Simulator::Now ().GetSeconds ()
  //               << " uedev " << localRnti << " report for " << destRnti
  //               << " SINR " << m_l3sinrMillicarMap[localRnti][destRnti].sinr);
}

void
MmWaveEnbNetDevice::RegisterPeerDevicesSinrReportReadingCallback(Ptr<MmWaveEnbNetDevice> netDev, std::string context, uint16_t localRnti, uint64_t destRnti, double avgSinr) // , double positionX, double positionY
{
  netDev->RegisterPeerDevicesSinrReportReading(localRnti, destRnti, avgSinr); // , positionX, positionY
}

void
MmWaveEnbNetDevice::RegisterPeerDevicesSinrReportReading(uint16_t localRnti, uint64_t destRnti, double avgSinr) // , double positionX, double positionY
{
  // NS_LOG_FUNCTION(this);
  m_pairsSinrMillicarMap[localRnti][destRnti].mcs = 0;
  m_pairsSinrMillicarMap[localRnti][destRnti].sinr = avgSinr;
}

// modified

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy (void) const
{
  // NS_LOG_FUNCTION (this);
  return DynamicCast<MmWaveComponentCarrierEnb> (m_ccMap.at (0))->GetPhy ();
}

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy (uint8_t index)
{
  return DynamicCast<MmWaveComponentCarrierEnb> (m_ccMap.at (index))->GetPhy ();
}


uint16_t
MmWaveEnbNetDevice::GetCellId () const
{
  // NS_LOG_FUNCTION (this);
  return m_cellId;
}

bool
MmWaveEnbNetDevice::HasCellId (uint16_t cellId) const
{
  for (auto &it : m_ccMap)
    {

      if (DynamicCast<MmWaveComponentCarrierEnb> (it.second)->GetCellId () == cellId)
        {
          return true;
        }
    }
  return false;
}

uint8_t
MmWaveEnbNetDevice::GetBandwidth () const
{
  NS_LOG_FUNCTION (this);
  return m_Bandwidth;
}

void
MmWaveEnbNetDevice::SetBandwidth (uint8_t bw)
{
  NS_LOG_FUNCTION (this << bw);
  m_Bandwidth = bw;
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac (void)
{
  return DynamicCast<MmWaveComponentCarrierEnb> (m_ccMap.at (0))->GetMac ();
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac (uint8_t index)
{
  return DynamicCast<MmWaveComponentCarrierEnb> (m_ccMap.at (index))->GetMac ();
}

void
MmWaveEnbNetDevice::SetRrc (Ptr<LteEnbRrc> rrc)
{
  m_rrc = rrc;
}

Ptr<LteEnbRrc>
MmWaveEnbNetDevice::GetRrc (void)
{
  return m_rrc;
}

bool
MmWaveEnbNetDevice::DoSend (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION (this << packet   << dest << protocolNumber);
  NS_ABORT_MSG_IF (protocolNumber != Ipv4L3Protocol::PROT_NUMBER
                   && protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                   "unsupported protocol " << protocolNumber << ", only IPv4/IPv6 is supported");
  return m_rrc->SendData (packet);
}

void
MmWaveEnbNetDevice::UpdateConfig (void)
{
  NS_LOG_FUNCTION (this);

  if (m_isConstructed)
    {
      if (!m_isConfigured)
        {
          NS_LOG_LOGIC (this << " Configure cell " << m_cellId);
          // we have to make sure that this function is called only once
          //m_rrc->ConfigureCell (m_Bandwidth, m_Bandwidth, m_Earfcn, m_Earfcn, m_cellId);
          NS_ASSERT (!m_ccMap.empty ());

          // create the MmWaveComponentCarrierConf map used for the RRC setup
          std::map<uint8_t, LteEnbRrc::MmWaveComponentCarrierConf> ccConfMap;
          for (auto it = m_ccMap.begin (); it != m_ccMap.end (); ++it)
            {
              Ptr<MmWaveComponentCarrierEnb> ccEnb = DynamicCast<MmWaveComponentCarrierEnb> (it->second);
              LteEnbRrc::MmWaveComponentCarrierConf ccConf;
              ccConf.m_ccId = ccEnb->GetConfigurationParameters ()->GetCcId ();
              ccConf.m_cellId = ccEnb->GetCellId ();
              ccConf.m_bandwidth = ccEnb->GetBandwidthInRb ();

              ccConfMap[it->first] = ccConf;
            }

          m_rrc->ConfigureCell (ccConfMap);

          // trigger E2Termination activation for when the simulation starts
          // schedule at start time
          if (m_e2term != nullptr)
            {
              NS_LOG_DEBUG("E2sim start in cell " << m_cellId 
                << " force CSV logging " << m_forceE2FileLogging);              


              if(!m_forceE2FileLogging) {
                Simulator::Schedule (MicroSeconds (0), &E2Termination::Start, m_e2term);
              }
              // else {
              //   m_cuUpFileName = "cu-up-cell-" + std::to_string(m_cellId) + ".txt";
              //   std::ofstream csv {};
              //   csv.open (m_cuUpFileName.c_str ());
              //   csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
              //          "m_pDCPBytesUL (0),"
              //          "m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
              //          "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID"
              //          "(pdcpThroughput),"
              //          "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
              //          "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
              //   csv.close ();
              //   m_cuCpFileName = "cu-cp-cell-" + std::to_string(m_cellId) + ".txt";
              //   csv.open (m_cuCpFileName.c_str ());
              //   csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
              //          "DRB.RelActNbr.5QI.UEID (0),L3 serving Id(m_cellId),UE (imsi),L3 serving SINR,"
              //          "L3 serving SINR 3gpp,"
              //          "L3 neigh Id 1 (cellId),L3 neigh SINR 1,L3 neigh SINR 3gpp 1 (convertedSinr),"
              //          "L3 neigh Id 2 (cellId),L3 neigh SINR 2,L3 neigh SINR 3gpp 2 (convertedSinr),"
              //          "L3 neigh Id 3 (cellId),L3 neigh SINR 3,L3 neigh SINR 3gpp 3 (convertedSinr),"
              //          "L3 neigh Id 4 (cellId),L3 neigh SINR 4,L3 neigh SINR 3gpp 4 (convertedSinr),"
              //          "L3 neigh Id 5 (cellId),L3 neigh SINR 5,L3 neigh SINR 3gpp 5 (convertedSinr),"
              //          "L3 neigh Id 6 (cellId),L3 neigh SINR 6,L3 neigh SINR 3gpp 6 (convertedSinr),"
              //          "L3 neigh Id 7 (cellId),L3 neigh SINR 7,L3 neigh SINR 3gpp 7 (convertedSinr),"
              //          "L3 neigh Id 8 (cellId),L3 neigh SINR 8,L3 neigh SINR 3gpp 8 (convertedSinr)"
              //          "\n";
              //   csv.close();

              //   m_duFileName = "du-cell-" + std::to_string(m_cellId) + ".txt";
              //   csv.open (m_duFileName.c_str ());
                
              //   std::string header_csv =
              //       "timestamp,ueImsiComplete,plmId,nrCellId,dlAvailablePrbs,"
              //       "ulAvailablePrbs,qci,dlPrbUsage,ulPrbUsage";

              //   std::string cell_header =
              //       "TB.TotNbrDl.1,TB.TotNbrDlInitial,TB.TotNbrDlInitial.Qpsk,"
              //       "TB.TotNbrDlInitial.16Qam,"
              //       "TB.TotNbrDlInitial.64Qam,RRU.PrbUsedDl,TB.ErrTotalNbrDl.1,"
              //       "QosFlow.PdcpPduVolumeDL_Filter,CARR.PDSCHMCSDist.Bin1,"
              //       "CARR.PDSCHMCSDist.Bin2,"
              //       "CARR.PDSCHMCSDist.Bin3,CARR.PDSCHMCSDist.Bin4,CARR.PDSCHMCSDist.Bin5,"
              //       "CARR.PDSCHMCSDist.Bin6,L1M.RS-SINR.Bin34,L1M.RS-SINR.Bin46, "
              //       "L1M.RS-SINR.Bin58,"
              //       "L1M.RS-SINR.Bin70,L1M.RS-SINR.Bin82,L1M.RS-SINR.Bin94,L1M.RS-SINR.Bin127,"
              //       "DRB.BufferSize.Qos,DRB.MeanActiveUeDl";

              //   std::string ue_header =
              //       "TB.TotNbrDl.1.UEID,TB.TotNbrDlInitial.UEID,TB.TotNbrDlInitial.Qpsk.UEID,"
              //       "TB.TotNbrDlInitial.16Qam.UEID,TB.TotNbrDlInitial.64Qam.UEID,"
              //       "TB.ErrTotalNbrDl.1.UEID,"
              //       "QosFlow.PdcpPduVolumeDL_Filter.UEID,RRU.PrbUsedDl.UEID,"
              //       "CARR.PDSCHMCSDist.Bin1.UEID,"
              //       "CARR.PDSCHMCSDist.Bin2.UEID,CARR.PDSCHMCSDist.Bin3.UEID,"
              //       "CARR.PDSCHMCSDist.Bin4.UEID,"
              //       "CARR.PDSCHMCSDist.Bin5.UEID,"
              //       "CARR.PDSCHMCSDist.Bin6.UEID,L1M.RS-SINR.Bin34.UEID, L1M.RS-SINR.Bin46.UEID,"
              //       "L1M.RS-SINR.Bin58.UEID,L1M.RS-SINR.Bin70.UEID,L1M.RS-SINR.Bin82.UEID,"
              //       "L1M.RS-SINR.Bin94.UEID,L1M.RS-SINR.Bin127.UEID,DRB.BufferSize.Qos.UEID,"
              //       "DRB.UEThpDl.UEID, DRB.UEThpDlPdcpBased.UEID";

              //   csv << header_csv + "," + cell_header + "," + ue_header + "\n";
              //   csv.close();
              //   Simulator::Schedule(MicroSeconds(500), &MmWaveEnbNetDevice::BuildAndSendReportMessage, this, E2Termination::RicSubscriptionRequest_rval_s{});
              // }

            }
          m_isConfigured = true;
        }

      //m_rrc->SetCsgId (m_csgId, m_csgIndication);
    }
  else
    {
      /*
      * Lower layers are not ready yet, so do nothing now and expect
      * ``DoInitialize`` to re-invoke this function.
      */
    }

    // schedule sending the reports from end devices
    // Simulator::Schedule(MilliSeconds(40), &MmWaveEnbNetDevice::BuildAndSendMillicarReportMessage, this, E2Termination::RicSubscriptionRequest_rval_s{});
}

void
MmWaveEnbNetDevice::SetCcMap (std::map< uint8_t, Ptr<MmWaveComponentCarrier> > ccm)
{
  NS_ASSERT_MSG (!m_isConfigured, "attempt to set CC map after configuration");
  m_ccMap = ccm;
}

Ptr<E2Termination>
MmWaveEnbNetDevice::GetE2Termination() const
{
  return m_e2term;
}

void
MmWaveEnbNetDevice::AddToDoHandoverTrace(uint16_t sourceCellId, uint64_t imsi, uint16_t destinationCellId){
  NS_LOG_FUNCTION(this << sourceCellId << imsi << destinationCellId);
  // m_doHandoverTrace(sourceCellId, imsi, destinationCellId);

  // source, destination, intermediate rnti
  m_ricControlMessageTrace(12, 12, 12);
}

void 
MmWaveEnbNetDevice::DoHandoverTrace (E2AP_PDU_t* ric_pdu)
{
  NS_LOG_FUNCTION(this);
  // here we receive a control message from the xapp, which we have to decode and actuate upon it
  Ptr<RicControlMessage> controlMessage = Create<RicControlMessage> (ric_pdu);
  NS_LOG_INFO ("\n Message type received " << controlMessage->m_messageFormatType << "\n");
  // xer_fprint(stderr, &asn_DEF_E2AP_PDU, ric_pdu);
  if (controlMessage->m_messageFormatType == RicControlMessage::ControlMessage_PR_handoverMessage_Format){
    std::map<long, std::map<long, long>> resultMap = controlMessage->m_allHandoverList;
    NS_LOG_DEBUG("The map size " << resultMap.size());
    // NS_LOG_DEBUG(resultMap);
    // create a function to actuate the handover action
    // auto mapIt = resultMap.find((long)m_cellId);
    for (std::map<long, std::map<long, long>>::iterator mapIt = resultMap.begin(); 
      mapIt!=resultMap.end(); ++mapIt){
      for(std::map<long,long>::iterator singleUser = mapIt->second.begin(); singleUser!=mapIt->second.end(); ++singleUser){
        uint64_t ueId = (uint64_t) singleUser->first;
        uint16_t destinationCellid = (uint16_t) singleUser->second;
          NS_LOG_INFO("User id " << ueId << " has to be moved to cell id " << destinationCellid);
          Simulator::ScheduleWithContext(1, Seconds(0), 
            &MmWaveEnbNetDevice::AddToDoHandoverTrace, this, (uint16_t)mapIt->first, ueId, destinationCellid);
        
      }
    }
    // if(mapIt != resultMap.end()){
    // }
  }
}

/**
* Service model register callback.
* This function is triggered whenever a message 
* from that service model (function id) is received.
*
* \param pdu request message
*/
void 
MmWaveEnbNetDevice::MillicarServiceModelRegisterCallback (E2AP_PDU_t* sm_pdu)
{
  NS_LOG_FUNCTION(this);
  NS_LOG_DEBUG ("\nReceived service model response, cellId= " << m_cellId << "\n");

  DoHandoverTrace(sm_pdu);

  // relay the message to the right ue
  // uint8_t *buf;

  // int len = e2ap_asn1c_encode_pdu(sm_pdu, &buf);
  // NS_LOG_DEBUG ("Len of decoding  " << len << "\n");

  // m_ricControlMessageTrace(12, 12);

  // for (std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator ueReportIt = m_l3sinrMillicarMap.begin();
  //   ueReportIt != m_l3sinrMillicarMap.end(); ++ ueReportIt){
      // Simulator::ScheduleNow (&LteEnbRrc::SendE2MessageBuffer,
      //                                         m_rrc, ueReportIt->first, buf, len);
      // m_rrc->SendE2MessageBuffer( ueReportIt->first, buf, len);

    // }
}



void
MmWaveEnbNetDevice::SetE2Termination(Ptr<E2Termination> e2term)
{
  NS_LOG_FUNCTION(this);
  m_e2term = e2term;

  if (!m_forceE2FileLogging)
    {
      Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription> ();
      e2term->RegisterKpmCallbackToE2Sm (
          200, kpmFd,
          std::bind (&MmWaveEnbNetDevice::KpmSubscriptionCallback, this, std::placeholders::_1));
    
      // register to service model
      Ptr<FunctionDescription> fd = Create<FunctionDescription> ();
      e2term->RegisterSmCallbackToE2Sm(300, fd, 
            std::bind (&MmWaveEnbNetDevice::MillicarServiceModelRegisterCallback, this, std::placeholders::_1));
    }
}

std::string
MmWaveEnbNetDevice::GetImsiString(uint64_t imsi)
{
  std::string ueImsi = std::to_string(imsi);
  std::string ueImsiComplete {};
  if (ueImsi.length() == 1)
  {
    ueImsiComplete = "0000" + ueImsi;
  }
  else if (ueImsi.length() == 2)
  {
    ueImsiComplete = "000" + ueImsi;
  }
  else
  {
    ueImsiComplete = "00" + ueImsi;
  }
  return ueImsiComplete;
}

Ptr<KpmIndicationHeader>
MmWaveEnbNetDevice::BuildRicIndicationHeader (std::string plmId, std::string gnbId,
                                              uint16_t nrCellId)
{
  if (!m_forceE2FileLogging)
    {
      KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
      headerValues.m_plmId = plmId;
      headerValues.m_gnbId = gnbId;
      headerValues.m_nrCellId = nrCellId;
      auto time = Simulator::Now ();
      uint64_t timestamp = m_startTime + (uint64_t) time.GetMilliSeconds ();
      NS_LOG_DEBUG ("NR plmid " << plmId << " gnbId " << gnbId << " nrCellId " << nrCellId);
      NS_LOG_DEBUG ("Timestamp " << timestamp);
      headerValues.m_timestamp = timestamp;

      Ptr<KpmIndicationHeader> header =
          Create<KpmIndicationHeader> (KpmIndicationHeader::GlobalE2nodeType::gNB, headerValues);

      return header;
    }
  else
    {
      return nullptr;
    }
}

Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageCuUp(std::string plmId)
{
  Ptr<MmWaveIndicationMessageHelper> indicationMessageHelper =
      Create<MmWaveIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuUp,
                                             m_forceE2FileLogging, m_reducedPmValues);

  // get <rnti, UeManager> map of connected UEs
  auto ueMap = m_rrc->GetUeMap();
  // gNB-wide PDCP volume in downlink
  double cellDlTxVolume = 0;
  // rx bytes in downlink
  double cellDlRxVolume = 0;

  // sum of the per-user average latency
  double perUserAverageLatencySum = 0;

  std::unordered_map<uint64_t, std::string> uePmString {};

  for (auto ue : ueMap)
  {
    uint64_t imsi = ue.second->GetImsi();
    std::string ueImsiComplete = GetImsiString (imsi);

    // double rxDlPackets = m_e2PdcpStatsCalculator->GetDlRxPackets(imsi, 3); // LCID 3 is used for data
    long txDlPackets = m_e2PdcpStatsCalculator->GetDlTxPackets(imsi, 3); // LCID 3 is used for data
    double txBytes = m_e2PdcpStatsCalculator->GetDlTxData(imsi, 3)  * 8 / 1e3; // in kbit, not byte
    double rxBytes = m_e2PdcpStatsCalculator->GetDlRxData(imsi, 3)  * 8 / 1e3; // in kbit, not byte
    cellDlTxVolume += txBytes;
    cellDlRxVolume += rxBytes;

    long txPdcpPduNrRlc = 0;
    double txPdcpPduBytesNrRlc = 0;

    auto drbMap = ue.second->GetDrbMap();
    for (auto drb : drbMap)
    {
      txPdcpPduNrRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
      txPdcpPduBytesNrRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
      drb.second->m_rlc->ResetRlcCounters();
    }

    auto rlcMap = ue.second->GetRlcMap(); // secondary-connected RLCs
    for (auto drb : rlcMap)
    {
      txPdcpPduNrRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
      txPdcpPduBytesNrRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
      drb.second->m_rlc->ResetRlcCounters();
    }
    txPdcpPduBytesNrRlc *= 8 / 1e3;

    double pdcpLatency = m_e2PdcpStatsCalculator->GetDlDelay(imsi, 3) / 1e5; // unit: x 0.1 ms
    perUserAverageLatencySum += pdcpLatency;

    double pdcpThroughput = txBytes / m_e2Periodicity; // unit kbps
    double pdcpThroughputRx = rxBytes / m_e2Periodicity; // unit kbps
    
    if (m_drbThrDlPdcpBasedComputationUeid.find (imsi) != m_drbThrDlPdcpBasedComputationUeid.end ())
    {
      m_drbThrDlPdcpBasedComputationUeid.at (imsi) += pdcpThroughputRx; 
    }
    else
    {
      m_drbThrDlPdcpBasedComputationUeid [imsi] = pdcpThroughputRx; 
    }

    // compute bitrate based on RLC statistics, decoupled from pdcp throughput
    double rlcLatency = m_e2RlcStatsCalculator->GetDlDelay(imsi, 3) / 1e9; // unit: s
    double pduStats = m_e2RlcStatsCalculator->GetDlPduSizeStats(imsi, 3)[0] * 8.0 / 1e3; // unit kbit
    double rlcBitrate = (rlcLatency == 0) ? 0 : pduStats / rlcLatency; // unit kbit/s

    m_drbThrDlUeid [imsi] = rlcBitrate; 
    
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << m_cellId << " cell, connected UE with IMSI " << imsi 
      << " ueImsiString " << ueImsiComplete
      << " txDlPackets " << txDlPackets 
      << " txDlPacketsNr " << txPdcpPduNrRlc
      << " txBytes " << txBytes 
      << " rxBytes " << rxBytes 
      << " txDlBytesNr " << txPdcpPduBytesNrRlc
      << " pdcpLatency " << pdcpLatency 
      << " pdcpThroughput " << pdcpThroughput
      << " rlcBitrate " << rlcBitrate);

    m_e2PdcpStatsCalculator->ResetResultsForImsiLcid (imsi, 3);
    
    if (!indicationMessageHelper->IsOffline ())
      {
        indicationMessageHelper->AddCuUpUePmItem (ueImsiComplete, txPdcpPduBytesNrRlc,
                                                  txPdcpPduNrRlc);
      }

    uePmString.insert(std::make_pair(imsi, ",,,," + std::to_string(txPdcpPduBytesNrRlc) + "," +
      std::to_string(txPdcpPduNrRlc)));
  }

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->FillCuUpValues (plmId);
    }

  NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << m_cellId << " cell volume " << cellDlTxVolume);

  if (m_forceE2FileLogging)
    {
      std::ofstream csv{};
      csv.open (m_cuUpFileName.c_str (), std::ios_base::app);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_cuUpFileName.c_str ());
        }

      uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

      // the string is timestamp, ueImsiComplete, DRB.PdcpSduDelayDl (cellAverageLatency),
      // m_pDCPBytesUL (0), m_pDCPBytesDL (cellDlTxVolume), DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
      // Tot.PdcpSduNbrDl.UEID (txDlPackets), DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
      // DRB.PdcpSduDelayDl.UEID (pdcpLatency), QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
      // DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)

      for (auto ue : ueMap)
        {
          uint64_t imsi = ue.second->GetImsi ();
          std::string ueImsiComplete = GetImsiString (imsi);

          auto uePms = uePmString.find (imsi)->second;

          std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," + "," +
                                 "," + "," + uePms + "\n";

          csv << to_print;
        }
      csv.close ();
      return nullptr;
    }
  else
    {
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}

template <typename A, typename B>
std::pair<B, A>
flip_pair (const std::pair<A, B> &p)
{
  return std::pair<B, A> (p.second, p.first);
}

template <typename A, typename B> // , typename C
std::multimap<B, A> // , C
flip_map (const std::map<A, B> &src)
{
  std::multimap<B, A> dst; // , C
  std::transform (src.begin (), src.end (), std::inserter (dst, dst.begin ()), flip_pair<A, B>);
  return dst;
}

Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageCuCp(std::string plmId)
{
  Ptr<MmWaveIndicationMessageHelper> indicationMessageHelper =
      Create<MmWaveIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                             m_forceE2FileLogging, m_reducedPmValues);

  auto ueMap = m_rrc->GetUeMap();

  std::unordered_map<uint64_t, std::string> uePmString {};

  for (auto ue : ueMap)
  {
    uint64_t imsi = ue.second->GetImsi();
    std::string ueImsiComplete = GetImsiString(imsi);
    

    Ptr<MeasurementItemList> ueVal = Create<MeasurementItemList> (ueImsiComplete);

    long numDrb = ue.second->GetDrbMap().size();

    if (!m_reducedPmValues)
      {    
        ueVal->AddItem<long> ("DRB.EstabSucc.5QI.UEID", numDrb);
        ueVal->AddItem<long> ("DRB.RelActNbr.5QI.UEID", 0); // not modeled in the simulator
      } 

    // create L3 RRC reports

    // for the same cell
    double sinrThisCell = 10 * std::log10 (m_l3sinrMap[imsi][m_cellId]);
    double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);

    Ptr<L3RrcMeasurements> l3RrcMeasurementServing;
    if (!indicationMessageHelper->IsOffline ())
      {
        l3RrcMeasurementServing =
            L3RrcMeasurements::CreateL3RrcUeSpecificSinrServing (m_cellId, m_cellId, convertedSinr);
      }
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << " enbdev " << m_cellId << " UE " << imsi << " L3 serving SINR " << sinrThisCell
                  << " L3 serving SINR 3gpp " << convertedSinr);

    std::string servingStr = std::to_string (numDrb) + "," + std::to_string (0) + "," +
                             std::to_string (m_cellId) + "," + std::to_string (imsi) + "," +
                             std::to_string (sinrThisCell) + "," + std::to_string (convertedSinr);

    // For the neighbors
    // TODO create double map, imsi -> cell -> sinr
    // TODO store at most 8 reports for each UE, as per the standard

    Ptr<L3RrcMeasurements> l3RrcMeasurementNeigh;
    if (!indicationMessageHelper->IsOffline ())
      {
        l3RrcMeasurementNeigh = L3RrcMeasurements::CreateL3RrcUeSpecificSinrNeigh ();
      }
    double sinr;
    std::string neighStr;

    //invert key and value in sortFlipMap, then sort by value
    std::multimap<long double, uint16_t> sortFlipMap = flip_map (m_l3sinrMap[imsi]);
    //new sortFlipMap structure sortFlipMap < sinr, cellId >
    //The assumption is that the first cell in the scenario is always LTE and the rest NR
    uint16_t nNeighbours = E2SM_REPORT_MAX_NEIGH;
    if (m_l3sinrMap[imsi].size () < nNeighbours)
      {
        nNeighbours = m_l3sinrMap[imsi].size () - 1;
      }
    int itIndex = 0;
    // Save only the first E2SM_REPORT_MAX_NEIGH SINR for each UE which represent the best values among all the SINRs detected by all the cells
    for (std::map<long double, uint16_t>::iterator it = --sortFlipMap.end ();
         it != --sortFlipMap.begin () && itIndex < nNeighbours; it--)
      {
        uint16_t cellId = it->second;
        if (cellId != m_cellId)
          {
            sinr = 10 * std::log10 (it->first); // now SINR is a key due to the sort of the map
            convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinr);
            if (!indicationMessageHelper->IsOffline ())
              {
                l3RrcMeasurementNeigh->AddNeighbourCellMeasurement (cellId, convertedSinr);
              }
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << " enbdev " << m_cellId << " UE " << imsi << " L3 neigh " << cellId
                          << " SINR " << sinr << " sinr encoded " << convertedSinr
                          << " first insert");
            neighStr += "," + std::to_string (cellId) + "," + std::to_string (sinr) + "," +
                        std::to_string (convertedSinr);
            itIndex++;
          }
      }
      for (int i =nNeighbours; i< E2SM_REPORT_MAX_NEIGH; i++){
        neighStr += ",,,";
      }
    

    uePmString.insert (std::make_pair (imsi, servingStr + neighStr));

    if (!indicationMessageHelper->IsOffline ())
      {
        indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, numDrb, 0,
                                                  l3RrcMeasurementServing, l3RrcMeasurementNeigh);
      }
  }

  if (!indicationMessageHelper->IsOffline ())
    {
      // Fill CuCp specific fields
      indicationMessageHelper->FillCuCpValues (ueMap.size ()); // Number of Active UEs
    }

  if (m_forceE2FileLogging)
    {
      std::ofstream csv{};
      csv.open (m_cuCpFileName.c_str (), std::ios_base::app);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_cuCpFileName.c_str ());
        }

      NS_LOG_DEBUG ("m_cuCpFileName open " << m_cuCpFileName);

      // the string is timestamp, ueImsiComplete, numActiveUes, DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0), L3 serving Id (m_cellId), UE (imsi), L3 serving SINR, L3 serving SINR 3gpp, L3 neigh Id (cellId), L3 neigh Sinr, L3 neigh SINR 3gpp (convertedSinr)
      // The values for L3 neighbour cells are repeated for each neighbour (7 times in this implementation)

      uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

      for (auto ue : ueMap)
        {
          uint64_t imsi = ue.second->GetImsi ();
          std::string ueImsiComplete = GetImsiString (imsi);

          auto uePms = uePmString.find (imsi)->second;

          std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                                 std::to_string (ueMap.size()) + "," + uePms + "\n";

          NS_LOG_DEBUG (to_print);

          csv << to_print;
        }
      csv.close ();
      return nullptr;
    }
  else
    {
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}


Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildMillicarReportRicIndicationMessageCucp(std::string plmId, uint16_t generatingNodeRnti, uint16_t numReports)
{
  NS_LOG_FUNCTION(this);
  Ptr<MmWaveIndicationMessageHelper> indicationMessageHelper =
      Create<MmWaveIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                             m_forceE2FileLogging, m_reducedPmValues);
    uint64_t imsi = (uint64_t) generatingNodeRnti;
    std::string ueImsiComplete = GetImsiString(imsi);
    // get the position of the node
    std::map<uint16_t, Ptr<MmWaveUeNetDevice>>::iterator millicarDevice = m_millicarDevicesMap.find(generatingNodeRnti);
    double positionX = -1;
    double positionY = -1;
    if (millicarDevice!= m_millicarDevicesMap.end()){
      Ptr<MobilityModel> ueMobility = millicarDevice->second->GetNode()->GetObject<MobilityModel>();
      positionX = ueMobility->GetPosition().x;
      positionY = ueMobility->GetPosition().y;
    }
    
    // double sinrThis = 10 * std::log10 (m_l3sinrMillicarMap[generatingNodeRnti][generatingNodeRnti].sinr);
    Ptr<L3RrcMeasurements> l3RrcMeasurementServing;
    Ptr<ServingCellMeasurementsWrap> servingCellMeasurements; 

    std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator firstOrderAssignIt = m_l3sinrMillicarMap.find(generatingNodeRnti);
    if(firstOrderAssignIt!= m_l3sinrMillicarMap.end()){
      // we might have multiple users a peer is communicating, thus we need to go through them
      std::multimap<SinrMcsPair, uint16_t> sortFlipMap = flip_map (firstOrderAssignIt->second); // SinrMcsPairCmp
      // uint16_t nNeighbours = E2SM_REPORT_MAX_NEIGH;
       uint16_t nNeighbours = 1;
      if (firstOrderAssignIt->second.size () < nNeighbours)
      {
        nNeighbours = firstOrderAssignIt->second.size () - 1;
      }
      uint16_t itIndex = 0;
      bool _sinrDataExist = false;
      NS_LOG_DEBUG("Found measurement data for rnti " << generatingNodeRnti << "  with size " << firstOrderAssignIt->second.size());
      if (!indicationMessageHelper->IsOffline ()){ 
        // l3RrcMeasurementServing = L3RrcMeasurements::CreateL3RrcUeSpecificPeerDevicesMeasurements ();
        l3RrcMeasurementServing = Create<L3RrcMeasurements> (RRCEvent_b1);
        servingCellMeasurements = Create<ServingCellMeasurementsWrap> (ServingCellMeasurements_PR_nr_measResultServingMOList); 
      
        _sinrDataExist = sortFlipMap.size() >0;

        NS_LOG_DEBUG("Size of nNeighbours " << nNeighbours << " size of flip " << sortFlipMap.size());
        for (std::multimap<SinrMcsPair, uint16_t>::iterator it = sortFlipMap.begin ();
            (it != sortFlipMap.end ()) && (itIndex < nNeighbours); it++)
        {
          NS_LOG_DEBUG("Entering the loop to write individual reporst");
          
          NS_LOG_DEBUG("Adding Peer device with rnti " << it->second << " sinr " << it->first.sinr
          << " and mcs " << (uint32_t)it->first.mcs);
          double sinrThisCell = 10 * std::log10 (it->first.sinr);
          double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
          servingCellMeasurements->AddMeasResultServMo (L3RrcMeasurements::CreateActivePeerDeviceSinrMcs(it->second, convertedSinr,it->first.mcs)->GetPointer());
          _sinrDataExist =true;
          itIndex++;
        }

        if(_sinrDataExist){
          l3RrcMeasurementServing->AddServingCellMeasurement (servingCellMeasurements->GetPointer ());
        }
      }
      
      
    }
    
    Ptr<L3RrcMeasurements> l3RrcMeasurementNeigh;
    
    std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator firstOrderIt = m_pairsSinrMillicarMap.find(generatingNodeRnti);
    NS_LOG_DEBUG("Size of neighbour " << firstOrderIt->second.size());
    if(firstOrderIt!= m_pairsSinrMillicarMap.end()){
        if (!indicationMessageHelper->IsOffline ())
        {
          l3RrcMeasurementNeigh = L3RrcMeasurements::CreateL3RrcUeSpecificSinrNeigh ();
        }
        //invert key and value in sortFlipMap, then sort by value
        std::multimap<SinrMcsPair, uint16_t> sortFlipMap = flip_map (m_pairsSinrMillicarMap[generatingNodeRnti]); // SinrMcsPairCmp
        uint16_t nNeighbours = E2SM_REPORT_MAX_NEIGH;
        if (m_pairsSinrMillicarMap[generatingNodeRnti].size () < nNeighbours)
          {
            nNeighbours = m_pairsSinrMillicarMap[generatingNodeRnti].size ();
          }
        int itIndex = 0;
        for (std::map<SinrMcsPair, uint16_t>::iterator it = sortFlipMap.begin ();
          it != sortFlipMap.end () && itIndex < nNeighbours; it++)
        {
          if (!indicationMessageHelper->IsOffline ())
          {
            
            double sinrPeer = 10 * std::log10 (it->first.sinr);
            double convertedSinrPeer = L3RrcMeasurements::ThreeGppMapSinr (sinrPeer);
            NS_LOG_DEBUG("Rnti " << it->second << " converted sinr " << convertedSinrPeer << " & mcs " << +it->first.mcs);
            l3RrcMeasurementNeigh->AddNeighbourCellMeasurementMcs (it->second, convertedSinrPeer, it->first.mcs);
          }
          itIndex++;
        }
    }
    if (!indicationMessageHelper->IsOffline ()){
      // indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, generatingNodeRnti, l3RrcMeasurementNeigh);
      indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, generatingNodeRnti, positionX, 
                                                positionY, l3RrcMeasurementServing, l3RrcMeasurementNeigh);
    }

    if (!indicationMessageHelper->IsOffline ())
    {
      // Fill CuCp specific fields
      indicationMessageHelper->FillCuCpValues (numReports); // Number of Active UEs
    }

    if (m_forceE2FileLogging)
    {
      return nullptr;
    }else{
      return indicationMessageHelper->CreateIndicationMessage ();
    }

}

uint32_t 
MmWaveEnbNetDevice::GetRlcBufferOccupancy(Ptr<LteRlc> rlc) const
{
  if (DynamicCast<LteRlcAm>(rlc) != nullptr)
  {
    return DynamicCast<LteRlcAm>(rlc)->GetTxBufferSize(); 
  }
  else if(DynamicCast<LteRlcUm>(rlc) != nullptr)
  {
    return DynamicCast<LteRlcUm>(rlc)->GetTxBufferSize(); 
  }
  else if(DynamicCast<LteRlcUmLowLat>(rlc) != nullptr)
  {
    return DynamicCast<LteRlcUmLowLat>(rlc)->GetTxBufferSize(); 
  }
  else 
  {
    return 0;
  }
}


Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId)
{
  Ptr<MmWaveIndicationMessageHelper> indicationMessageHelper =
      Create<MmWaveIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::Du,
                                             m_forceE2FileLogging, m_reducedPmValues);
  
  auto ueMap = m_rrc->GetUeMap();

  uint32_t macPduCellSpecific = 0;
  uint32_t macPduInitialCellSpecific = 0;
  uint32_t macVolumeCellSpecific = 0;
  uint32_t macQpskCellSpecific = 0;
  uint32_t mac16QamCellSpecific = 0;
  uint32_t mac64QamCellSpecific = 0;
  uint32_t macRetxCellSpecific = 0;
  uint32_t macMac04CellSpecific = 0;
  uint32_t macMac59CellSpecific = 0;
  uint32_t macMac1014CellSpecific = 0;
  uint32_t macMac1519CellSpecific = 0;
  uint32_t macMac2024CellSpecific = 0;
  uint32_t macMac2529CellSpecific = 0;

  uint32_t macSinrBin1CellSpecific = 0;
  uint32_t macSinrBin2CellSpecific = 0;
  uint32_t macSinrBin3CellSpecific = 0;
  uint32_t macSinrBin4CellSpecific = 0;
  uint32_t macSinrBin5CellSpecific = 0;
  uint32_t macSinrBin6CellSpecific = 0;
  uint32_t macSinrBin7CellSpecific = 0;
  
  uint32_t rlcBufferOccupCellSpecific = 0;

  uint32_t macPrbsCellSpecific = 0;

  std::unordered_map<uint64_t, std::string> uePmStringDu {};
  
  for (auto ue : ueMap)
  {
    uint64_t imsi = ue.second->GetImsi();
    std::string ueImsiComplete = GetImsiString(imsi);
    uint16_t rnti = ue.second->GetRnti();

    uint32_t macPduUe = m_e2DuCalculator->GetMacPduUeSpecific(rnti, m_cellId);

    macPduCellSpecific += macPduUe;
 
    uint32_t macPduInitialUe = m_e2DuCalculator->GetMacPduInitialTransmissionUeSpecific(rnti, m_cellId);
    macPduInitialCellSpecific += macPduInitialUe;

    uint32_t macVolume = m_e2DuCalculator->GetMacVolumeUeSpecific(rnti, m_cellId);
    macVolumeCellSpecific += macVolume;

    uint32_t macQpsk = m_e2DuCalculator->GetMacPduQpskUeSpecific(rnti, m_cellId);
    macQpskCellSpecific += macQpsk;

    uint32_t mac16Qam = m_e2DuCalculator->GetMacPdu16QamUeSpecific(rnti, m_cellId);
    mac16QamCellSpecific += mac16Qam;

    uint32_t mac64Qam = m_e2DuCalculator->GetMacPdu64QamUeSpecific(rnti, m_cellId);
    mac64QamCellSpecific += mac64Qam;

    uint32_t macRetx = m_e2DuCalculator->GetMacPduRetransmissionUeSpecific(rnti, m_cellId);
    macRetxCellSpecific += macRetx;

    // Numerator = (Sum of number of symbols across all rows (TTIs) group by cell ID and UE ID within a given time window)
    double macNumberOfSymbols = m_e2DuCalculator->GetMacNumberOfSymbolsUeSpecific(rnti, m_cellId);
    
    auto phyMac = GetMac()->GetConfigurationParameters();
    // Denominator = (Periodicity of the report time window in ms*number of TTIs per ms*14)
    Time reportingWindow = Simulator::Now () - m_e2DuCalculator->GetLastResetTime (rnti, m_cellId);
    double denominatorPrb = std::ceil (reportingWindow.GetNanoSeconds () / phyMac->GetSlotPeriod ().GetNanoSeconds ()) * 14; 
    
    NS_LOG_DEBUG ("macNumberOfSymbols " << macNumberOfSymbols << " denominatorPrb " << denominatorPrb);

    // Average Number of PRBs allocated for the UE = (NR/DR)*139 (where 139 is the total number of PRBs available per NR cell, given numerology 2 with 60 kHz SCS)
    double macPrb = 0;
    if (denominatorPrb != 0)
    {
      macPrb = macNumberOfSymbols / denominatorPrb * 139; // TODO fix this for different numerologies
    }
    macPrbsCellSpecific += macPrb;

    uint32_t macMac04 = m_e2DuCalculator->GetMacMcs04UeSpecific(rnti, m_cellId);
    macMac04CellSpecific += macMac04;

    uint32_t macMac59 = m_e2DuCalculator->GetMacMcs59UeSpecific(rnti, m_cellId);
    macMac59CellSpecific += macMac59;

    uint32_t macMac1014 = m_e2DuCalculator->GetMacMcs1014UeSpecific(rnti, m_cellId);
    macMac1014CellSpecific += macMac1014;

    uint32_t macMac1519 = m_e2DuCalculator->GetMacMcs1519UeSpecific(rnti, m_cellId);
    macMac1519CellSpecific += macMac1519;

    uint32_t macMac2024 = m_e2DuCalculator->GetMacMcs2024UeSpecific(rnti, m_cellId);
    macMac2024CellSpecific += macMac2024;

    uint32_t macMac2529 = m_e2DuCalculator->GetMacMcs2529UeSpecific(rnti, m_cellId);
    macMac2529CellSpecific += macMac2529;

    uint32_t macSinrBin1 = m_e2DuCalculator->GetMacSinrBin1UeSpecific(rnti, m_cellId);
    macSinrBin1CellSpecific += macSinrBin1;

    uint32_t macSinrBin2 = m_e2DuCalculator->GetMacSinrBin2UeSpecific(rnti, m_cellId);
    macSinrBin2CellSpecific += macSinrBin2;

    uint32_t macSinrBin3 = m_e2DuCalculator->GetMacSinrBin3UeSpecific(rnti, m_cellId);
    macSinrBin3CellSpecific += macSinrBin3;

    uint32_t macSinrBin4 = m_e2DuCalculator->GetMacSinrBin4UeSpecific(rnti, m_cellId);
    macSinrBin4CellSpecific += macSinrBin4;

    uint32_t macSinrBin5 = m_e2DuCalculator->GetMacSinrBin5UeSpecific(rnti, m_cellId);
    macSinrBin5CellSpecific += macSinrBin5;

    uint32_t macSinrBin6 = m_e2DuCalculator->GetMacSinrBin6UeSpecific(rnti, m_cellId);
    macSinrBin6CellSpecific += macSinrBin6;

    uint32_t macSinrBin7 = m_e2DuCalculator->GetMacSinrBin7UeSpecific(rnti, m_cellId);
    macSinrBin7CellSpecific += macSinrBin7;
    
    // get buffer occupancy info
    uint32_t rlcBufferOccup = 0;
    auto drbMap = ue.second->GetDrbMap();
    for (auto drb : drbMap)
    {
      auto rlc = drb.second->m_rlc;
      rlcBufferOccup += GetRlcBufferOccupancy(rlc);
    }
    auto rlcMap = ue.second->GetRlcMap(); // secondary-connected RLCs
    for (auto drb : rlcMap)
    {
      auto rlc = drb.second->m_rlc;
      rlcBufferOccup += GetRlcBufferOccupancy(rlc);
    }
    rlcBufferOccupCellSpecific += rlcBufferOccup;

    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << m_cellId << " cell, connected UE with IMSI " << imsi 
        << " rnti " << rnti     
      << " macPduUe " << macPduUe
      << " macPduInitialUe " << macPduInitialUe
      << " macVolume " << macVolume 
      << " macQpsk " << macQpsk
      << " mac16Qam " << mac16Qam
      << " mac64Qam " << mac64Qam
      << " macRetx " << macRetx
      << " macPrb " << macPrb
      << " macMac04 " << macMac04
      << " macMac59 " << macMac59
      << " macMac1014 " << macMac1014
      << " macMac1519 " << macMac1519
      << " macMac2024 " << macMac2024
      << " macMac2529 " << macMac2529
      << " macSinrBin1 " << macSinrBin1
      << " macSinrBin2 " << macSinrBin2
      << " macSinrBin3 " << macSinrBin3
      << " macSinrBin4 " << macSinrBin4
      << " macSinrBin5 " << macSinrBin5
      << " macSinrBin6 " << macSinrBin6
      << " macSinrBin7 " << macSinrBin7
      << " rlcBufferOccup " << rlcBufferOccup
    );

    // UE-specific Downlink IP combined EN-DC throughput from LTE eNB. Unit is kbps. Pdcp based computation
    // This value is not requested anymore, so it has been removed from the delivery, but it will be still logged;
    double drbThrDlPdcpBasedUeid = m_drbThrDlPdcpBasedComputationUeid.find (imsi) != m_drbThrDlPdcpBasedComputationUeid.end () ? m_drbThrDlPdcpBasedComputationUeid.at (imsi) : 0;    

    // UE-specific Downlink IP combined EN-DC throughput from LTE eNB. Unit is kbps. Rlc based computation
    double drbThrDlUeid = m_drbThrDlUeid.find (imsi) != m_drbThrDlUeid.end () ? m_drbThrDlUeid.at (imsi) : 0;

    indicationMessageHelper->AddDuUePmItem (
        ueImsiComplete, macPduUe, macPduInitialUe, macQpsk, mac16Qam, mac64Qam, macRetx, macVolume,
        macPrb, macMac04, macMac59, macMac1014, macMac1519, macMac2024, macMac2529, macSinrBin1,
        macSinrBin2, macSinrBin3, macSinrBin4, macSinrBin5, macSinrBin6, macSinrBin7,
        rlcBufferOccup, drbThrDlUeid);

    uePmStringDu.insert(std::make_pair(imsi, std::to_string(macPduUe) + ","
                                            + std::to_string(macPduInitialUe)+","
                                            + std::to_string(macQpsk)+","
                                            + std::to_string(mac16Qam)+","
                                            + std::to_string(mac64Qam)+","
                                            + std::to_string(macRetx)+","
                                            + std::to_string(macVolume)+","
                                            + std::to_string(macPrb)+","
                                            + std::to_string(macMac04)+","
                                            + std::to_string(macMac59)+","
                                            + std::to_string(macMac1014)+","
                                            + std::to_string(macMac1519)+","
                                            + std::to_string(macMac2024)+","
                                            + std::to_string(macMac2529)+","
                                            + std::to_string(macSinrBin1)+","
                                            + std::to_string(macSinrBin2)+","
                                            + std::to_string(macSinrBin3)+","
                                            + std::to_string(macSinrBin4)+","
                                            + std::to_string(macSinrBin5)+","
                                            + std::to_string(macSinrBin6)+","
                                            + std::to_string(macSinrBin7)+","
                                            + std::to_string(rlcBufferOccup)+','
                                            + std::to_string(drbThrDlUeid)+','
                                            + std::to_string(drbThrDlPdcpBasedUeid)
                                            ));
    
    // reset UE
    m_e2DuCalculator->ResetPhyTracesForRntiCellId(rnti, m_cellId);
  }
  m_drbThrDlPdcpBasedComputationUeid.clear ();
  m_drbThrDlUeid.clear ();

  // Denominator = (Total number of rows (TTIs) within a given time window* 14)
  // Numerator = (Sum of number of symbols across all rows (TTIs) group by cell ID within a given time window) * 139
  // Average Number of PRBs allocated for the UE = (NR/DR) (where 139 is the total number of PRBs available per NR cell, given numerology 2 with 60 kHz SCS)
  double prbUtilizationDl = macPrbsCellSpecific;

  NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << m_cellId << " cell, connected UEs number " << ueMap.size() 
      << " macPduCellSpecific " << macPduCellSpecific
      << " macPduInitialCellSpecific " << macPduInitialCellSpecific
      << " macVolumeCellSpecific " << macVolumeCellSpecific 
      << " macQpskCellSpecific " << macQpskCellSpecific
      << " mac16QamCellSpecific " << mac16QamCellSpecific
      << " mac64QamCellSpecific " << mac64QamCellSpecific
      << " macRetxCellSpecific " << macRetxCellSpecific
      << " macPrbsCellSpecific " << macPrbsCellSpecific //<< " " << macNumberOfSymbolsCellSpecific << " " << denominatorPrb
      << " macMac04CellSpecific " << macMac04CellSpecific
      << " macMac59CellSpecific " << macMac59CellSpecific
      << " macMac1014CellSpecific " << macMac1014CellSpecific
      << " macMac1519CellSpecific " << macMac1519CellSpecific
      << " macMac2024CellSpecific " << macMac2024CellSpecific
      << " macMac2529CellSpecific " << macMac2529CellSpecific
      << " macSinrBin1CellSpecific " << macSinrBin1CellSpecific
      << " macSinrBin2CellSpecific " << macSinrBin2CellSpecific
      << " macSinrBin3CellSpecific " << macSinrBin3CellSpecific
      << " macSinrBin4CellSpecific " << macSinrBin4CellSpecific
      << " macSinrBin5CellSpecific " << macSinrBin5CellSpecific
      << " macSinrBin6CellSpecific " << macSinrBin6CellSpecific
      << " macSinrBin7CellSpecific " << macSinrBin7CellSpecific
    );

  long dlAvailablePrbs = 139; // TODO this is for the current configuration, make it configurable
  long ulAvailablePrbs = 139; // TODO this is for the current configuration, make it configurable
  long qci = 1;
  long dlPrbUsage =  std::min ((long) (prbUtilizationDl / dlAvailablePrbs * 100), (long) 100); // percentage of used PRBs    
  long ulPrbUsage = 0; // TODO for future implementation

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->AddDuCellPmItem (
          macPduCellSpecific, macPduInitialCellSpecific, macQpskCellSpecific, mac16QamCellSpecific,
          mac64QamCellSpecific, prbUtilizationDl, macRetxCellSpecific, macVolumeCellSpecific,
          macMac04CellSpecific, macMac59CellSpecific, macMac1014CellSpecific,
          macMac1519CellSpecific, macMac2024CellSpecific, macMac2529CellSpecific,
          macSinrBin1CellSpecific, macSinrBin2CellSpecific, macSinrBin3CellSpecific,
          macSinrBin4CellSpecific, macSinrBin5CellSpecific, macSinrBin6CellSpecific,
          macSinrBin7CellSpecific, rlcBufferOccupCellSpecific, ueMap.size ());

      Ptr<CellResourceReport> cellResRep = Create<CellResourceReport> ();
      cellResRep->m_plmId = plmId;
      cellResRep->m_nrCellId = nrCellId;
      cellResRep->dlAvailablePrbs = dlAvailablePrbs;
      cellResRep->ulAvailablePrbs = ulAvailablePrbs;

      Ptr<ServedPlmnPerCell> servedPlmnPerCell = Create<ServedPlmnPerCell> ();
      servedPlmnPerCell->m_plmId = plmId;
      servedPlmnPerCell->m_nrCellId = nrCellId;

      Ptr<EpcDuPmContainer> epcDuVal = Create<EpcDuPmContainer> ();
      epcDuVal->m_qci = qci;
      epcDuVal->m_dlPrbUsage = dlPrbUsage;
      epcDuVal->m_ulPrbUsage = ulPrbUsage;

      servedPlmnPerCell->m_perQciReportItems.insert (epcDuVal);
      cellResRep->m_servedPlmnPerCellItems.insert (servedPlmnPerCell);

      indicationMessageHelper->AddDuCellResRepPmItem (cellResRep);
      indicationMessageHelper->FillDuValues (plmId + std::to_string (nrCellId));
    }

  if (m_forceE2FileLogging) {
    std::ofstream csv {};
    csv.open (m_duFileName.c_str (),  std::ios_base::app);
    if (!csv.is_open ())
    {
      NS_FATAL_ERROR ("Can't open file " << m_duFileName.c_str ());
    }

    uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now().GetMilliSeconds ();

    
    // the string is timestamp, ueImsiComplete, plmId, nrCellId, dlAvailablePrbs, ulAvailablePrbs, qci , dlPrbUsage, ulPrbUsage, /*CellSpecificValues*/, /* UESpecificValues */

    /*
      CellSpecificValues: 
        TB.TotNbrDl.1, TB.TotNbrDlInitial, TB.TotNbrDlInitial.Qpsk, TB.TotNbrDlInitial.16Qam, TB.TotNbrDlInitial.64Qam, RRU.PrbUsedDl,
        TB.ErrTotalNbrDl.1, QosFlow.PdcpPduVolumeDL_Filter, CARR.PDSCHMCSDist.Bin1, CARR.PDSCHMCSDist.Bin2, CARR.PDSCHMCSDist.Bin3,
        CARR.PDSCHMCSDist.Bin4, CARR.PDSCHMCSDist.Bin5, CARR.PDSCHMCSDist.Bin6, L1M.RS-SINR.Bin34, L1M.RS-SINR.Bin46, L1M.RS-SINR.Bin58,
        L1M.RS-SINR.Bin70, L1M.RS-SINR.Bin82, L1M.RS-SINR.Bin94, L1M.RS-SINR.Bin127, DRB.BufferSize.Qos, DRB.MeanActiveUeDl
    */

    std::string to_print_cell = plmId + "," 
        + std::to_string (nrCellId) + "," 
        + std::to_string (dlAvailablePrbs) + "," 
        + std::to_string (ulAvailablePrbs) + "," 
        + std::to_string (qci) + "," 
        + std::to_string (dlPrbUsage) + "," 
        + std::to_string (ulPrbUsage) + "," 
        + std::to_string (macPduCellSpecific) + "," 
        + std::to_string (macPduInitialCellSpecific) + "," 
        + std::to_string (macQpskCellSpecific) + "," 
        + std::to_string (mac16QamCellSpecific) + "," 
        + std::to_string (mac64QamCellSpecific) + ","
        + std::to_string ((long) std::ceil (prbUtilizationDl)) + "," 
        + std::to_string (macRetxCellSpecific) + "," 
        + std::to_string (macVolumeCellSpecific) + "," 
        + std::to_string (macMac04CellSpecific) + "," 
        + std::to_string (macMac59CellSpecific) + "," 
        + std::to_string (macMac1014CellSpecific) + "," 
        + std::to_string (macMac1519CellSpecific) + "," 
        + std::to_string (macMac2024CellSpecific) + "," 
        + std::to_string (macMac2529CellSpecific) + "," 
        + std::to_string (macSinrBin1CellSpecific) + "," 
        + std::to_string (macSinrBin2CellSpecific) + "," 
        + std::to_string (macSinrBin3CellSpecific) + "," 
        + std::to_string (macSinrBin4CellSpecific) + "," 
        + std::to_string (macSinrBin5CellSpecific) + "," 
        + std::to_string (macSinrBin6CellSpecific) + "," 
        + std::to_string (macSinrBin7CellSpecific) + "," 
        + std::to_string (rlcBufferOccupCellSpecific) + "," 
        + std::to_string (ueMap.size ());

    /* 
      UESpecificValues:

          TB.TotNbrDl.1.UEID, TB.TotNbrDlInitial.UEID, TB.TotNbrDlInitial.Qpsk.UEID, TB.TotNbrDlInitial.16Qam.UEID,TB.TotNbrDlInitial.64Qam.UEID, TB.ErrTotalNbrDl.1.UEID, QosFlow.PdcpPduVolumeDL_Filter.UEID,
          RRU.PrbUsedDl.UEID, CARR.PDSCHMCSDist.Bin1.UEID, CARR.PDSCHMCSDist.Bin2.UEID, CARR.PDSCHMCSDist.Bin3.UEID, CARR.PDSCHMCSDist.Bin5.UEID, CARR.PDSCHMCSDist.Bin6.UEID, 
          L1M.RS-SINR.Bin34.UEID, L1M.RS-SINR.Bin46.UEID, L1M.RS-SINR.Bin58.UEID, L1M.RS-SINR.Bin70.UEID, L1M.RS-SINR.Bin82.UEID, L1M.RS-SINR.Bin94.UEID, L1M.RS-SINR.Bin127.UEID, 
          DRB.BufferSize.Qos.UEID, DRB.UEThpDl.UEID, DRB.UEThpDlPdcpBased.UEID
    */

    for (auto ue : ueMap)
    {
      uint64_t imsi = ue.second->GetImsi();
      std::string ueImsiComplete = GetImsiString(imsi);

      auto uePms = uePmStringDu.find(imsi)->second;

      std::string to_print = std::to_string (timestamp) + ","
      + ueImsiComplete + "," 
      + to_print_cell +  ","
      + uePms + "\n";

      csv << to_print;
    }
    csv.close ();
    
    return nullptr;
    }
  else
    {
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}

void 
MmWaveEnbNetDevice::AddMillicarDevice(uint16_t millicarRnti, Ptr<MmWaveUeNetDevice> ueNetDevice){
  NS_LOG_FUNCTION(this);
  m_millicarDevicesMap[millicarRnti] = ueNetDevice;
}

void
MmWaveEnbNetDevice::BuildAndSendMillicarReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
  NS_LOG_FUNCTION(this);
  std::string plmId = "111";
  std::string gnbId = std::to_string(m_cellId);

  // TODO here we can get something from RRC and onward
  NS_LOG_DEBUG("This celll id " << std::to_string(m_cellId) << " BuildAndSendMessage at time " << Simulator::Now().GetSeconds());
  NS_LOG_DEBUG("Size of buffer " << m_l3sinrMillicarMap.size());


  if(m_sendCuCp)
  {
    // Create CU-CP
    Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    
    // create message for all the nodes in the list
    NS_LOG_DEBUG("Number of device " << m_millicarDevicesMap.size());
    for (std::map<uint16_t, Ptr<MmWaveUeNetDevice>>::iterator ueNetDevIt = m_millicarDevicesMap.begin();
    ueNetDevIt != m_millicarDevicesMap.end(); ++ ueNetDevIt){
      // for each of the millicar devices we sent a report; in case the device is not active we sent only the position

    // for (std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator ueReportIt = m_l3sinrMillicarMap.begin();
    // ueReportIt != m_l3sinrMillicarMap.end(); ++ ueReportIt){
      // Ptr<KpmIndicationMessage> cuCpMsg = BuildRicIndicationMessageCuCp(plmId);
      Ptr<KpmIndicationMessage> cuCpMsg = BuildMillicarReportRicIndicationMessageCucp(plmId, ueNetDevIt->first, m_millicarDevicesMap.size());
      // Send CU-CP only if offline logging is disabled
      if (!m_forceE2FileLogging && header != nullptr && cuCpMsg != nullptr)
      {
        NS_LOG_DEBUG ("Send LTE CU-CP");
        E2AP_PDU *pdu_cucp_ue = new E2AP_PDU; 
        encoding::generate_e2apv1_indication_request_parameterized(pdu_cucp_ue, 
                                                                  params.requestorId,
                                                                  params.instanceId,
                                                                  params.ranFuncionId,
                                                                  params.actionId,
                                                                  1, // TODO sequence number  
                                                                  (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                                  header->m_size, // size of the encoded header
                                                                  (uint8_t*) cuCpMsg->m_buffer, // buffer containing the encoded message
                                                                  cuCpMsg->m_size); // size of the encoded message  
        m_e2term->SendE2Message (pdu_cucp_ue);
        delete pdu_cucp_ue;
      }
    }
    // clear if necessary
    m_l3sinrMillicarMap.clear();
    m_pairsSinrMillicarMap.clear();
  }
  
  if (!m_forceE2FileLogging)
    Simulator::ScheduleWithContext (1, Seconds (m_e2Periodicity),
                                    &MmWaveEnbNetDevice::BuildAndSendMillicarReportMessage, this, params);
  else
    Simulator::Schedule (Seconds (m_e2Periodicity), &MmWaveEnbNetDevice::BuildAndSendMillicarReportMessage,
                         this, params);
}

void
MmWaveEnbNetDevice::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
  std::string plmId = "111";
  std::string gnbId = std::to_string(m_cellId);

  // TODO here we can get something from RRC and onward
  NS_LOG_DEBUG("MmWaveEnbNetDevice " << m_cellId << " BuildAndSendMessage at time " << Simulator::Now().GetSeconds());

  // we don't need it since it takes the data of rlc and pdcp (bytes)
  m_sendCuUp=false;
  
  if(m_sendCuUp)
  {
    // Create CU-UP
    Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    Ptr<KpmIndicationMessage> cuUpMsg = BuildRicIndicationMessageCuUp(plmId);

    // Send CU-UP only if offline logging is disabled
    if (!m_forceE2FileLogging && header != nullptr && cuUpMsg != nullptr)
    {
      NS_LOG_DEBUG ("Send NR CU-UP");
      E2AP_PDU *pdu_cuup_ue = new E2AP_PDU; 
      encoding::generate_e2apv1_indication_request_parameterized(pdu_cuup_ue, 
                                                               params.requestorId,
                                                               params.instanceId,
                                                               params.ranFuncionId,
                                                               params.actionId,
                                                               1, // TODO sequence number  
                                                               (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                               header->m_size, // size of the encoded header
                                                               (uint8_t*) cuUpMsg->m_buffer, // buffer containing the encoded message
                                                               cuUpMsg->m_size); // size of the encoded message  

      // DecodeMessage(pdu_cuup_ue);
      m_e2term->SendE2Message (pdu_cuup_ue);
      delete pdu_cuup_ue;
    }
  }

  // we need this since it sends the sinr of the serving and neighbour nodes
  m_sendCuCp = true;

  if(m_sendCuCp)
  {
    // Create and send CU-CP
    Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    Ptr<KpmIndicationMessage> cuCpMsg = BuildRicIndicationMessageCuCp(plmId);

    // Send CU-CP only if offline logging is disabled
    if (!m_forceE2FileLogging && header != nullptr && cuCpMsg != nullptr)
    {

      NS_LOG_DEBUG ("Send NR CU-CP");
      E2AP_PDU *pdu_cucp_ue = new E2AP_PDU; 
      encoding::generate_e2apv1_indication_request_parameterized(pdu_cucp_ue, 
                                                               params.requestorId,
                                                               params.instanceId,
                                                               params.ranFuncionId,
                                                               params.actionId,
                                                               1, // TODO sequence number  
                                                               (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                               header->m_size, // size of the encoded header
                                                               (uint8_t*) cuCpMsg->m_buffer, // buffer containing the encoded message
                                                               cuCpMsg->m_size); // size of the encoded message  
      // DecodeMessage(pdu_cucp_ue);
      m_e2term->SendE2Message (pdu_cucp_ue);
      delete pdu_cucp_ue;
    }
  }

  // we need this, but we have to insert the info over the assigned resources
  m_sendDu = false;
  if(m_sendDu)
  {
    // Create DU
    Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    Ptr<KpmIndicationMessage> duMsg = BuildRicIndicationMessageDu(plmId, m_cellId);

    // Send DU only if offline logging is disabled
    if (!m_forceE2FileLogging && header != nullptr && duMsg != nullptr)
    {

      NS_LOG_DEBUG ("Send NR DU");
      E2AP_PDU *pdu_du_ue = new E2AP_PDU; 
      encoding::generate_e2apv1_indication_request_parameterized(pdu_du_ue, 
                                                               params.requestorId,
                                                               params.instanceId,
                                                               params.ranFuncionId,
                                                               params.actionId,
                                                               1, // TODO sequence number  
                                                               (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                               header->m_size, // size of the encoded header
                                                               (uint8_t*) duMsg->m_buffer, // buffer containing the encoded message
                                                               duMsg->m_size); // size of the encoded message  

      NS_LOG_DEBUG("Gnb id " << gnbId);

      // DecodeMessage(pdu_du_ue);
      m_e2term->SendE2Message (pdu_du_ue);
      delete pdu_du_ue;
    }
  }
  
  if (!m_forceE2FileLogging)
    Simulator::ScheduleWithContext (1, Seconds (m_e2Periodicity),
                                    &MmWaveEnbNetDevice::BuildAndSendReportMessage, this, params);
  else
    Simulator::Schedule (Seconds (m_e2Periodicity), &MmWaveEnbNetDevice::BuildAndSendReportMessage,
                         this, params);
}

void
MmWaveEnbNetDevice::ForwardE2Message(uint8_t* buffer, size_t buffSize){
  NS_LOG_FUNCTION(this << buffSize << m_e2term);
  // in the buffer send by the ue, has only the message payload of a ric control message
  // the header we insert in in the enb and eventually send it to the xapp
  std::string plmId = "111";
  std::string gnbId = std::to_string(m_cellId);

  uint8_t idx;

  if((m_e2term == nullptr) | (!m_isReportingEnabled)){
    return;
  }

  // NS_LOG_UNCOND("Forwardind to Xapp");
  
	E2AP_PDU_t *pdu = nullptr;
	auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2AP_PDU, (void **) &pdu, (void *)buffer, buffSize);
	// print decoded payload
	if (retval.code == RC_OK) {
    
    // decoded correctly
    RICindication_t *ricIndication = &pdu->choice.initiatingMessage->value.choice.RICindication;

    for (idx = 0; idx < ricIndication->protocolIEs.list.count; idx++)
   {
      if(ricIndication->protocolIEs.list.array[idx]->id == 26)
      {
        int payload_size = ricIndication->protocolIEs.list.array[idx]-> \
			                              value.choice.RICindicationMessage.size;
        char* payload = (char*) calloc(payload_size, sizeof(char));
        memcpy(payload, ricIndication->protocolIEs.list.array[idx]-> \
                                        value.choice.RICindicationMessage.buf, payload_size);


        E2Termination::RicSubscriptionRequest_rval_s params = {24, 0, 200, 1};

        Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);

        pdu = new E2AP_PDU; ;

        encoding::generate_e2apv1_indication_request_parameterized(pdu, 
                                                                    params.requestorId,
                                                                    params.instanceId,
                                                                    params.ranFuncionId,
                                                                    params.actionId,
                                                                    1, // TODO sequence number  
                                                                    (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                                    header->m_size, // size of the encoded header
                                                                    (uint8_t*) payload, // buffer containing the encoded message
                                                                    payload_size); // size of the encoded message  

        // m_rrc->SendE2MessageBuffer(1, buffer, buffSize);
        // DecodeMessage(pdu);
        m_e2term->SendE2Message (pdu);
        // delete pdu;
      }
   }

    
  }
	
}

void
MmWaveEnbNetDevice::DecodeMessage (E2AP_PDU *pdu_du_ue)

{
  // TODO here we can get something from RRC and onward
  NS_LOG_FUNCTION(this << "MmWaveEnbNetDevice decoding");

  uint8_t idx;
  uint8_t ied;
  uint8_t ret = RC_OK;
  uint32_t recvBufLen;
  RICindication_t *ricIndication;
  RICaction_ToBeSetup_ItemIEs_t *actionItem;

  char *printBuffer;
  size_t size;
  FILE *stream = open_memstream(&printBuffer, &size);
  // asn_fprint(stream, &asn_DEF_E2AP_PDU, pdu_du_ue);
  xer_fprint(stream, &asn_DEF_E2AP_PDU, pdu_du_ue);

  tinyxml2::XMLDocument fullDoc;
  fullDoc.Parse(printBuffer);
  fullDoc.Print();
		

  printf("\nE2AP : RIC Indication received");
  ricIndication = &pdu_du_ue->choice.initiatingMessage->value.choice.RICindication;

  printf("protocolIEs elements %d\n", ricIndication->protocolIEs.list.count);			

  for (idx = 0; idx < ricIndication->protocolIEs.list.count; idx++)
   {
      switch(ricIndication->protocolIEs.list.array[idx]->id)
      {
        case 28:  // RIC indication type
        {
          long ricindicationType = ricIndication->protocolIEs.list.array[idx]-> \
                                      value.choice.RICindicationType;

          printf("ricindicationType %ld\n", ricindicationType);

          break;
        }
        case 26:  // RIC indication message
        {
          // break;
          int payload_size = ricIndication->protocolIEs.list.array[idx]-> \
                                      value.choice.RICindicationMessage.size;

          char* payload = (char*) calloc(payload_size, sizeof(char));
          memcpy(payload, ricIndication->protocolIEs.list.array[idx]-> \
                                      value.choice.RICindicationMessage.buf, payload_size);

          printf("Message size %d\n", payload_size);

          E2SM_KPM_IndicationMessage_t *descriptor = 0;
          
          auto retvalMsgKpm = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2SM_KPM_IndicationMessage, (void **) &descriptor, payload, payload_size);

          std::cout << " Printing with xml styple the message indication" << std::endl;
          
          char *printBufferMessage;
          size_t sizeMessage;
          FILE *streamMessage = open_memstream(&printBufferMessage, &sizeMessage);
          xer_fprint(streamMessage, &asn_DEF_E2SM_KPM_IndicationMessage, descriptor);
          
          E2SM_KPM_IndicationMessage_Format1_t *format = descriptor->choice.indicationMessage_Format1;

          // xer_fprint(stream, &asn_DEF_E2SM_KPM_IndicationMessage_Format1, format);

          tinyxml2::XMLDocument msgDoc;
          msgDoc.Parse(printBufferMessage);
          msgDoc.Print();

          break;
          // pm containers
          for(uint8_t pmContInd = 0; pmContInd<format->pm_Containers.list.count; pmContInd++){

            PM_Containers_Item_t *containers_list = format->pm_Containers.list.array[pmContInd];

            PF_Container_t *ranContainer = containers_list->performanceContainer;

            switch (ranContainer->present)
            {
            case PF_Container_PR_oDU:
              {
                NS_LOG_DEBUG ("O-DU: Get Cell Resource Report Item");
                ODU_PF_Container_t *odu = ranContainer->choice.oDU;
                for (uint8_t id_cellReports = 0; id_cellReports < odu->cellResourceReportList.list.count; id_cellReports++)
                {
                  CellResourceReportListItem_t *cellResourceReportList = odu->cellResourceReportList.list.array[id_cellReports];
                  for (uint8_t id_servedcellReports = 0; id_servedcellReports < cellResourceReportList->servedPlmnPerCellList.list.count; id_servedcellReports++)
                  {
                    ServedPlmnPerCellListItem_t* servedCellItem = cellResourceReportList->servedPlmnPerCellList.list.array[id_servedcellReports];
                    PLMN_Identity_t	 pLMN_Identity = servedCellItem->pLMN_Identity;
                    EPC_DU_PM_Container_t *edpc = servedCellItem->du_PM_EPC;
                    for (uint8_t id_cqiReports = 0; id_cqiReports < edpc->perQCIReportList_du.list.count; id_cqiReports++)
                    {
                      PerQCIReportListItem_t* cqiReportItem = edpc->perQCIReportList_du.list.array[id_cqiReports];
                      long *dlUsedPrbs = cqiReportItem->dl_PRBUsage;
                      long *ulUsedPrbs = cqiReportItem->ul_PRBUsage;
                      long qci = cqiReportItem->qci;
                    }
                  }
                  long *dlAvailablePrbs = cellResourceReportList->dl_TotalofAvailablePRBs;
                  long *ulAvailablePrbs = cellResourceReportList->ul_TotalofAvailablePRBs;
                  PLMN_Identity_t pLMN_Identity = cellResourceReportList->nRCGI.pLMN_Identity;
                  NRCellIdentity_t nRCellIdentity = cellResourceReportList->nRCGI.nRCellIdentity;

                }
              }
              break;

            case PF_Container_PR_oCU_CP:
              {
                NS_LOG_DEBUG ("O-CU: Get Cell Resource Report Item");
                OCUCP_PF_Container_t *ocucp = ranContainer->choice.oCU_CP;
                long *numActiveUes = ocucp->cu_CP_Resource_Status.numberOfActive_UEs;
              }
              break;

            case PF_Container_PR_oCU_UP:
              {
                NS_LOG_DEBUG ("O-CU-UP: Get Cell Resource Report Item");
                OCUUP_PF_Container_t* ocuup = ranContainer->choice.oCU_UP;
                for (uint8_t id_pfContainers = 0; id_pfContainers < ocuup->pf_ContainerList.list.count; id_pfContainers++)
                {
                  PF_ContainerListItem_t * pf_Container = ocuup->pf_ContainerList.list.array[id_pfContainers];
                  long interfaceType = pf_Container->interface_type;
                  for (uint8_t plmnListInd = 0; plmnListInd< pf_Container->o_CU_UP_PM_Container.plmnList.list.count; plmnListInd++){
                    PlmnID_Item_t* plmnIdItem = pf_Container->o_CU_UP_PM_Container.plmnList.list.array[plmnListInd];
                    PLMN_Identity_t	 pLMN_Identity = plmnIdItem->pLMN_Identity;
                    EPC_CUUP_PM_Format_t* cuuppmf = plmnIdItem->cu_UP_PM_EPC;
                    for (uint8_t cqiReportInd = 0; cqiReportInd< cuuppmf->perQCIReportList_cuup.list.count; cqiReportInd++){
                      PerQCIReportListItemFormat_t* pqrli = cuuppmf->perQCIReportList_cuup.list.array[cqiReportInd];
                      INTEGER_t *pDCPBytesDL = pqrli->pDCPBytesDL;
                      INTEGER_t *pDCPBytesUL = pqrli->pDCPBytesUL;
                      long drbqci = pqrli->drbqci;
                    }
                  }
                }
              }
              break;
            
            default:
              break;
            }

          }
          // list of pm information
          for(uint8_t pmInfoInd = 0; pmInfoInd<format->list_of_PM_Information->list.count; pmInfoInd++){
            PM_Info_Item_t* pmInfoItem = format->list_of_PM_Information->list.array[pmInfoInd];
            
            MeasurementTypeName_t measName = pmInfoItem->pmType.choice.measName;

            switch (pmInfoItem->pmVal.present)
            {
            case MeasurementValue_PR_valueInt:
              {
                long value = pmInfoItem->pmVal.choice.valueInt;
              }
              break;
            case MeasurementValue_PR_valueReal:
              {
                double value = pmInfoItem->pmVal.choice.valueReal;
              }
              break;
            case MeasurementValue_PR_valueRRC:
              {
                L3_RRC_Measurements_t *valueRRC = pmInfoItem->pmVal.choice.valueRRC;
              }
              break;
            
            default:
              break;
            }

            MeasurementValue_t pmVal = pmInfoItem->pmVal;
            // pmInfoItem->pmVal
          }

          // List of ue matched
          for(uint8_t ueInd = 0; ueInd<format->list_of_matched_UEs->list.count; ueInd++){
            PerUE_PM_Item_t* perUePmItem = format->list_of_matched_UEs->list.array[ueInd];
            // OctetString_t ueIdOctetString = perUePmItem->ueId;
            for(uint8_t pmListId =0; pmListId< perUePmItem->list_of_PM_Information->list.count; pmListId++){
              PM_Info_Item_t* pmItem = perUePmItem->list_of_PM_Information->list.array[pmListId];
              // pmItem->pmType.choice.measName

              switch (pmItem->pmVal.present)
              {
                case MeasurementValue_PR_valueInt:
                {
                  long value = pmItem->pmVal.choice.valueInt;
                }
                break;
                case MeasurementValue_PR_valueReal:
                  {
                    double value = pmItem->pmVal.choice.valueReal;
                  }
                break;
                case MeasurementValue_PR_valueRRC:
                  {
                    L3_RRC_Measurements_t *valueRRC = pmItem->pmVal.choice.valueRRC;
                  }
                break;
              
              default:
                break;
              }
              
            }
          }


          std::cout << "E2SM type " <<descriptor->present << std::endl;

          break;
        }

        case 25:  // RIC indication header
        {
          // break;
          std::cout << "Ric indication header at index " << (int)idx << std::endl;
          int payload_size = ricIndication->protocolIEs.list.array[idx]-> \
                                      value.choice.RICindicationHeader.size;


          char* payload = (char*) calloc(payload_size, sizeof(char));
          memcpy(payload, ricIndication->protocolIEs.list.array[idx]-> \
                                      value.choice.RICindicationHeader.buf, payload_size);

          E2SM_KPM_IndicationHeader_t *descriptor = 0;

          auto retvalMsgKpm = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2SM_KPM_IndicationHeader, (void **) &descriptor, payload, payload_size);

          char *printBufferHeader;
          size_t sizeHeader;
          FILE *streamHeader = open_memstream(&printBufferHeader, &sizeHeader);

          xer_fprint(streamHeader, &asn_DEF_E2SM_KPM_IndicationHeader, descriptor);

          std::cout << "Printer header doc " << (int)idx << std::endl;

          tinyxml2::XMLDocument headerDoc;
          headerDoc.Parse(printBufferHeader);
          headerDoc.Print();

          break;

          
          E2SM_KPM_IndicationHeader_Format1_t* ind_header = descriptor->choice.indicationHeader_Format1;

          std::cout << "E2SM type " <<descriptor->present << std::endl;
          std::cout << "global e2 node type " <<ind_header->id_GlobalE2node_ID.present << std::endl;

          std::string plmId = "";
          std::string gnbId = "";
          uint64_t _timestamp = 0;

          switch(ind_header->id_GlobalE2node_ID.present){
            case GlobalE2node_ID_PR_NOTHING: {

            }
            break;
            case GlobalE2node_ID_PR_gNB: {
              GlobalE2node_gNB_ID_t* _gnbChoice = ind_header->id_GlobalE2node_ID.choice.gNB;
              int gnbIdSize = _gnbChoice->global_gNB_ID.gnb_id.choice.gnb_ID.size;
              // std::cout << "Size of gnb id " << gnbIdSize << std::endl;
              if (gnbIdSize>0){
                char gnbIdOut[gnbIdSize + 1];
                std::memcpy (gnbIdOut, _gnbChoice->global_gNB_ID.gnb_id.choice.gnb_ID.buf, gnbIdSize);
                gnbIdOut[gnbIdSize] = '\0';
                std::cout << "Value of gnb id " << gnbIdOut << std::endl;
                gnbId = std::string(gnbIdOut);
              }

              int plmnIdSize = _gnbChoice->global_gNB_ID.plmn_id.size;
              char plmnIdOut[plmnIdSize + 1];
              std::memcpy (plmnIdOut, _gnbChoice->global_gNB_ID.plmn_id.buf, plmnIdSize);
              plmnIdOut[plmnIdSize] = '\0';
              plmId = std::string(plmnIdOut);
            }
            break;
            case GlobalE2node_ID_PR_en_gNB: {
              GlobalE2node_en_gNB_ID_t	* _gnbChoice = ind_header->id_GlobalE2node_ID.choice.en_gNB;
              int gnbIdSize = _gnbChoice->global_gNB_ID.gNB_ID.choice.gNB_ID.size;
              // std::cout << "Size of gnb id " << gnbIdSize << std::endl;
              if (gnbIdSize>0){
                char gnbIdOut[gnbIdSize + 1];
                std::memcpy (gnbIdOut, _gnbChoice->global_gNB_ID.gNB_ID.choice.gNB_ID.buf, gnbIdSize);
                gnbIdOut[gnbIdSize] = '\0';
                // std::cout << "Value of gnb id " << std::string(gnbIdOut) << std::endl;
                gnbId = std::string(gnbIdOut);
              }

              int plmnIdSize = _gnbChoice->global_gNB_ID.pLMN_Identity.size;
              char plmnIdOut[plmnIdSize + 1];
              std::memcpy (plmnIdOut, _gnbChoice->global_gNB_ID.pLMN_Identity.buf, plmnIdSize);
              plmnIdOut[plmnIdSize] = '\0';
              plmId = std::string(plmnIdOut);
            }
            break;
            case GlobalE2node_ID_PR_ng_eNB: {
              GlobalE2node_ng_eNB_ID_t	* _gnbChoice = ind_header->id_GlobalE2node_ID.choice.ng_eNB;
              BIT_STRING_t _bit_string_obj;
              switch (_gnbChoice->global_ng_eNB_ID.enb_id.present)
              {
                case ENB_ID_Choice_PR_enb_ID_macro:
                  _bit_string_obj = _gnbChoice->global_ng_eNB_ID.enb_id.choice.enb_ID_macro;
                  break;
                case ENB_ID_Choice_PR_enb_ID_shortmacro:
                  _bit_string_obj = _gnbChoice->global_ng_eNB_ID.enb_id.choice.enb_ID_shortmacro;
                  break;
                case ENB_ID_Choice_PR_enb_ID_longmacro:
                  _bit_string_obj = _gnbChoice->global_ng_eNB_ID.enb_id.choice.enb_ID_longmacro;
                  break;
                
                default:
                  break;
              }
              int gnbIdSize = _bit_string_obj.size;
              // std::cout << "Size of gnb id " << gnbIdSize << std::endl;
              if (gnbIdSize>0){
                char gnbIdOut[gnbIdSize + 1];
                std::memcpy (gnbIdOut, _bit_string_obj.buf, gnbIdSize);
                gnbIdOut[gnbIdSize] = '\0';
                // std::cout << "Value of gnb id " << gnbIdOut << std::endl;
                gnbId = std::string(gnbIdOut);
              }

              int plmnIdSize = _gnbChoice->global_ng_eNB_ID.plmn_id.size;
              char plmnIdOut[plmnIdSize + 1];
              std::memcpy (plmnIdOut, _gnbChoice->global_ng_eNB_ID.plmn_id.buf, plmnIdSize);
              plmnIdOut[plmnIdSize] = '\0';
              plmId = std::string(plmnIdOut);
            }
            break;
            case GlobalE2node_ID_PR_eNB: {
              GlobalE2node_eNB_ID_t* _gnbChoice = ind_header->id_GlobalE2node_ID.choice.eNB;
              // std::cout << "Type of enb id " << (int) _gnbChoice->global_eNB_ID.eNB_ID.present << std::endl;
              BIT_STRING_t _bit_string_obj;
              switch (_gnbChoice->global_eNB_ID.eNB_ID.present)
              {
              case ENB_ID_PR_macro_eNB_ID:
                _bit_string_obj = _gnbChoice->global_eNB_ID.eNB_ID.choice.macro_eNB_ID;
                break;
              case ENB_ID_PR_home_eNB_ID:
                _bit_string_obj = _gnbChoice->global_eNB_ID.eNB_ID.choice.home_eNB_ID;
                break;
              case ENB_ID_PR_short_Macro_eNB_ID:
                _bit_string_obj = _gnbChoice->global_eNB_ID.eNB_ID.choice.short_Macro_eNB_ID;
                break;
              case ENB_ID_PR_long_Macro_eNB_ID:
                _bit_string_obj = _gnbChoice->global_eNB_ID.eNB_ID.choice.long_Macro_eNB_ID;
                break;
              
              default:
                break;
              }
              
              int gnbIdSize = (int) _bit_string_obj.size;
              // std::cout << "Size of gnb id " << gnbIdSize << std::endl;
              if (gnbIdSize>0){
                char gnbIdOut[gnbIdSize + 1];
                std::memcpy (gnbIdOut, _bit_string_obj.buf, gnbIdSize);
                gnbIdOut[gnbIdSize] = '\0';
                // std::cout << "Value of gnb id " << gnbIdOut << std::endl;
                gnbId = std::string(gnbIdOut);
              }
              // std::cout << "Consider plmn" << std::endl;
              PLMN_Identity_t plmn_identity = _gnbChoice->global_eNB_ID.pLMN_Identity;
              int plmnIdSize = plmn_identity.size;
              // std::cout << "Size of plmn id " << plmnIdSize << std::endl;
              if (plmnIdSize > 0){
                char plmnIdOut[plmnIdSize + 1];
                // std::cout << "Size of gnb buffer " << plmn_identity.buf << std::endl;
                // std::cout << "Size of gnb buffer " << plmn_identity.buf << std::endl;
                std::memcpy (plmnIdOut, plmn_identity.buf, plmnIdSize);
                plmnIdOut[plmnIdSize] = '\0';
                plmId = std::string(plmnIdOut);
                
              }
              
            }
            break;
          }

          std::cout << "plmn id " << plmId << " gnb id " << gnbId << std::endl;
          long _firstConversion = 0;

          std::cout << "Size of timestamp " << (int)ind_header->collectionStartTime.size << std::endl;
          std::memcpy(&_firstConversion, ind_header->collectionStartTime.buf, ind_header->collectionStartTime.size);

          _timestamp = be64toh(_firstConversion);

          std::cout << "Time stamp " << _timestamp << std::endl;

        }
        
      }
   }

	
}

void
MmWaveEnbNetDevice::SetStartTime (uint64_t st)
{
  m_startTime = st;
}

}
}
