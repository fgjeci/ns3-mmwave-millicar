/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 * Author: Marco Miozzo <mmiozzo@cttc.es> : Update to FF API Architecture
 * Author: Nicola Baldo <nbaldo@cttc.es>  : Integrated with new RRC and MAC architecture
 * Author: Danilo Abrignani <danilo.abrignani@unibo.it> : Integrated with new architecture - GSoC 2015 - Carrier Aggregation
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 */

#include <ns3/llc-snap-header.h>
#include <ns3/simulator.h>
#include <ns3/callback.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/lte-net-device.h>
#include <ns3/packet-burst.h>
#include <ns3/uinteger.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/pointer.h>
#include <ns3/enum.h>
#include <ns3/string.h>
#include <ns3/lte-amc.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/lte-enb-net-device-oran.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/mc-enb-pdcp.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/lte-anr.h>
#include <ns3/lte-ffr-algorithm.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/object-map.h>
#include <ns3/object-factory.h>
#include <ns3/lte-radio-bearer-info.h>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <ns3/lte-indication-message-helper.h>
#include <ns3/config.h>

// oran interface
#include <ns3/oran-interface.h>

#include "encode_e2apv1.hpp"
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


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteEnbNetDeviceOran");

NS_OBJECT_ENSURE_REGISTERED (LteEnbNetDeviceOran);

/**
* KPM Subscription Request callback.
* This function is triggered whenever a RIC Subscription Request for 
* the KPM RAN Function is received.
*
* \param pdu request message
*/
void 
LteEnbNetDeviceOran::KpmSubscriptionCallback (E2AP_PDU_t* sub_req_pdu)
{
  NS_LOG_DEBUG ("\nReceived RIC Subscription Request, cellId = " << m_cellId << "\n");

  E2Termination::RicSubscriptionRequest_rval_s params = m_e2term->ProcessRicSubscriptionRequest (sub_req_pdu);
  NS_LOG_DEBUG ("requestorId " << +params.requestorId << 
                 ", instanceId " << +params.instanceId << 
                 ", ranFuncionId " << +params.ranFuncionId << 
                 ", actionId " << +params.actionId);  
  
  if (!m_isReportingEnabled && !m_forceE2FileLogging)
  {
    BuildAndSendReportMessage (params);
    m_isReportingEnabled = true; 
  }
}

void 
LteEnbNetDeviceOran::MillicarServiceModelRegisterCallback (E2AP_PDU_t* sm_pdu)
{
  NS_LOG_FUNCTION(this);
  NS_LOG_DEBUG ("\nReceived service model response, cellId= " << m_cellId << "\n");

  // relay the message to the right ue
  uint8_t *buf;

  int len = e2ap_asn1c_encode_pdu(sm_pdu, &buf);
  NS_LOG_DEBUG ("Len of decoding  " << len << "\n");
  m_rrc->SendE2MessageBuffer( 1, buf, len);
    
}

void
LteEnbNetDeviceOran::ReadControlFile ()
{
  // open the control file and read handover commands
  if (m_controlFilename != "")
    {
      std::ifstream csv{};
      csv.open (m_controlFilename.c_str (), std::ifstream::in);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_controlFilename.c_str ());
        }
      std::string line;

      if (m_controlFilename.find ("ts_actions_for_ns3.csv") != std::string::npos)
        {

          long long timestamp{};

          while (std::getline (csv, line))
            {
              if (line == "")
                {
                  // skip empty lines
                  continue;
                }
              NS_LOG_INFO ("Read handover command");
              std::stringstream lineStream (line);
              std::string cell;

              std::getline (lineStream, cell, ',');
              timestamp = std::stoll (cell);

              uint64_t imsi;
              std::getline (lineStream, cell, ',');
              imsi = std::stoi (cell);

              uint16_t targetCellId;
              std::getline (lineStream, cell, ',');
              // uncomment the next line if need to remove PLM ID, first 3 digits always 111
              // cell.erase(0, 3);
              targetCellId = std::stoi (cell);

              NS_LOG_INFO ("Handover command for timestamp " << timestamp << " imsi " << imsi
                                                             << " targetCellId " << targetCellId);

              m_rrc->TakeUeHoControl (imsi);
              Simulator::ScheduleWithContext (1, Seconds (0),
                                              &LteEnbRrc::PerformHandoverToTargetCell, m_rrc, imsi,
                                              targetCellId);
            }
        }
      else
        {
          NS_FATAL_ERROR (
              "Unknown use case not implemented yet with filename: " << m_controlFilename);
        }

      csv.close ();

      std::ofstream csvDelete{};
      csvDelete.open (m_controlFilename.c_str ());

      NS_LOG_INFO ("File flushed");
    }

  // TODO check if we need to run multiple times in a m_e2Periodicity time delta,
  // to catch commands that are late
  // We can run every ms, if the file is empty, do not do anything
  Simulator::Schedule (Seconds (0.001), &LteEnbNetDeviceOran::ReadControlFile, this);
}



TypeId LteEnbNetDeviceOran::GetTypeId (void)
{
  static TypeId
    tid =
    TypeId ("ns3::LteEnbNetDeviceOran")
    .SetParent<LteNetDevice> ()
    .AddConstructor<LteEnbNetDeviceOran> ()
    .AddAttribute ("LteEnbRrc",
                   "The RRC associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_rrc),
                   MakePointerChecker <LteEnbRrc> ())
    .AddAttribute ("LteHandoverAlgorithm",
                   "The handover algorithm associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_handoverAlgorithm),
                   MakePointerChecker <LteHandoverAlgorithm> ())
    .AddAttribute ("LteAnr",
                   "The automatic neighbour relation function associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_anr),
                   MakePointerChecker <LteAnr> ())
    .AddAttribute ("LteFfrAlgorithm",
                   "The FFR algorithm associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_ffrAlgorithm),
                   MakePointerChecker <LteFfrAlgorithm> ())
    .AddAttribute ("LteEnbComponentCarrierManager",
                   "The RRC associated to this EnbNetDevice",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_componentCarrierManager),
                   MakePointerChecker <LteEnbComponentCarrierManager> ())
    .AddAttribute ("ComponentCarrierMap", "List of component carriers.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&LteEnbNetDeviceOran::m_ccMap),
                   MakeObjectMapChecker<ComponentCarrierEnb> ())
    .AddAttribute ("UlBandwidth",
                   "Uplink Transmission Bandwidth Configuration in number of Resource Blocks",
                   UintegerValue (100),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::SetUlBandwidth,
                                         &LteEnbNetDeviceOran::GetUlBandwidth),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("DlBandwidth",
                   "Downlink Transmission Bandwidth Configuration in number of Resource Blocks",
                   UintegerValue (100),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::SetDlBandwidth,
                                         &LteEnbNetDeviceOran::GetDlBandwidth),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("CellId",
                   "Cell Identifier",
                   UintegerValue (0),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::m_cellId),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("DlEarfcn",
                   "Downlink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                   "as per 3GPP 36.101 Section 5.7.3. ",
                   UintegerValue (100),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::m_dlEarfcn),
                   MakeUintegerChecker<uint32_t> (0, 262143))
    .AddAttribute ("UlEarfcn",
                   "Uplink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                   "as per 3GPP 36.101 Section 5.7.3. ",
                   UintegerValue (18100),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::m_ulEarfcn),
                   MakeUintegerChecker<uint32_t> (0, 262143))
    .AddAttribute ("CsgId",
                   "The Closed Subscriber Group (CSG) identity that this eNodeB belongs to",
                   UintegerValue (0),
                   MakeUintegerAccessor (&LteEnbNetDeviceOran::SetCsgId,
                                         &LteEnbNetDeviceOran::GetCsgId),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("CsgIndication",
                   "If true, only UEs which are members of the CSG (i.e. same CSG ID) "
                   "can gain access to the eNodeB, therefore enforcing closed access mode. "
                   "Otherwise, the eNodeB operates as a non-CSG cell and implements open access mode.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbNetDeviceOran::SetCsgIndication,
                                        &LteEnbNetDeviceOran::GetCsgIndication),
                   MakeBooleanChecker ())
    .AddAttribute ("E2Termination",
                   "The E2 termination object associated to this node",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::SetE2Termination,
                                        &LteEnbNetDeviceOran::GetE2Termination),
                   MakePointerChecker <E2Termination> ())
    .AddAttribute ("E2PdcpCalculator",
                   "The PDCP calculator object for E2 reporting",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_e2PdcpStatsCalculator),
                   MakePointerChecker <mmwave::MmWaveBearerStatsCalculator> ())    
    .AddAttribute ("E2RlcCalculator",
                   "The RLC calculator object for E2 reporting",
                   PointerValue (),
                   MakePointerAccessor (&LteEnbNetDeviceOran::m_e2RlcStatsCalculator),
                   MakePointerChecker <mmwave::MmWaveBearerStatsCalculator> ())
    .AddAttribute ("E2Periodicity",
                   "Periodicity of E2 reporting (value in seconds)",
                   DoubleValue (0.1),
                   MakeDoubleAccessor (&LteEnbNetDeviceOran::m_e2Periodicity),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EnableCuUpReport",
                   "If true, send CuUpReport",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbNetDeviceOran::m_sendCuUp),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableCuCpReport",
                   "If true, send CuCpReport",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbNetDeviceOran::m_sendCuCp),
                   MakeBooleanChecker ())
    .AddAttribute ("ReducedPmValues",
                   "If true, send only a subset of pmValues",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbNetDeviceOran::m_reducedPmValues),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableE2FileLogging",
                   "If true, force E2 indication generation and write E2 fields in csv file",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbNetDeviceOran::m_forceE2FileLogging),
                   MakeBooleanChecker ())
    .AddAttribute ("ControlFileName",
                   "Filename for the stand alone control mode. The file is deleted after every read."
                   "Format should correspond to the particular use case:\n"
                   "TS: Contains multiple lines with ts, imsi, targetCellId\n",
                   StringValue(""),
                   MakeStringAccessor (&LteEnbNetDeviceOran::m_controlFilename),
                   MakeStringChecker())
  ;
  return tid; 
}

LteEnbNetDeviceOran::LteEnbNetDeviceOran ()
  : m_isConstructed (false),
    m_isConfigured (false),
    m_anr (0),
    m_componentCarrierManager(0), 
    m_isReportingEnabled (false),
    m_reducedPmValues (false),
    m_forceE2FileLogging (false),
    m_cuUpFileName (),
    m_cuCpFileName ()
{
  NS_LOG_FUNCTION (this);
}

LteEnbNetDeviceOran::~LteEnbNetDeviceOran (void)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbNetDeviceOran::DoDispose ()
{
  NS_LOG_FUNCTION (this);

  m_rrc->Dispose ();
  m_rrc = 0;

  m_handoverAlgorithm->Dispose ();
  m_handoverAlgorithm = 0;

  if (m_anr != nullptr)
    {
      m_anr->Dispose ();
      m_anr = 0;
    }
  m_componentCarrierManager->Dispose();
  m_componentCarrierManager = 0;
  // ComponentCarrierEnb::DoDispose() will call DoDispose
  // of its PHY, MAC, FFR and scheduler instance
  for (uint32_t i = 0; i < m_ccMap.size (); i++)
    {
      m_ccMap.at (i)->Dispose ();
      m_ccMap.at (i) = 0;
    }

  LteNetDevice::DoDispose ();
}



Ptr<LteEnbMac>
LteEnbNetDeviceOran::GetMac () const
{
  return m_ccMap.at (0)->GetMac ();
}

Ptr<LteEnbPhy>
LteEnbNetDeviceOran::GetPhy () const
{
  return m_ccMap.at (0)->GetPhy ();
}

Ptr<LteEnbMac>
LteEnbNetDeviceOran::GetMac (uint8_t index)
{
  return m_ccMap.at (index)->GetMac ();
}

Ptr<LteEnbPhy>
LteEnbNetDeviceOran::GetPhy(uint8_t index)
{
  return m_ccMap.at (index)->GetPhy ();
}

Ptr<LteEnbRrc>
LteEnbNetDeviceOran::GetRrc () const
{
  return m_rrc;
}

Ptr<LteEnbComponentCarrierManager>
LteEnbNetDeviceOran::GetComponentCarrierManager () const
{
  return  m_componentCarrierManager;
}

uint16_t
LteEnbNetDeviceOran::GetCellId () const
{
  return m_cellId;
}

bool
LteEnbNetDeviceOran::HasCellId (uint16_t cellId) const
{
  for (auto &it: m_ccMap)
    {
      if (it.second->GetCellId () == cellId)
        {
          return true;
        }
    }
  return false;
}

uint8_t
LteEnbNetDeviceOran::GetUlBandwidth () const
{
  return m_ulBandwidth;
}

void
LteEnbNetDeviceOran::SetUlBandwidth (uint8_t bw)
{
  NS_LOG_FUNCTION (this << uint16_t (bw));
  switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
      m_ulBandwidth = bw;
      break;

    default:
      NS_FATAL_ERROR ("invalid bandwidth value " << (uint16_t) bw);
      break;
    }
}

uint8_t
LteEnbNetDeviceOran::GetDlBandwidth () const
{
  return m_dlBandwidth;
}

void
LteEnbNetDeviceOran::SetDlBandwidth (uint8_t bw)
{
  NS_LOG_FUNCTION (this << uint16_t (bw));
  switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
      m_dlBandwidth = bw;
      break;

    default:
      NS_FATAL_ERROR ("invalid bandwidth value " << (uint16_t) bw);
      break;
    }
}

uint32_t
LteEnbNetDeviceOran::GetDlEarfcn () const
{
  return m_dlEarfcn;
}

void
LteEnbNetDeviceOran::SetDlEarfcn (uint32_t earfcn)
{
  NS_LOG_FUNCTION (this << earfcn);
  m_dlEarfcn = earfcn;
}

uint32_t
LteEnbNetDeviceOran::GetUlEarfcn () const
{
  return m_ulEarfcn;
}

void
LteEnbNetDeviceOran::SetUlEarfcn (uint32_t earfcn)
{
  NS_LOG_FUNCTION (this << earfcn);
  m_ulEarfcn = earfcn;
}

uint32_t
LteEnbNetDeviceOran::GetCsgId () const
{
  return m_csgId;
}

void
LteEnbNetDeviceOran::SetCsgId (uint32_t csgId)
{
  NS_LOG_FUNCTION (this << csgId);
  m_csgId = csgId;
  UpdateConfig (); // propagate the change to RRC level
}

bool
LteEnbNetDeviceOran::GetCsgIndication () const
{
  return m_csgIndication;
}

void
LteEnbNetDeviceOran::SetCsgIndication (bool csgIndication)
{
  NS_LOG_FUNCTION (this << csgIndication);
  m_csgIndication = csgIndication;
  UpdateConfig (); // propagate the change to RRC level
}

std::map < uint8_t, Ptr<ComponentCarrierEnb> >
LteEnbNetDeviceOran::GetCcMap ()
{
  return m_ccMap;
}

void
LteEnbNetDeviceOran::SetCcMap (std::map< uint8_t, Ptr<ComponentCarrierEnb> > ccm)
{
  NS_ASSERT_MSG (!m_isConfigured, "attempt to set CC map after configuration");
  m_ccMap = ccm;
}

void
LteEnbNetDeviceOran::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_isConstructed = true;
  UpdateConfig ();
  std::map< uint8_t, Ptr<ComponentCarrierEnb> >::iterator it;
  for (it = m_ccMap.begin (); it != m_ccMap.end (); ++it)
    {
       it->second->Initialize ();
    }
  m_rrc->Initialize ();
  m_componentCarrierManager->Initialize();
  m_handoverAlgorithm->Initialize ();

  if (m_anr != nullptr)
    {
      m_anr->Initialize ();
    }

  m_ffrAlgorithm->Initialize ();

    // connect to mac sidelink trace 
    // connect to callback
    // bool _retValue = Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveSidelinkMac/SlSinrReport",
    // MakeBoundCallback (&LteEnbNetDeviceOran::RegisterSlSinrReportReadingCallback, this));

    // NS_LOG_DEBUG("Registered to sl mac sinr " << _retValue);
}

// end modification
void
LteEnbNetDeviceOran::RegisterSlSinrReportReadingCallback(Ptr<LteEnbNetDeviceOran> netDev, std::string context, uint16_t localRnti, uint16_t destRnti, uint8_t numSym, uint32_t tbSize, double avgSinr)
{
  netDev->RegisterSlSinrReportReading(localRnti, destRnti, numSym, tbSize, avgSinr);
}

void
LteEnbNetDeviceOran::RegisterSlSinrReportReading(uint16_t localRnti, uint16_t destRnti, uint8_t numSym, uint32_t tbSize, double avgSinr)
{
  // NS_LOG_FUNCTION(this);
  m_l3sinrMap[localRnti][destRnti].mcs = numSym;
  m_l3sinrMap[localRnti][destRnti].sinr = avgSinr;
  
  // NS_LOG_LOGIC (Simulator::Now ().GetSeconds ()
  //               << " uedev " << localRnti << " report for " << destRnti
  //               << " SINR " << m_l3sinrMap[localRnti][destRnti].sinr);
}

// modified


bool
LteEnbNetDeviceOran::Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION (this << packet << dest << protocolNumber);
  NS_ABORT_MSG_IF (protocolNumber != Ipv4L3Protocol::PROT_NUMBER
                   && protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                   "unsupported protocol " << protocolNumber << ", only IPv4 and IPv6 are supported");
  return m_rrc->SendData (packet);
}


void
LteEnbNetDeviceOran::UpdateConfig (void)
{
  NS_LOG_FUNCTION (this);

  if (m_isConstructed)
    {
      if (!m_isConfigured)
        {
          NS_LOG_LOGIC (this << " Configure cell " << m_cellId);
          // we have to make sure that this function is called only once
          NS_ASSERT (!m_ccMap.empty ());
          m_rrc->ConfigureCell (m_ccMap);
          m_isConfigured = true;
        }

      NS_LOG_LOGIC (this << " Updating SIB1 of cell " << m_cellId
                         << " with CSG ID " << m_csgId
                         << " and CSG indication " << m_csgIndication);
      m_rrc->SetCsgId (m_csgId, m_csgIndication);

      if(m_e2term)
      {
      	NS_LOG_DEBUG("E2sim start in cell " << m_cellId 
          << " force CSV logging " << m_forceE2FileLogging);

        if (!m_forceE2FileLogging)
          {
            Simulator::Schedule (MicroSeconds (0), &E2Termination::Start, m_e2term);
          }
        else { // give some time for the simulation to start, TODO check value
          m_cuUpFileName = "cu-up-cell-" + std::to_string(m_cellId) + ".txt";
          std::ofstream csv {};
          csv.open (m_cuUpFileName.c_str ());
          csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
                 "m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),"
                 "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
                 "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),"
                 "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
                 "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
          csv.close();

          m_cuCpFileName = "cu-cp-cell-" + std::to_string(m_cellId) + ".txt";
          csv.open (m_cuCpFileName.c_str ());
          csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
                 "DRB.RelActNbr.5QI.UEID (0),enbdev (m_cellId),UE (imsi),sameCellSinr,"
                 "sameCellSinr 3gpp encoded,L3 neigh Id (cellId),"
                 "sinr,3gpp encoded sinr (convertedSinr)\n";
          csv.close();
          Simulator::Schedule(MicroSeconds(500), &LteEnbNetDeviceOran::BuildAndSendReportMessage, this, E2Termination::RicSubscriptionRequest_rval_s{});

          Simulator::Schedule(MicroSeconds(1000), &LteEnbNetDeviceOran::ReadControlFile, this);
        }
      }
    }
  else
    {
      /*
       * Lower layers are not ready yet, so do nothing now and expect
       * ``DoInitialize`` to re-invoke this function.
       */
    }
}

Ptr<E2Termination>
LteEnbNetDeviceOran::GetE2Termination() const
{
  return m_e2term;
}

void
LteEnbNetDeviceOran::SetE2Termination(Ptr<E2Termination> e2term)
{
  m_e2term = e2term;

  NS_LOG_DEBUG("Register E2SM");

  if (!m_forceE2FileLogging)
    {
      Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription> ();
      e2term->RegisterKpmCallbackToE2Sm (
          200, kpmFd,
          std::bind (&LteEnbNetDeviceOran::KpmSubscriptionCallback, this, std::placeholders::_1));

      Ptr<RicControlFunctionDescription> ricCtrlFd = Create<RicControlFunctionDescription> ();
      e2term->RegisterSmCallbackToE2Sm (300, ricCtrlFd,
                                        std::bind (&LteEnbNetDeviceOran::MillicarServiceModelRegisterCallback,
                                                   this, std::placeholders::_1));
    }
}

std::string
LteEnbNetDeviceOran::GetImsiString(uint64_t imsi)
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
LteEnbNetDeviceOran::BuildRicIndicationHeader (std::string plmId, std::string gnbId, uint16_t nrCellId)
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
          Create<KpmIndicationHeader> (KpmIndicationHeader::GlobalE2nodeType::eNB, headerValues);

      return header;
    }
  else
    {
      return nullptr;
    }
}

Ptr<KpmIndicationMessage>
LteEnbNetDeviceOran::BuildRicIndicationMessageCuUp(std::string plmId)
{
  Ptr<LteIndicationMessageHelper> indicationMessageHelper =
      Create<LteIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuUp,
                                          m_forceE2FileLogging, m_reducedPmValues);

  // get <rnti, UeManager> map of connected UEs
  auto ueMap = m_rrc->GetUeMap();
  // gNB-wide PDCP volume in downlink
  double cellDlTxVolume = 0;

  // sum of the per-user average latency
  double perUserAverageLatencySum = 0;

  std::unordered_map<uint64_t, std::string> uePmString {};

  for (auto ue : ueMap)
  {
    uint64_t imsi = ue.second->GetImsi();
    std::string ueImsiComplete = GetImsiString(imsi);

    // TODO fix types
    long txDlPackets = m_e2PdcpStatsCalculator->GetDlTxPackets(imsi, 3); // LCID 3 is used for data
    double txBytes = m_e2PdcpStatsCalculator->GetDlTxData(imsi, 3)  * 8 / 1e3; // in kbit, not byte
    cellDlTxVolume += txBytes;

    long txPdcpPduLteRlc = 0;
    double txPdcpPduBytesLteRlc = 0;
    auto drbMap = ue.second->GetDrbMap();
    for (auto drb : drbMap)
    {
      txPdcpPduLteRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
      txPdcpPduBytesLteRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
      drb.second->m_rlc->ResetRlcCounters();
    }
    auto rlcMap = ue.second->GetRlcMap(); // secondary-connected RLCs
    for (auto drb : rlcMap)
    {
      txPdcpPduLteRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
      txPdcpPduBytesLteRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
      drb.second->m_rlc->ResetRlcCounters();
    }
    txPdcpPduBytesLteRlc *= 8 / 1e3;

    long txPdcpPduNrRlc = std::max(long(0), txDlPackets - txPdcpPduLteRlc);
    double txPdcpPduBytesNrRlc = std::max(0.0, txBytes - txPdcpPduBytesLteRlc);

    double pdcpLatency = m_e2PdcpStatsCalculator->GetDlDelay(imsi, 3) / 1e5; // unit: x 0.1 ms
    perUserAverageLatencySum += pdcpLatency;

    double pdcpThroughput = txBytes / m_e2Periodicity; // unit kbps

    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << std::to_string(m_cellId) << " cell, connected UE with IMSI " << imsi 
      << " ueImsiString " << ueImsiComplete
      << " txDlPackets " << txDlPackets 
      << " txDlPacketsNr " << txPdcpPduNrRlc
      << " txBytes " << txBytes 
      << " txDlBytesNr " << txPdcpPduBytesNrRlc
      << " pdcpLatency " << pdcpLatency 
      << " pdcpThroughput " << pdcpThroughput);

    m_e2PdcpStatsCalculator->ResetResultsForImsiLcid (imsi, 3);

      if (!indicationMessageHelper->IsOffline ()){
          indicationMessageHelper->AddCuUpUePmItem (ueImsiComplete, txBytes, txDlPackets,
                                                    pdcpThroughput, pdcpLatency);
      }

    uePmString.insert(std::make_pair(imsi, std::to_string(txBytes) + "," +
      std::to_string(txDlPackets) + "," +
      std::to_string(pdcpThroughput) + "," +
      std::to_string(pdcpLatency)));
  }

  // get average cell latency
  double cellAverageLatency = 0; 
  if (!ueMap.empty ())
  {
    cellAverageLatency = perUserAverageLatencySum / ueMap.size();
  }
    
  NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << std::to_string(m_cellId) << " cell, connected UEs number " << ueMap.size() 
      << " cellAverageLatency " << cellAverageLatency
    );

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->AddCuUpCellPmItem (cellAverageLatency);
    }

  // PDCP volume for the whole cell
  if (!indicationMessageHelper->IsOffline ())
    {
      // pDCPBytesUL = 0 since it is not supported from the simulator
      indicationMessageHelper->FillCuUpValues (plmId, 0, cellDlTxVolume);
    }

  NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " " << std::to_string(m_cellId) << " cell volume " << cellDlTxVolume);

  if (m_forceE2FileLogging) {
    std::ofstream csv {};
    csv.open (m_cuUpFileName.c_str (),  std::ios_base::app);
    if (!csv.is_open ())
    {
      NS_FATAL_ERROR ("Can't open file " << m_cuUpFileName.c_str ());
    }

    uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now().GetMilliSeconds ();

    // the string is timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),
    // m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
    // Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
    // DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
    // DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)

    // the last two are not available on LTE
    for (auto ue : ueMap)
    {
      uint64_t imsi = ue.second->GetImsi();
      std::string ueImsiComplete = GetImsiString(imsi);

      auto uePms = uePmString.find(imsi)->second;

      std::string to_print = std::to_string(timestamp) + "," + 
        ueImsiComplete + "," +
        std::to_string(cellAverageLatency) + "," +
        std::to_string(0) + "," +
        std::to_string(cellDlTxVolume) + "," +
        uePms + ",,\n";

      csv << to_print;
    }
    csv.close();
    return nullptr;
    }
  else
    {
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}

Ptr<KpmIndicationMessage>
LteEnbNetDeviceOran::BuildRicIndicationMessageCuCp(std::string plmId)
{
  Ptr<LteIndicationMessageHelper> indicationMessageHelper =
      Create<LteIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                       m_forceE2FileLogging, m_reducedPmValues);

  auto ueMap = m_rrc->GetUeMap();
  auto ueMapSize = ueMap.size ();

  std::unordered_map<uint64_t, std::string> uePmString {};

  for (auto ue : ueMap)
  {
    uint64_t imsi = ue.second->GetImsi();
    std::string ueImsiComplete = GetImsiString(imsi);
    long numDrb = ue.second->GetDrbMap().size();

    if (!indicationMessageHelper->IsOffline ())
      {
        // DRB.RelActNbr.5QI.UEID not modeled in the simulator
        indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, numDrb, 0);
      }

    uePmString.insert(std::make_pair(imsi, std::to_string(numDrb) + "," +
      std::to_string(0)));
  }

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->FillCuCpValues (ueMapSize);
    }

  if (m_forceE2FileLogging) {
    std::ofstream csv {};
    csv.open (m_cuCpFileName.c_str (),  std::ios_base::app);
    if (!csv.is_open ())
    {
      NS_FATAL_ERROR ("Can't open file " << m_cuCpFileName.c_str ());
    }

    NS_LOG_DEBUG ("m_cuCpFileName open " << m_cuCpFileName);

    uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

    for (auto ue : ueMap)
    {
      uint64_t imsi = ue.second->GetImsi();
      std::string ueImsiComplete = GetImsiString(imsi);

      auto uePms = uePmString.find(imsi)->second;

      std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                             std::to_string (ueMapSize) + "," + uePms + ",,,,,,," +
                             "\n";

      NS_LOG_DEBUG(to_print);

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


// modified

template <typename A, typename B>
std::pair<B, A>
flip_pair (const std::pair<A, B> &p)
{
  return std::pair<B, A> (p.second, p.first);
}

template <typename A, typename B>
std::multimap<B, A>
flip_map (const std::map<A, B> &src)
{
  std::multimap<B, A> dst;
  std::transform (src.begin (), src.end (), std::inserter (dst, dst.begin ()), flip_pair<A, B>);
  return dst;
}

Ptr<KpmIndicationMessage>
LteEnbNetDeviceOran::BuildMillicarReportRicIndicationMessageCucp(std::string plmId, uint16_t generatingNodeRnti, uint16_t numReports)
{
  NS_LOG_FUNCTION(this);
  Ptr<LteIndicationMessageHelper> indicationMessageHelper =
      Create<LteIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                             m_forceE2FileLogging, m_reducedPmValues);
    uint64_t imsi = (uint64_t) generatingNodeRnti;
    std::string ueImsiComplete = GetImsiString(imsi);
    std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator firstOrderIt = m_l3sinrMap.find(generatingNodeRnti);
    if(firstOrderIt!= m_l3sinrMap.end()){
        Ptr<L3RrcMeasurements> l3RrcMeasurementNeigh;
        if (!indicationMessageHelper->IsOffline ())
        {
          l3RrcMeasurementNeigh = L3RrcMeasurements::CreateL3RrcUeSpecificSinrNeigh ();
        }
        //invert key and value in sortFlipMap, then sort by value
        std::multimap<SinrMcsPair, uint16_t> sortFlipMap = flip_map (firstOrderIt->second);
        uint16_t nNeighbours = E2SM_REPORT_MAX_NEIGH;
        if (m_l3sinrMap[generatingNodeRnti].size () < nNeighbours)
          {
            nNeighbours = m_l3sinrMap[generatingNodeRnti].size () - 1;
          }
        int itIndex = 0;
        for (std::map<SinrMcsPair, uint16_t>::iterator it = --sortFlipMap.end ();
          it != --sortFlipMap.begin () && itIndex < nNeighbours; it--)
        {
          if (!indicationMessageHelper->IsOffline ())
          {
            l3RrcMeasurementNeigh->AddNeighbourCellMeasurementMcs (it->second, it->first.sinr, it->first.mcs);
          }
          itIndex++;
        }

        if (!indicationMessageHelper->IsOffline ())
        {
          indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, generatingNodeRnti, l3RrcMeasurementNeigh);
        }

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

void
LteEnbNetDeviceOran::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
  std::string plmId = "111";
  std::string gnbId = std::to_string(m_cellId);

  // TODO here we can get something from RRC and onward
  NS_LOG_DEBUG("LteEnbNetDeviceOran " << std::to_string(m_cellId) << " BuildAndSendMessage at time " << Simulator::Now().GetSeconds());
  
  if(m_sendCuCp)
  {
    // Create CU-CP
    Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    
    // create message for all the nodes in the list

    for (std::map<uint16_t, std::map<uint16_t, SinrMcsPair>>::iterator ueReportIt = m_l3sinrMap.begin();
    ueReportIt != m_l3sinrMap.end(); ++ ueReportIt){
      // Ptr<KpmIndicationMessage> cuCpMsg = BuildRicIndicationMessageCuCp(plmId);
      Ptr<KpmIndicationMessage> cuCpMsg = BuildMillicarReportRicIndicationMessageCucp(plmId, ueReportIt->first, m_l3sinrMap.size());
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
    // m_l3sinrMap.clear();
  }
  
  if (!m_forceE2FileLogging)
    Simulator::ScheduleWithContext (1, Seconds (m_e2Periodicity),
                                    &LteEnbNetDeviceOran::BuildAndSendReportMessage, this, params);
  else
    Simulator::Schedule (Seconds (m_e2Periodicity), &LteEnbNetDeviceOran::BuildAndSendReportMessage,
                         this, params);
}

void
LteEnbNetDeviceOran::SetStartTime (uint64_t st)
{
  m_startTime = st;
}

} // namespace ns3
