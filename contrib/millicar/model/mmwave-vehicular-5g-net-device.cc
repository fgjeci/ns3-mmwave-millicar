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
 *   Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                               Integration of Carrier Aggregation
 */

#define NS_LOG_APPEND_CONTEXT                                            \
do                                                                     \
  {                                                                    \
    std::clog << Simulator::Now() << " [ Rnti " << GetRnti() << "] ";  \
  }                                                                    \
while (false);

#include "mmwave-vehicular-5g-net-device.h"

#include <ns3/mmwave-enb-net-device.h>
#include <ns3/mmwave-net-device.h>
#include <ns3/mmwave-ue-phy.h>

#include "ns3/mmwave-component-carrier-ue.h"
#include <ns3/callback.h>
#include <ns3/enum.h>
#include <ns3/ipv4-header.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv4.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/llc-snap-header.h>
#include <ns3/log.h>
#include <ns3/lte-ue-component-carrier-manager.h>
#include <ns3/node.h>
#include <ns3/packet-burst.h>
#include <ns3/packet.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/uinteger.h>

//// millicar include 
#include <ns3/object-map.h>
#include <ns3/ipv6-header.h>
#include <ns3/epc-tft.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-tm.h>
#include <ns3/lte-radio-bearer-tag.h>
#include <ns3/mmwave-sidelink-mac.h>
#include <ns3/mmwave-vehicular-net-device.h>

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

#include <ns3/mmwave-vehicular-indication-message-helper.h>
#include <ns3/config.h>
#include "mmwave-packet-relay-tag.h"
#include "mmwave-packet-relay-header.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWaveMillicarUeNetDevice");

namespace mmwave
{

PdcpSpecificSidelinkPdcpSapUser::PdcpSpecificSidelinkPdcpSapUser (Ptr<MmWaveMillicarUeNetDevice> netDevice)
  : m_netDevice (netDevice)
{

}

void
PdcpSpecificSidelinkPdcpSapUser::ReceivePdcpSdu (ReceivePdcpSduParameters params)
{
  // NS_LOG_FUNCTION(this);
  // m_netDevice->Receive (params.pdcpSdu);
  m_netDevice->ReceiveMillicar (params.pdcpSdu);
}

uint16_t
PdcpSpecificSidelinkPdcpSapUser::GetRnti() const{
  if(m_netDevice!=nullptr){
    return m_netDevice->GetRnti();
  }
  return 0;
}

NS_OBJECT_ENSURE_REGISTERED(MmWaveMillicarUeNetDevice);

TypeId
MmWaveMillicarUeNetDevice::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::MmWaveMillicarUeNetDevice")
            .SetParent<MmWaveNetDevice>()
            .AddConstructor<MmWaveMillicarUeNetDevice>()
            .AddAttribute("EpcUeNas",
                          "The NAS associated to this UeNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_nas),
                          MakePointerChecker<EpcUeNas>())
            .AddAttribute("mmWaveUeRrc",
                          "The RRC associated to this UeNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_rrc),
                          MakePointerChecker<LteUeRrc>())
            .AddAttribute("LteUeComponentCarrierManager",
                          "The ComponentCarrierManager associated to this UeNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_componentCarrierManager),
                          MakePointerChecker<LteUeComponentCarrierManager>())
            .AddAttribute("Imsi",
                          "International Mobile Subscriber Identity assigned to this UE",
                          UintegerValue(0),
                          MakeUintegerAccessor(&MmWaveMillicarUeNetDevice::m_imsi),
                          MakeUintegerChecker<uint64_t>())
            .AddAttribute("LteUeRrc",
                          "The RRC layer associated with the ENB",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_rrc),
                          MakePointerChecker<LteUeRrc>())
            .AddAttribute ("SidelinkRadioBearerMap", "List of SidelinkRadioBearerMap by BID",
                          ObjectMapValue (),
                          MakeObjectMapAccessor (&MmWaveMillicarUeNetDevice::m_bearerToInfoMapMillicar),
                          MakeObjectMapChecker<millicar::SidelinkRadioBearerInfo> ())
            .AddAttribute ("MtuMillicar", "The MAC-level Maximum Transmission Unit",
                          UintegerValue (30000),
                          MakeUintegerAccessor (&MmWaveMillicarUeNetDevice::SetMtuMillicar,
                                                  &MmWaveMillicarUeNetDevice::GetMtuMillicar),
                          MakeUintegerChecker<uint16_t> ())
            .AddAttribute ("RlcType",
                          "Set the RLC mode to use (AM not supported for now)",
                          StringValue ("LteRlcTm"),
                          MakeStringAccessor (&MmWaveMillicarUeNetDevice::m_rlcTypeMillicar),
                          MakeStringChecker ())
            // mac and physical pointer
            .AddAttribute("MmWaveSidelinkMac",
                          "The sideling mac layer of the mmwave device",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_macMillicar),
                          MakePointerChecker<millicar::MmWaveSidelinkMac>())
            .AddAttribute("MmWaveSidelinkPhy",
                          "The sidelink phy layer of the mmwave device",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveMillicarUeNetDevice::m_phyMillicar),
                          MakePointerChecker<millicar::MmWaveSidelinkPhy>())
            .AddTraceSource("SendPacketReport",
                          "Report of send packets",
                          MakeTraceSourceAccessor (&MmWaveMillicarUeNetDevice::m_sendPacketTrace),
                          "ns3::millicar::MmWaveMillicarUeNetDevice::SendPacketTracedCallback"
                          ) 
            .AddTraceSource("RelayPacketReport",
                          "Report of relay packets",
                          MakeTraceSourceAccessor (&MmWaveMillicarUeNetDevice::m_relayPacketTrace),
                          "ns3::millicar::MmWaveMillicarUeNetDevice::RelayPacketTracedCallback"
                          ) 
            // modified
            .AddAttribute ("TracesPath",
                          "The path where to store the path. ",
                          StringValue ("./"),
                          MakeStringAccessor (&MmWaveMillicarUeNetDevice::m_tracesPath),
                          MakeStringChecker ())
            .AddAttribute ("PhyTraceHelperMillicar",
                   "Trace helper for millicar",
                   PointerValue (),
                   MakePointerAccessor (&MmWaveMillicarUeNetDevice::m_phyTraceHelperMillicar),
                   MakePointerChecker <millicar::MmWaveVehicularTracesHelper> ())
    ;
    return tid;
}

MmWaveMillicarUeNetDevice::MmWaveMillicarUeNetDevice(void)
{
    NS_LOG_FUNCTION(this);
    // m_phyTraceHelperMillicar = CreateObject<millicar::MmWaveVehicularTracesHelper>();

}

MmWaveMillicarUeNetDevice::MmWaveMillicarUeNetDevice (Ptr<millicar::MmWaveSidelinkPhy> phy, Ptr<millicar::MmWaveSidelinkMac> mac)
{
  NS_LOG_FUNCTION (this);
  m_phyMillicar = phy;
  m_macMillicar = mac;
  // m_phyTraceHelperMillicar = CreateObject<millicar::MmWaveVehicularTracesHelper>();
}

MmWaveMillicarUeNetDevice::~MmWaveMillicarUeNetDevice(void)
{
}

void
MmWaveMillicarUeNetDevice::DoInitialize(void)
{
    NS_LOG_FUNCTION(this);
    m_isConstructed = true;
    UpdateConfig();

    for (auto it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
    {
        Ptr<MmWaveComponentCarrierUe> ccUe = DynamicCast<MmWaveComponentCarrierUe>(it->second);
        ccUe->GetPhy()->Initialize();
        ccUe->GetMac()->Initialize();
    }
    m_rrc->Initialize();

    // initializing millicar phy and mac
    m_phyMillicar->Initialize();
    m_macMillicar->Initialize();

    // RicControlMessage
    bool _retValue = Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/RicControlMessage",
    MakeBoundCallback (&MmWaveMillicarUeNetDevice::RegisterRicControlMessageCallback, this));

    NS_LOG_LOGIC("Registered to ric control traces sinr " << _retValue);
}

void 
MmWaveMillicarUeNetDevice::PrintRelay(){
  NS_LOG_FUNCTION(this);
  for(auto it = m_relayPaths.begin(); it != m_relayPaths.end(); ++it)
  {
    for (auto secondIt = it->second.begin(); secondIt != it->second.end(); ++secondIt){
      NS_LOG_DEBUG("" << it->first << " " << secondIt->first << " " << secondIt->second);
    }
  }
}

void
MmWaveMillicarUeNetDevice::TestRelay(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti){
  NS_LOG_FUNCTION(this << " source " <<  localRnti << " dest " << destRnti << " intermediate " << intermediateRnti);
  m_relayPaths[localRnti][destRnti] = intermediateRnti;
  m_relayPaths[destRnti][localRnti] = intermediateRnti;
  // have to update the destination of rlc buffer in m_bearerToInfoMapMillicar
  if ((localRnti == GetRnti()) || (destRnti == GetRnti())){
    // if from this rnti we have to change route, then all packets meant to be sent to destRnti
    // should now pass through intermediateRnti
    // for this we change mac, since it is more controllable, rather than rlc
    // AddRelayPath
    m_macMillicar->AddRelayPath(localRnti, destRnti, intermediateRnti);
    m_phyMillicar->AddRelayPath(localRnti, destRnti, intermediateRnti);
  }
  PrintRelay();
}



void
MmWaveMillicarUeNetDevice::RegisterToMillicarEnbTraces(){
  // connect to mac sidelink trace 
    // connect to callback
    // bool _retValue = Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/RicControlMessage",
    // MakeBoundCallback (&MmWaveMillicarUeNetDevice::RegisterRicControlMessageCallback, this));

    // NS_LOG_DEBUG("Registered to sl mac sinr " << _retValue);
}

// end modification
void
MmWaveMillicarUeNetDevice::RegisterRicControlMessageCallback(Ptr<MmWaveMillicarUeNetDevice> netDev, std::string context, uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti)
{
  netDev->RegisterRicControlMessage(localRnti, destRnti, intermediateRnti);
}

void
MmWaveMillicarUeNetDevice::RegisterRicControlMessage(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti)
{
  // NS_LOG_FUNCTION(this << localRnti << " " << destRnti << " intermediate " << intermediateRnti);

  m_relayPaths[localRnti][destRnti] = intermediateRnti;
  m_relayPaths[destRnti][localRnti] = intermediateRnti;
  // NS_LOG_LOGIC (Simulator::Now ().GetSeconds ()
  //               << " uedev " << localRnti << " report for " << destRnti
  //               << " SINR " << m_l3sinrMillicarMap[localRnti][destRnti].sinr);
  // have to update the destination of rlc buffer in m_bearerToInfoMapMillicar
  if ((localRnti == GetRnti()) || (destRnti == GetRnti())){
    // if from this rnti we have to change route, then all packets meant to be sent to destRnti
    // should now pass through intermediateRnti
    // for this we change mac, since it is more controllable, rather than rlc
    // AddRelayPath
    m_macMillicar->AddRelayPath(localRnti, destRnti, intermediateRnti);
    m_phyMillicar->AddRelayPath(localRnti, destRnti, intermediateRnti);
  }

  PrintRelay();
}

// modified

void
MmWaveMillicarUeNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_targetEnb = 0;

    m_rrc->Dispose();
    m_rrc = 0;

    m_nas->Dispose();
    m_nas = 0;

    for (uint32_t i = 0; i < m_ccMap.size(); i++)
    {
        m_ccMap.at(i)->Dispose();
    }
    m_componentCarrierManager->Dispose();
}



uint32_t
MmWaveMillicarUeNetDevice::GetCsgId() const
{
    NS_LOG_FUNCTION(this);
    return m_csgId;
}

void
MmWaveMillicarUeNetDevice::SetCsgId(uint32_t csgId)
{
    NS_LOG_FUNCTION(this << csgId);
    m_csgId = csgId;
    UpdateConfig(); // propagate the change down to NAS and RRC
}

void
MmWaveMillicarUeNetDevice::UpdateConfig(void)
{
    NS_LOG_FUNCTION(this);

    if (m_isConstructed)
    {
        NS_LOG_LOGIC(this << " Updating configuration: IMSI " << m_imsi << " CSG ID " << m_csgId);
        m_nas->SetImsi(m_imsi);
        m_rrc->SetImsi(m_imsi);
        m_nas->SetCsgId(m_csgId); // this also handles propagation to RRC
    }
    else
    {
        /*
         * NAS and RRC instances are not be ready yet, so do nothing now and
         * expect ``DoInitialize`` to re-invoke this function.
         */
    }

    // Simulator::Schedule(MilliSeconds(40), &MmWaveMillicarUeNetDevice::BuildAndSendReportMessage, this, E2Termination::RicSubscriptionRequest_rval_s{});
}

bool
MmWaveMillicarUeNetDevice::DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    // NS_LOG_FUNCTION(this << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                        protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber
                                            << ", only IPv4 and IPv6 are supported");
    // in this appliation the devices only communicate one another
    // in a more generic scenario, should be decided the stack to be sent to
    
    return SendMillicar(packet, dest, protocolNumber);
    // return m_nas->Send(packet, protocolNumber);
}

bool
MmWaveMillicarUeNetDevice::Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    // NS_LOG_FUNCTION(this << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                        protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber
                                            << ", only IPv4 and IPv6 are supported");
    // in this appliation the devices only communicate one another
    // in a more generic scenario, should be decided the stack to be sent to
    
    return SendMillicar(packet, dest, protocolNumber);
    // return m_nas->Send(packet, protocolNumber);
}

Ptr<KpmIndicationHeader>
MmWaveMillicarUeNetDevice::BuildRicIndicationHeader (std::string plmId, std::string gnbId,
                                              uint16_t nrCellId)
{
  NS_LOG_FUNCTION(this);
    KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
    headerValues.m_plmId = plmId;
    headerValues.m_gnbId = gnbId;
    headerValues.m_nrCellId = nrCellId;
    auto time = Simulator::Now ();
    uint64_t timestamp = 0 + (uint64_t) time.GetMilliSeconds ();
    NS_LOG_DEBUG ("NR plmid " << plmId << " gnbId " << gnbId << " nrCellId " << nrCellId);
    NS_LOG_DEBUG ("Timestamp " << timestamp);
    headerValues.m_timestamp = timestamp;

    Ptr<KpmIndicationHeader> header =
        Create<KpmIndicationHeader> (KpmIndicationHeader::GlobalE2nodeType::gNB, headerValues);

    return header;
}

Ptr<KpmIndicationMessage>
MmWaveMillicarUeNetDevice::BuildRicIndicationMessageCp(std::string plmId, uint16_t nrCellId)
{
  NS_LOG_FUNCTION(this);
  Ptr<MmWaveVehicularIndicationMessageHelper> indicationMessageHelper =
      Create<MmWaveVehicularIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                             false, true);

  indicationMessageHelper->AddCpUePmTestItem(1, 100);
  indicationMessageHelper->FillCuCpValues(0); // plmId + std::to_string (nrCellId)

  return indicationMessageHelper->CreateIndicationMessage();
}

void
MmWaveMillicarUeNetDevice::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
  NS_LOG_FUNCTION(this);
  std::string plmId = "111";
  // std::string gnbId = std::to_string(m_cellId);
  Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, plmId, 1);
  Ptr<KpmIndicationMessage> msg = BuildRicIndicationMessageCp(plmId, 1);

  E2AP_PDU *pdu = new E2AP_PDU; 

  encoding::generate_e2apv1_indication_request_parameterized(pdu, 
                                                               params.requestorId,
                                                               params.instanceId,
                                                               params.ranFuncionId,
                                                               params.actionId,
                                                               1, // TODO sequence number  
                                                               (uint8_t*) header->m_buffer, // buffer containing the encoded header
                                                               header->m_size, // size of the encoded header
                                                               (uint8_t*) msg->m_buffer, // buffer containing the encoded message
                                                               msg->m_size); // size of the encoded message  

  // once we have built it, we can send it to the baes station

  SendE2Msg(pdu);

  Simulator::Schedule(MilliSeconds(100), &MmWaveMillicarUeNetDevice::BuildAndSendReportMessage, this, E2Termination::RicSubscriptionRequest_rval_s{});
}

void
MmWaveMillicarUeNetDevice::SendE2Msg(E2AP_PDU *pdu){
    NS_LOG_FUNCTION(this);
    uint8_t *buf;
    int len = e2ap_asn1c_encode_pdu(pdu, &buf);
    // calling ue rrc to send the message
    m_rrc->SendE2MessageBuffer(buf, len);

}

void
MmWaveMillicarUeNetDevice::ProcessE2Message(uint8_t* buffer, size_t buffSize){
  NS_LOG_FUNCTION(this << buffSize);
  // in the buffer send by the ue, has only the message payload of a ric control message
  // the header we insert in in the enb and eventually send it to the xapp

  uint8_t idx;
  
	E2AP_PDU_t *pdu = nullptr;
	auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2AP_PDU, (void **) &pdu, (void *)buffer, buffSize);
	// print decoded payload
	if (retval.code == RC_OK) {

    // we have to decode the decode the message payload
    //first we check the coise of E2AP pdu. 
    switch (pdu->present)
    {
    case E2AP_PDU_PR_successfulOutcome:
      {
          RICcontrolAcknowledge_t* ricControlAck = &pdu->choice.successfulOutcome->value.choice.RICcontrolAcknowledge;

      }
      break;

    case E2AP_PDU_PR_initiatingMessage:
      {
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

              E2SM_KPM_IndicationMessage_t *descriptor = nullptr;

              auto retvalMsgKpm = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2SM_KPM_IndicationMessage, (void **) &descriptor, payload, payload_size);

              char *printBufferMessage;
              size_t sizeMessage;
              FILE *streamMessage = open_memstream(&printBufferMessage, &sizeMessage);
              xer_fprint(streamMessage, &asn_DEF_E2SM_KPM_IndicationMessage, descriptor);

              NS_LOG_DEBUG("Shoud finish here");
              // auto res = 1/0;
            }
          }
      }
    
    default:
      break;
    }
    
  }

  delete pdu;
	
}

// millicar part


Ptr<millicar::MmWaveSidelinkMac>
MmWaveMillicarUeNetDevice::GetMacMillicar (void) const
{
  // NS_LOG_FUNCTION (this);
  return m_macMillicar;
}

Ptr<millicar::MmWaveSidelinkPhy>
MmWaveMillicarUeNetDevice::GetPhyMillicar (void) const
{
  // NS_LOG_FUNCTION (this);
  return m_phyMillicar;
}

bool
MmWaveMillicarUeNetDevice::SetMtuMillicar (const uint16_t mtu)
{
  m_mtuMillicar = mtu;
  return true;
}

uint16_t
MmWaveMillicarUeNetDevice::GetMtuMillicar (void) const
{
  return m_mtuMillicar;
}

TypeId
MmWaveMillicarUeNetDevice::GetRlcTypeMillicar (std::string rlcType)
{
  if (rlcType == "LteRlcSm")
  {
    return LteRlcSm::GetTypeId ();
  }
  else if (rlcType == "LteRlcUm")
  {
    return LteRlcUm::GetTypeId ();
  }
  else if (rlcType == "LteRlcTm")
  {
    return LteRlcTm::GetTypeId ();
  }
  else
  {
    NS_FATAL_ERROR ("Unknown RLC type");
  }
}

void
MmWaveMillicarUeNetDevice::ActivateBearerMillicar(const uint8_t bearerId, const uint16_t destRnti, const Address& dest)
{
  NS_LOG_FUNCTION(this << bearerId);
  uint8_t lcid = bearerId; // set the LCID to be equal to the bearerId
  // future extensions could consider a different mapping
  // and will actually exploit the following map
  std::cout << "Rnti " << m_macMillicar->GetRnti() << " dest rnti " << destRnti 
            << " bearerID " << +bearerId << " addr " << dest << std::endl;
  m_bid2lcidMillicar.insert(std::make_pair(bearerId, lcid));

  NS_ASSERT_MSG(m_bearerToInfoMapMillicar.find (bearerId) == m_bearerToInfoMapMillicar.end (),
    "There's another bearer associated to this bearerId: " << uint32_t(bearerId));

  EpcTft::PacketFilter slFilter;
  slFilter.remoteAddress= Ipv4Address::ConvertFrom(dest);

  Ptr<Node> node = GetNode ();
  Ptr<Ipv4> nodeIpv4 = node->GetObject<Ipv4> ();
  int32_t interface =  nodeIpv4->GetInterfaceForDevice (this);
  Ipv4Address src = nodeIpv4->GetAddress (interface, 0).GetLocal ();
  slFilter.localAddress= Ipv4Address::ConvertFrom(src);
  //slFilter.direction= EpcTft::DOWNLINK;
  slFilter.remoteMask= Ipv4Mask("255.255.255.255");
  slFilter.localMask= Ipv4Mask("255.255.255.255");

  NS_LOG_DEBUG(this << " Add filter for " << Ipv4Address::ConvertFrom(dest));

  Ptr<EpcTft> tft = Create<EpcTft> (); // Create a new tft
  tft->Add (slFilter); // Add the packet filter

  m_tftClassifierMillicar.Add(tft, bearerId);

  // Create RLC instance with specific RNTI and LCID
  ObjectFactory rlcObjectFactory;
  rlcObjectFactory.SetTypeId (GetRlcTypeMillicar(m_rlcTypeMillicar));
  Ptr<LteRlc> rlc = rlcObjectFactory.Create ()->GetObject<LteRlc> ();

  // modified
  
  // rlc->TraceConnectWithoutContext("BufferSize",
	// 										   MakeCallback(&millicar::MmWaveVehicularTracesHelper::UeRlcBufferSize, m_phyTraceHelperMillicar, GetRnti(),  m_imsi));
  NS_LOG_DEBUG("Attaching to trace " << m_phyTraceHelperMillicar);
  rlc->TraceConnectWithoutContext("BufferSize",
											   MakeBoundCallback(&millicar::MmWaveVehicularTracesHelper::UeRlcBufferSizeShort, m_phyTraceHelperMillicar, GetRnti()));
  // end modification

  rlc->SetLteMacSapProvider (m_macMillicar->GetMacSapProvider());
  rlc->SetRnti (destRnti); // this is the rnti of the destination
  rlc->SetLcId (lcid);

  // Call to the MAC method that created the SAP for binding the MAC instance on this node to the RLC instance just created
  m_macMillicar->AddMacSapUser(lcid, rlc->GetLteMacSapUser());

  Ptr<LtePdcp> pdcp = CreateObject<LtePdcp> ();
  pdcp->SetRnti (destRnti); // this is the rnti of the destination
  pdcp->SetLcId (lcid);

  // Create the PDCP SAP that connects the PDCP instance to this NetDevice
  LtePdcpSapUser* pdcpSapUser = new PdcpSpecificSidelinkPdcpSapUser (this);
  pdcp->SetLtePdcpSapUser (pdcpSapUser);
  pdcp->SetLteRlcSapProvider (rlc->GetLteRlcSapProvider ());
  rlc->SetLteRlcSapUser (pdcp->GetLteRlcSapUser ());
  rlc->Initialize (); // this is needed to trigger the BSR procedure if RLC SM is selected

  Ptr<millicar::SidelinkRadioBearerInfo> rbInfo = CreateObject<millicar::SidelinkRadioBearerInfo> ();
  rbInfo->m_rlc= rlc;
  rbInfo->m_pdcp = pdcp;
  rbInfo->m_rnti = destRnti;

  NS_LOG_DEBUG(this << " MmWaveMillicarUeNetDevice::ActivateBearer() bid: " << (uint32_t)bearerId << " rnti: " << destRnti);

  // insert the tuple <lcid, pdcpSapProvider> in the map of this NetDevice, so that we are able to associate it to them later
  m_bearerToInfoMapMillicar.insert (std::make_pair (bearerId, rbInfo));
}

void
MmWaveMillicarUeNetDevice::ReceiveMillicar (Ptr<Packet> p)
{
  // NS_LOG_FUNCTION (this << p);
  // NS_LOG_DEBUG ("Received packet at: " << Simulator::Now().GetSeconds() << "s");
  uint8_t ipType;
  // have to check that packet is relay packet or not
  MmWavePacketRelayTag packetRelayTag;
  bool hasRelayTag = p->PeekPacketTag(packetRelayTag);
  // MmWavePacketRelayHeader packetRelayHeader;
  // p->RemoveHeader(packetRelayHeader);
  // bool hasRelayHeader = packetRelayHeader.GetDestinationRnti()!=UINT16_MAX;

  p->CopyData (&ipType, 1);
  ipType = (ipType>>4) & 0x0f;

  Ipv4Header ipv4Header;
  p->PeekHeader(ipv4Header);
  // if (std::find(_debug_vecto.begin(), _debug_vecto.end(), GetPhyMillicar()->GetRnti())!= _debug_vecto.end()){
  NS_LOG_DEBUG("Rec S " << ipv4Header.GetSource() 
              << " d " << ipv4Header.GetDestination() 
              << " tag " << packetRelayTag.GetDestinationRnti()
              << " seq " << p->GetUid()
              << " peek " << hasRelayTag
              // << " isrelay " << hasRelayHeader
              // << " size " << p->GetSize()
              
              );
  // }
  
  
  if(hasRelayTag){
  // if(hasRelayHeader){
    // Don't remove the tag to avoid errors in case of multihop
    // p->RemovePacketTag(packetRelayTag);
    // Address destAddress = packetRelayTag.GetDestinationAddress();
    // uint16_t protocolNumber = packetRelayTag.GetProtocolTypeNumber();
    uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
    uint16_t sourceRnti = packetRelayTag.GetSourceRnti();
    uint16_t intermediateRnti = packetRelayTag.GetIntermediateRnti();
    Vector _pos = GetNode()->GetObject<MobilityModel> ()->GetPosition ();
    m_relayPacketTrace(p, GetRnti(), _pos);
    // uint16_t destinationRnti = packetRelayHeader.GetDestinationRnti();
    // if the destination is not the local rnti we relay otherwise we send it to application layer
    if (destinationRnti!=GetRnti()){
      // means we have to relay the packet 
      RelayMillicar(p, destinationRnti);
      // this packet is for relay only, thus we do not forward in upper layers
      return;
    }
    
  }
  
  NS_LOG_DEBUG("packet up");


  if (ipType == 0x04)
  {
    m_rxCallback (this, p, Ipv4L3Protocol::PROT_NUMBER, Address ());
  }
  else if (ipType == 0x06)
  {
    m_rxCallback (this, p, Ipv6L3Protocol::PROT_NUMBER, Address ());
  }
  else
  {
    NS_ABORT_MSG ("MmWaveMillicarUeNetDevice::Receive - Unknown IP type...");
  }
}

// this shoudl be different from the regular send function as the packet
// doesn't have the source and destination such that it can produce
// the lcid needed to define the destination
bool
MmWaveMillicarUeNetDevice::RelayMillicar (Ptr<Packet> packet, uint16_t destinationRnti){
  NS_LOG_FUNCTION(this<< "local rnti" << GetRnti() << "dest rnti" << destinationRnti);
  Ipv4Header ipv4Header;
  uint32_t ipv4Size = packet->PeekHeader(ipv4Header);
  
  Ptr<millicar::SidelinkRadioBearerInfo> bearerInfo;
  
  for (auto bearerIt = m_bearerToInfoMapMillicar.begin(); bearerIt!=m_bearerToInfoMapMillicar.end(); ++bearerIt)
  {
    if(bearerIt->second->m_rnti == destinationRnti){
      bearerInfo = bearerIt->second;  
    }
  }

  NS_ASSERT_MSG(bearerInfo!=nullptr, "Cannot relay packet from " << GetRnti() 
                                    << " to unreachable destination " << destinationRnti);
  LtePdcpSapProvider::TransmitPdcpSduParameters params;
  params.pdcpSdu = packet;
  params.rnti = destinationRnti;
  params.lcid = 0;

  MmWavePacketRelayTag packetRelayTag;
  bool hasRelayTag = packet->RemovePacketTag(packetRelayTag);

  // Don't remove packet tags, to avoid errors in multi-hop scenarios
  packet->RemoveAllPacketTags (); // remove all tags in case there is any
  // re insert the packet tag


  // MmWavePacketRelayTag packetRelayTag;
  // packetRelayTag.SetDestinationRnti(destinationRnti);
  packet->AddPacketTag(packetRelayTag);


  params.pdcpSdu = packet;
  bearerInfo->m_pdcp->GetLtePdcpSapProvider()->TransmitPdcpSdu (params);

  // if (std::find(_debug_vecto.begin(), _debug_vecto.end(), GetPhyMillicar()->GetRnti())!= _debug_vecto.end()){
  NS_LOG_DEBUG("Rel S " << ipv4Header.GetSource() 
              << " d " << ipv4Header.GetDestination() 
              << " tag " << packetRelayTag.GetDestinationRnti()
              // << " size " << ipv4Size
              << " seq " << packet->GetUid()
              
              );
  // }

  return true;
}

uint16_t
MmWaveMillicarUeNetDevice::GetRnti() const{
  if (m_phyMillicar!=nullptr){
    return m_phyMillicar->GetRnti();
  }
  return 0;
}

bool
MmWaveMillicarUeNetDevice::SendMillicar (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
  // NS_LOG_FUNCTION (this<< dest);
  // classify the incoming packet
  uint32_t id = m_tftClassifierMillicar.Classify (packet, EpcTft::UPLINK, protocolNumber);
  NS_ASSERT ((id & 0xFFFFFF00) == 0);
  uint8_t bid = (uint8_t) (id & 0x000000FF);
  // source and destination
  Ipv4Header ipv4Header;
  uint32_t ipv4Size = packet->PeekHeader(ipv4Header);

  // if there is an error and could not find the lcid we simply do not send the packet
  // this happens only when we have activated relay
  if (m_bid2lcidMillicar.find(bid) == m_bid2lcidMillicar.end()){
    std::cout << "BearerId to LCID mapping not found " << +bid << " in rnti " << GetPhyMillicar()->GetRnti()
    << " for packet with id " << packet->GetUid() <<std::endl;
    return false;
  }

  uint8_t lcid = BidToLcidMillicar(bid);
  // NS_LOG_DEBUG("sending packet ");

  // get the SidelinkRadioBearerInfo
  NS_ASSERT_MSG(m_bearerToInfoMapMillicar.find (bid) != m_bearerToInfoMapMillicar.end (), "No logical channel associated to this communication");
  auto bearerInfo = m_bearerToInfoMapMillicar.find (bid)->second;

  // modified
  // here we have to check wether we have active relay routes from the ric control
  MmWavePacketRelayTag packetRelayTag;
  // MmWavePacketRelayHeader packetRelayHeader;
  packetRelayTag.SetDestinationRnti(bearerInfo->m_rnti);
  packetRelayTag.SetSourceRnti(GetRnti());
  
  uint16_t intermediateNodeRnti = UINT16_MAX;
  std::map<uint16_t, std::map<uint16_t, uint16_t>>::iterator relayPathsIt = m_relayPaths.find(GetPhyMillicar()->GetRnti());
  if(relayPathsIt!= m_relayPaths.end()){
    // check if also the destination is found
    std::map<uint16_t, uint16_t>::iterator destPathIt = relayPathsIt->second.find(bearerInfo->m_rnti);
    if(destPathIt != relayPathsIt->second.end()){
      // get the intermediate node
      intermediateNodeRnti = destPathIt->second;
      // create the header of relayed packet
      // packetRelayTag.SetProtocolTypeNumber(protocolNumber);
      // packetRelayTag.SetDestinationAddress(dest);
      // packetRelayTag.SetDestinationRnti(bearerInfo->m_rnti);
      // packetRelayHeader.SetDestinationRnti(bearerInfo->m_rnti);
      packetRelayTag.SetIntermediateRnti(intermediateNodeRnti);

      // have to find the bearerinfo for that communication
      // loop through m_bearerToInfoMapMillicar until we find it
      for (auto bearerIt = m_bearerToInfoMapMillicar.begin(); bearerIt!=m_bearerToInfoMapMillicar.end(); ++bearerIt)
      {
        if(bearerIt->second->m_rnti == intermediateNodeRnti){
          // get the bearer info for the intermediate node, since we are sending the packet to
          // this intermediate node
          // if (std::find(_debug_vecto.begin(), _debug_vecto.end(), GetPhyMillicar()->GetRnti())!= _debug_vecto.end()){
            NS_LOG_DEBUG("Sr " << GetRnti() <<
            " dr " << bearerInfo->m_rnti << " ir " << intermediateNodeRnti
            << " tag " << packetRelayTag.GetDestinationRnti()
            // << " header " << packetRelayHeader.GetDestinationRnti()
            );
          // }
          
          // NS_LOG_DEBUG("Old lcid " << +lcid << " new lcid " << +bearerIt->first);
          // we set the bearer to the intermediate node
          bearerInfo = bearerIt->second;
          lcid = bearerIt->first;
        }
      }
    }
  }

  // end modification

  LtePdcpSapProvider::TransmitPdcpSduParameters params;
  // params.pdcpSdu = packet;
  params.rnti = bearerInfo->m_rnti;
  params.lcid = lcid;

  // NS_LOG_DEBUG(this << " MmWaveMillicarUeNetDevice::Send() bid " << (uint32_t)bid << " lcid " << (uint32_t)lcid << " rnti " << bearerInfo->m_rnti);
  // Remove all previous tags the packet might have and insert new one to do the relay functionality
  packet->RemoveAllPacketTags (); // remove all tags in case there is any

  // modified
  // if (packetRelayTag.GetDestinationRnti() != UINT16_MAX){
    // NS_LOG_DEBUG("Adding tag " << packetRelayTag.GetDestinationRnti());
  packet->AddPacketTag(packetRelayTag);
  // }

  
  Vector _pos = GetNode()->GetObject<MobilityModel> ()->GetPosition ();
  
  // Add header in all cases & filter acceptable rntis in receive 
  // if (packetRelayHeader.GetDestinationRnti() != UINT16_MAX){
  // NS_LOG_DEBUG("Adding header " << packetRelayHeader.GetDestinationRnti()
  //             << " packet size " << packet->GetSize());
  // packet->AddHeader(packetRelayHeader);
  // }

  // end modification

  // if (std::find(_debug_vecto.begin(), _debug_vecto.end(), GetPhyMillicar()->GetRnti())!= _debug_vecto.end()){
  NS_LOG_DEBUG("send S " << ipv4Header.GetSource() 
                << " d " << ipv4Header.GetDestination() 
                << " tag " << packetRelayTag.GetDestinationRnti()
                // << " bid " << +bid
                // << " id " << id
                // << " size header " << ipv4Size
                // << " packet size " << packet->GetSize()
                << " seq " << packet->GetUid()
                << " lr " << GetPhyMillicar()->GetRnti()
                
                );
  // }
  m_sendPacketTrace(packet, GetRnti(), 
                    m_bearerToInfoMapMillicar.find (bid)->second->m_rnti, // destination
                    intermediateNodeRnti, GetRnti(), _pos);

  params.pdcpSdu = packet;
  bearerInfo->m_pdcp->GetLtePdcpSapProvider()->TransmitPdcpSdu (params);

  return true;
}

uint8_t
MmWaveMillicarUeNetDevice::BidToLcidMillicar(const uint8_t bearerId) const
{
  NS_ASSERT_MSG(m_bid2lcidMillicar.find(bearerId) != m_bid2lcidMillicar.end(),
    "BearerId to LCID mapping not found " << +bearerId << " in rnti " << GetPhyMillicar()->GetRnti());
  return m_bid2lcidMillicar.find(bearerId)->second;
}

void
MmWaveMillicarUeNetDevice::SetAntennaArrayMillicar (Ptr<UniformPlanarArray> antenna)
{
  NS_LOG_FUNCTION (this);
  m_antennaMillicar = antenna;
}

void
MmWaveMillicarUeNetDevice::MillicarSinrMapReport(uint16_t rnti, std::map<uint64_t, double> mapReport){
  // send through rrc 
  // we send the data by traces
  // cycle on all the Imsi whose SINR is known in cell mmWaveCellId
    for (std::map<uint64_t, double>::iterator peerMapIter = mapReport.begin();
         peerMapIter != mapReport.end();
         ++peerMapIter)
    {
      // rnti is the id of the device generating the report; in the map first index is the peer id
      // second index is the sinr
      m_notifyMillicarSinrTrace(rnti, peerMapIter->first, peerMapIter->second);
    }
}

Ptr<UniformPlanarArray>
MmWaveMillicarUeNetDevice::GetAntennaArrayMillicar () const
{
  // NS_LOG_FUNCTION (this);
  return m_antennaMillicar;
}

} // namespace mmwave
} // namespace ns3
