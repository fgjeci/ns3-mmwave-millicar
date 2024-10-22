
#include "stats-helper.h"
#include <ns3/network-module.h>
#include <ns3/mmwave-packet-relay-tag.h>
#include <ns3/seq-ts-size-header.h>
#include <ns3/udp-l4-protocol.h>
#include <ns3/udp-header.h>
#include <ns3/lte-rlc-header.h>
#include <ns3/lte-rlc-tag.h>
#include <ns3/lte-pdcp-tag.h>
// #include "ns3/mmwave-sidelink-mac.h"



namespace ns3 {


// namespace mmwave{


NS_LOG_COMPONENT_DEFINE("EfStatsHelper");
NS_OBJECT_ENSURE_REGISTERED (EfStatsHelper);

TypeId
EfStatsHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EfStatsHelper")
    .SetParent<Object> ()
    .SetGroupName ("EfStatsHelper")
	.AddConstructor<EfStatsHelper> ();
  return tid;
}

EfStatsHelper::EfStatsHelper(void){
    NS_LOG_FUNCTION (this);
}

EfStatsHelper::~EfStatsHelper (void)
{
  NS_LOG_FUNCTION (this);
}

void
EfStatsHelper::DoInitialize(void){
	NS_LOG_FUNCTION (this);
}

void
EfStatsHelper::SinrReportCallback (mmwave::SinrReportStats *stats, uint16_t sourceRnti, uint16_t rnti, uint8_t numSym, uint32_t tbSize,  double sinr)
{
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);
  	stats->SaveSinr (sourceRnti, rnti, numSym, tbSize, 0, sinr);
}

void
EfStatsHelper::AllPeersSinrReportCallback (mmwave::SinrReportStats *stats, uint16_t sourceRnti, uint64_t rnti,  double snr,  double sinr)
{
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);
  	stats->SaveSinr (sourceRnti, (uint16_t)rnti, 0, 0, snr, sinr);
}

void
EfStatsHelper::DecentralizedRelayReportCallback (mmwave::DecentralizedRelayStats *relayStats, mmwave::SfnSf sfnsf, uint16_t rnti, uint16_t destRnti, uint16_t intermediateRnti, 
							double directLinkSnr, double bestLinkSnr)
{
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);
  	relayStats->SaveDecentralizedRelayReport (sfnsf.m_frameNum, sfnsf.m_sfNum, sfnsf.m_slotNum, rnti, (uint16_t)destRnti, intermediateRnti, directLinkSnr, bestLinkSnr);
}

void
EfStatsHelper::PacketRelayLatencyReportCallback (mmwave::RelayLatencyStats *stats, Ptr<Packet> p, 
                                        uint16_t localRnti, uint16_t sourceRnti, 
                                        uint16_t destRnti, // uint16_t intermediateRnti, 
                                        // Vector pos ,const ns3::Address & from, const ns3::Address & to
                                        uint8_t traceSource
                                        )
{
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);
    Ptr<Packet> packet = p->Copy();

    // Ipv4Header ipv4Header;
    // uint32_t ipv4Size = packet->RemoveHeader(ipv4Header);

    uint32_t packetSize = packet->GetSize();

    // uint8_t protocol = ipv4Header.GetProtocol();
    // uint16_t payloadSize = ipv4Header.GetPayloadSize();

    // if (protocol == UdpL4Protocol::PROT_NUMBER && payloadSize >= 8)
    // {
      // UdpHeader udpHeader;
      // packet->RemoveHeader(udpHeader);

      // SeqTsHeader seqTsHeader;
      // packet->RemoveHeader(seqTsHeader);
      // set again the header
      // packet->AddHeader(ipv4Header);

      MmWaveMacPacketRelayTag packetRelayTag;
      bool hasTag = packet->PeekPacketTag(packetRelayTag);

      // std::stringstream sa, da;
      // sa << ipv4Header.GetSource();
      // da << ipv4Header.GetDestination();
      // sa << from;
      // da << to;
      // std::string sourceAddress = sa.str();
      // std::string destAddress = da.str();

      // uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
      // uint16_t _sourceRnti = packetRelayTag.GetSourceRnti();
      // uint16_t _intermediateRnti = packetRelayTag.GetIntermediateRnti();
      
      uint64_t packetId = packet->GetUid();

      // uint32_t seqNumber = seqTsHeader.GetSeq();
      double txTime = packetRelayTag.GetTs().GetSeconds();

      // std::cout << "Sequence number " << seqNumber << " txtime " << txTime << std::endl;

      stats->SaveRelayLatency (sourceRnti, destRnti, localRnti, // intermediateRnti, 
                            // sourceAddress, destAddress
                            // , pos, 
                            packetId, // seqNumber, 
                            txTime, packetSize,
                            traceSource
                            );

    // }
}

void
EfStatsHelper::SendPacketReportCallback (mmwave::SendPacketStats *stats, Ptr<Packet> p, uint16_t sourceRnti, 
                                        uint16_t destRnti, uint16_t intermediateRnti, uint16_t localRnti
                                        , Vector pos// ,const ns3::Address & from, const ns3::Address & to
                                        )
{
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);
    Ptr<Packet> packet = p->Copy();

    Ipv4Header ipv4Header;
    uint32_t ipv4Size = packet->RemoveHeader(ipv4Header);

    uint32_t packetSize = packet->GetSize();

    uint8_t protocol = ipv4Header.GetProtocol();
    uint16_t payloadSize = ipv4Header.GetPayloadSize();

    if (protocol == UdpL4Protocol::PROT_NUMBER && payloadSize >= 8)
    {
      UdpHeader udpHeader;
      packet->RemoveHeader(udpHeader);

      SeqTsHeader seqTsHeader;
      packet->RemoveHeader(seqTsHeader);
      // set again the header
      // packet->AddHeader(ipv4Header);

      MmWavePacketRelayTag packetRelayTag;
      bool hasTag = packet->PeekPacketTag(packetRelayTag);

      std::stringstream sa, da;
      sa << ipv4Header.GetSource();
      da << ipv4Header.GetDestination();
      // sa << from;
      // da << to;
      std::string sourceAddress = sa.str();
      std::string destAddress = da.str();

      uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
      uint16_t _sourceRnti = packetRelayTag.GetSourceRnti();
      uint16_t _intermediateRnti = packetRelayTag.GetIntermediateRnti();
      
      uint64_t packetId = packet->GetUid();

      uint32_t seqNumber = seqTsHeader.GetSeq();
      double txTime = seqTsHeader.GetTs().GetSeconds();

      // std::cout << "Sequence number " << seqNumber << " txtime " << txTime << std::endl;

      stats->SavePacketSend (sourceRnti, intermediateRnti, destRnti, localRnti,
                            sourceAddress, destAddress
                            , pos
                            , packetId, seqNumber, txTime, payloadSize
                            );

    }
}

void
EfStatsHelper::ReportMacBsr(mmwave::MacBsrStats *macBsrStats, mmwave::SfnSf sfnSf, uint16_t cellId, mmwave::MmWaveMacSchedSapProvider::SchedDlRlcBufferReqParameters schedParams){
	macBsrStats->SaveMacBsrStats(sfnSf, cellId, schedParams.m_rnti, schedParams.m_rlcTransmissionQueueSize, 
	schedParams.m_rlcTransmissionQueueHolDelay, schedParams.m_rlcStatusPduSize, schedParams.m_rlcRetransmissionQueueSize,
	schedParams.m_rlcRetransmissionHolDelay, schedParams.m_logicalChannelIdentity);
}

void
EfStatsHelper::ReportSchedulingInfo(mmwave::DlSchedulingStats *enbStats, uint16_t destRnti, millicar::SlSchedulingCallback schedParams){
  // std::cout << "Report " << std::endl;
  enbStats->SaveDlSchedulingStats(destRnti, schedParams.frame, schedParams.subframe, schedParams.slotNum,
        schedParams.txRnti, schedParams.symStart, schedParams.numSym,
        schedParams.mcs, schedParams.tbSize);
}

// void
// EfStatsHelper:: RelayPacketReportCallback (mmwave::SendPacketStats *stats, Ptr<Packet> p, uint16_t localRnti
//                                         , Vector pos// ,const ns3::Address & from, const ns3::Address & to
//                                         ){
//   // NS_LOG_UNCOND("RelayPacketReportCallback");
//     Ptr<Packet> packet = p->Copy();
//     // LteRlcHeader rlcHeader;
//     // packet->RemoveHeader(rlcHeader);
//     RlcTag rlcTag;
//     Time delayRlc;
//     if (packet->FindFirstMatchingByteTag(rlcTag))
//     {
//         delayRlc = Simulator::Now() - rlcTag.GetSenderTimestamp();
//     }
//     // Receiver timestamp
//     PdcpTag pdcpTag;
//     Time delay;
//     if (packet->FindFirstMatchingByteTag(pdcpTag))
//     {
//         delay = Simulator::Now() - pdcpTag.GetSenderTimestamp();
//     }
//     LtePdcpHeader pdcpHeader;
//     packet->RemoveHeader(pdcpHeader);
//     // NS_LOG_UNCOND("PDCP header: " << pdcpHeader);
//     // NS_LOG_UNCOND("Rlc tag: " << rlcTag.GetSenderTimestamp());
//     // NS_LOG_UNCOND("pdcpTag tag: " << pdcpTag.GetSenderTimestamp());
    
//     // Ipv4Header ipv4Header;
//     // uint32_t ipv4Size = packet->RemoveHeader(ipv4Header);

//     Ipv4Header ipv4Header;
//     uint32_t ipv4Size = packet->PeekHeader(ipv4Header);

//     uint8_t protocol = ipv4Header.GetProtocol();
//     uint16_t payloadSize = ipv4Header.GetPayloadSize();

//     // NS_LOG_UNCOND("relay report " << +protocol << " payload size "
//     //               << " ip header " << ipv4Header.GetDestination() << " size " << payloadSize);

//     // if (protocol == UdpL4Protocol::PROT_NUMBER && payloadSize >= 8)
//     // {
//       // UdpHeader udpHeader;
//       // packet->RemoveHeader(udpHeader);

//       SeqTsHeader seqTsHeader;
//       packet->RemoveHeader(seqTsHeader);
//       // set again the header
//       // packet->AddHeader(ipv4Header);

//       MmWavePacketRelayTag packetRelayTag;
//       bool hasTag = packet->PeekPacketTag(packetRelayTag);
//       uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
//       uint16_t sourceRnti = packetRelayTag.GetSourceRnti();
//       uint16_t intermediateRnti = packetRelayTag.GetIntermediateRnti();

//       NS_LOG_UNCOND("Dest " << destinationRnti << " source " << sourceRnti
//               << " inter " << intermediateRnti << " packet size " << p->GetSize());

//       std::stringstream sa, da;
//       sa << ipv4Header.GetSource();
//       da << ipv4Header.GetDestination();
//       // sa << from;
//       // da << to;
//       std::string sourceAddress = sa.str();
//       std::string destAddress = da.str();

//       // uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
//       // uint16_t sourceRnti = packetRelayTag.GetSourceRnti();
//       // uint16_t intermediateRnti = packetRelayTag.GetIntermediateRnti();
      
//       uint64_t packetId = packet->GetUid();
      
//       uint32_t seqNumber = seqTsHeader.GetSeq();
//       double txTime = seqTsHeader.GetTs().GetSeconds();

//       NS_LOG_UNCOND(" seq number " << seqNumber << " tx time "<<txTime);

//       // std::cout << "Sequence number " << seqNumber << " txtime " << txTime << std::endl;

//       stats->SavePacketRelay (sourceRnti, intermediateRnti, destinationRnti, localRnti,
//                             sourceAddress, destAddress
//                             , pos
//                             , packetId, pdcpHeader.GetSequenceNumber(), rlcTag.GetSenderTimestamp().GetSeconds(),
//                             packet->GetSize() 
//                             );
//     // }

    
// }

void
EfStatsHelper::RelayPacketReportCallback (mmwave::SendPacketStats *stats, Ptr<Packet> p, uint16_t localRnti
                                        , Vector pos// ,const ns3::Address & from, const ns3::Address & to
                                        ){
		// NS_LOG_UNCOND("saving sinr" << sourceCellId);

    Ptr<Packet> packet = p->Copy();
    
    Ipv4Header ipv4Header;
    uint32_t ipv4Size = packet->RemoveHeader(ipv4Header);

    uint8_t protocol = ipv4Header.GetProtocol();
    uint16_t payloadSize = ipv4Header.GetPayloadSize();

    if (protocol == UdpL4Protocol::PROT_NUMBER && payloadSize >= 8)
    {
      UdpHeader udpHeader;
      packet->RemoveHeader(udpHeader);

      SeqTsHeader seqTsHeader;
      packet->RemoveHeader(seqTsHeader);
      // set again the header
      // packet->AddHeader(ipv4Header);

      MmWavePacketRelayTag packetRelayTag;
      bool hasTag = packet->PeekPacketTag(packetRelayTag);

      std::stringstream sa, da;
      sa << ipv4Header.GetSource();
      da << ipv4Header.GetDestination();
      // sa << from;
      // da << to;
      std::string sourceAddress = sa.str();
      std::string destAddress = da.str();

      uint16_t destinationRnti = packetRelayTag.GetDestinationRnti();
      uint16_t sourceRnti = packetRelayTag.GetSourceRnti();
      uint16_t intermediateRnti = packetRelayTag.GetIntermediateRnti();
      
      uint64_t packetId = packet->GetUid();

      
      uint32_t seqNumber = seqTsHeader.GetSeq();
      double txTime = seqTsHeader.GetTs().GetSeconds();

      // std::cout << "Sequence number " << seqNumber << " txtime " << txTime << std::endl;

      stats->SavePacketRelay (sourceRnti, intermediateRnti, destinationRnti, localRnti,
                            sourceAddress, destAddress
                            , pos
                            , packetId, seqNumber, txTime,
                            payloadSize
                            );
    }

    
}




// }

}