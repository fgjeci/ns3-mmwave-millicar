/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Jaume Nin <jnin@cttc.cat>
 */

#include "mmwave-packet-relay-tag.h"

#include "ns3/log.h"
#include "ns3/tag.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWavePacketRelayTag");

/********************************************************
 *        GTP-U-v1 Header
 ********************************************************/

NS_OBJECT_ENSURE_REGISTERED(MmWavePacketRelayTag);

TypeId
MmWavePacketRelayTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MmWavePacketRelayTag")
                            .SetParent<Tag>()
                            // .SetGroupName("Lte")
                            .AddConstructor<MmWavePacketRelayTag>();
    return tid;
}

MmWavePacketRelayTag::MmWavePacketRelayTag()
{
}

// MmWavePacketRelayTag::~MmWavePacketRelayTag()
// {
// }

TypeId
MmWavePacketRelayTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
MmWavePacketRelayTag::GetSerializedSize(void) const
{
    // return 2 + 4 + m_address.GetSerializedSize();
    return 6;
}

void
// MmWavePacketRelayTag::Serialize(Buffer::Iterator start) const
MmWavePacketRelayTag::Serialize(TagBuffer i) const
{
    // Buffer::Iterator i = start;
    i.WriteU16(m_destinationRnti);
    i.WriteU16(m_sourceRnti);
    i.WriteU16(m_intermediateRnti);
    // i.WriteU16(m_protocolNumber);
    // i.WriteU32(m_address.GetSerializedSize());
    // uint8_t buff [m_address.GetSerializedSize()] ;
    // m_address.CopyAllTo(&buff[0], m_address.GetSerializedSize());
    // i.Write(buff, m_address.GetSerializedSize());
}

// uint32_t
// MmWavePacketRelayTag::Deserialize(Buffer::Iterator start)
// {
//     Buffer::Iterator i = start;
//     // m_protocolNumber = i.ReadU16();
//     // uint32_t addressSize = i.ReadU32();
//     // uint8_t buff [addressSize] ;
//     // start.Read(&buff[0], addressSize);
//     // m_address.CopyAllFrom(&buff[0], addressSize);
//     m_destinationRnti = i.ReadU16();
//     return GetSerializedSize();
// }

void
// MmWavePacketRelayTag::Deserialize(Buffer::Iterator start)
MmWavePacketRelayTag::Deserialize(TagBuffer i)
{
    // Buffer::Iterator i = start;
    m_destinationRnti = (uint16_t)i.ReadU16();
    m_sourceRnti = (uint16_t)i.ReadU16();
    m_intermediateRnti = (uint16_t)i.ReadU16();
    // return GetSerializedSize();
}

void
MmWavePacketRelayTag::Print(std::ostream& os) const
{
    os << "Destination Rnti " << m_destinationRnti;
    // os << " Protocol Typy number =" << (uint32_t)m_protocolNumber;
    // os << ", address=" << m_address;
}

uint16_t
MmWavePacketRelayTag::GetProtocolTypeNumber() const
{
    return m_protocolNumber;
}


Address
MmWavePacketRelayTag::GetDestinationAddress() const
{
    return m_address;
}

uint16_t
MmWavePacketRelayTag::GetSourceRnti() const
{
    return  m_sourceRnti;
}

uint16_t
MmWavePacketRelayTag::GetDestinationRnti() const
{
    return m_destinationRnti;
}

uint16_t
MmWavePacketRelayTag::GetIntermediateRnti() const
{
    return m_intermediateRnti;
}


void
MmWavePacketRelayTag::SetDestinationAddress(Address& destinationAddress)
{
    m_address = destinationAddress;
}

void
MmWavePacketRelayTag::SetSourceRnti(uint16_t sourceRnti)
{
    m_sourceRnti = sourceRnti;
}

void
MmWavePacketRelayTag::SetDestinationRnti(uint16_t destinationRnti)
{
    m_destinationRnti = destinationRnti;
}

void
MmWavePacketRelayTag::SetIntermediateRnti(uint16_t intermediateRnti)
{
    m_intermediateRnti = intermediateRnti;
}

void
MmWavePacketRelayTag::SetProtocolTypeNumber(uint16_t protocolTypeNumber)
{
    m_protocolNumber = protocolTypeNumber;
}


bool
MmWavePacketRelayTag::operator==(const MmWavePacketRelayTag& b) const
{
    if (m_address == b.m_address && m_protocolNumber == b.m_protocolNumber && m_destinationRnti == b.m_destinationRnti)
    {
        return true;
    }
    return false;
}

//////////////////////////// The Mac relay


TypeId
MmWaveMacPacketRelayTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MmWaveMacPacketRelayTag")
                            .SetParent<Tag>()
                            .AddConstructor<MmWaveMacPacketRelayTag>();
    return tid;
}

MmWaveMacPacketRelayTag::MmWaveMacPacketRelayTag():
    m_ts(Simulator::Now().GetTimeStep())
{
}

TypeId
MmWaveMacPacketRelayTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
MmWaveMacPacketRelayTag::GetSerializedSize(void) const
{
    // return 2 + 4 + m_address.GetSerializedSize();
    return 6 + 8;
}

void
MmWaveMacPacketRelayTag::Serialize(TagBuffer i) const
{
    // Buffer::Iterator i = start;
    i.WriteU16(m_destinationRnti);
    i.WriteU16(m_sourceRnti);
    i.WriteU16(m_intermediateRnti);
    i.WriteU64(m_ts);
}

void
MmWaveMacPacketRelayTag::Deserialize(TagBuffer i)
{
    m_destinationRnti = (uint16_t)i.ReadU16();
    m_sourceRnti = (uint16_t)i.ReadU16();
    m_intermediateRnti = (uint16_t)i.ReadU16();
    m_ts = (uint64_t)i.ReadU64();
}

uint16_t
MmWaveMacPacketRelayTag::GetDestinationRnti() const
{
    return m_destinationRnti;
}

uint16_t
MmWaveMacPacketRelayTag::GetSourceRnti() const
{
    return  m_sourceRnti;
}

uint16_t
MmWaveMacPacketRelayTag::GetIntermediateRnti() const
{
    return m_intermediateRnti;
}

void
MmWaveMacPacketRelayTag::SetDestinationRnti(uint16_t destinationRnti)
{
    m_destinationRnti = destinationRnti;
}

void
MmWaveMacPacketRelayTag::SetSourceRnti(uint16_t sourceRnti)
{
    m_sourceRnti = sourceRnti;
}

void
MmWaveMacPacketRelayTag::SetIntermediateRnti(uint16_t intermediateRnti)
{
    m_intermediateRnti = intermediateRnti;
}

Time
MmWaveMacPacketRelayTag::GetTs() const
{
    NS_LOG_FUNCTION(this);
    return TimeStep(m_ts);
}

void
MmWaveMacPacketRelayTag::Print(std::ostream& os) const
{
    os << "Destination Rnti " << m_destinationRnti;
    // os << " Protocol Typy number =" << (uint32_t)m_protocolNumber;
    // os << ", address=" << m_address;
}

} // namespace ns3
