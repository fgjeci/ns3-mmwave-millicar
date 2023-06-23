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

#include "mmwave-packet-relay-header.h"

#include "ns3/log.h"
#include "ns3/tag.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWavePacketRelayHeader");

/********************************************************
 *        GTP-U-v1 Header
 ********************************************************/

NS_OBJECT_ENSURE_REGISTERED(MmWavePacketRelayHeader);

TypeId
MmWavePacketRelayHeader::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MmWavePacketRelayHeader")
                            .SetParent<Header>()
                            // .SetGroupName("Lte")
                            .AddConstructor<MmWavePacketRelayHeader>();
    return tid;
}

MmWavePacketRelayHeader::MmWavePacketRelayHeader()
{
}

MmWavePacketRelayHeader::~MmWavePacketRelayHeader()
{
}

TypeId
MmWavePacketRelayHeader::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
MmWavePacketRelayHeader::GetSerializedSize(void) const
{
    // return 2 + 4 + m_address.GetSerializedSize();
    return 2;
}

void
MmWavePacketRelayHeader::Serialize(Buffer::Iterator start) const
// MmWavePacketRelayHeader::Serialize(TagBuffer i) const
{
    Buffer::Iterator i = start;
    i.WriteHtonU16(m_destinationRnti);
    // i.WriteU16(m_protocolNumber);
    // i.WriteU32(m_address.GetSerializedSize());
    // uint8_t buff [m_address.GetSerializedSize()] ;
    // m_address.CopyAllTo(&buff[0], m_address.GetSerializedSize());
    // i.Write(buff, m_address.GetSerializedSize());
}

uint32_t
MmWavePacketRelayHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
//     // m_protocolNumber = i.ReadU16();
//     // uint32_t addressSize = i.ReadU32();
//     // uint8_t buff [addressSize] ;
//     // start.Read(&buff[0], addressSize);
//     // m_address.CopyAllFrom(&buff[0], addressSize);
    m_destinationRnti = i.ReadNtohU16();
    return GetSerializedSize();
}

// void
// MmWavePacketRelayHeader::Deserialize(Buffer::Iterator start)
// // MmWavePacketRelayHeader::Deserialize(TagBuffer i)
// {
//     Buffer::Iterator i = start;
//     // m_destinationRnti = i.ReadU16();
//     // return GetSerializedSize();
// }

void
MmWavePacketRelayHeader::Print(std::ostream& os) const
{
    os << "Destination Rnti " << m_destinationRnti;
    // os << " Protocol Typy number =" << (uint32_t)m_protocolNumber;
    // os << ", address=" << m_address;
}

uint16_t
MmWavePacketRelayHeader::GetProtocolTypeNumber() const
{
    return m_protocolNumber;
}


Address
MmWavePacketRelayHeader::GetDestinationAddress() const
{
    return m_address;
}

uint16_t
MmWavePacketRelayHeader::GetDestinationRnti() const
{
    return m_destinationRnti;
}


void
MmWavePacketRelayHeader::SetDestinationAddress(Address& destinationAddress)
{
    this->m_address = destinationAddress;
}

void
MmWavePacketRelayHeader::SetDestinationRnti(uint16_t destinationRnti)
{
    this->m_destinationRnti = destinationRnti;
}

void
MmWavePacketRelayHeader::SetProtocolTypeNumber(uint16_t protocolTypeNumber)
{
    this->m_protocolNumber = protocolTypeNumber;
}


bool
MmWavePacketRelayHeader::operator==(const MmWavePacketRelayHeader& b) const
{
    if (m_address == b.m_address && m_protocolNumber == b.m_protocolNumber && m_destinationRnti == b.m_destinationRnti)
    {
        return true;
    }
    return false;
}

} // namespace ns3
