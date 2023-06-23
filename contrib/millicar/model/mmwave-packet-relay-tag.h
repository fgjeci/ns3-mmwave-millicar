#ifndef MMWAVE_PACKET_RELAY_TAG_H
#define MMWAVE_PACKET_RELAY_TAG_H

#include <ns3/header.h>
#include <ns3/ipv4-header.h>
#include <ns3/ptr.h>
#include "ns3/packet.h"

namespace ns3
{

class Tag;

/**
 * \ingroup lte
 *
 * Implementation of the GPRS Tunnelling Protocol header according to
 * GTPv1-U Release 10 as per 3Gpp TS 29.281 document
 *
 */
class MmWavePacketRelayTag : public Tag
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId(void);
    virtual TypeId GetInstanceTypeId(void) const;

    MmWavePacketRelayTag();
    // virtual ~MmWavePacketRelayTag();


    // virtual uint32_t Deserialize(Buffer::Iterator start);
    // virtual uint32_t GetSerializedSize(void) const;
    // virtual void Serialize(Buffer::Iterator start) const;
    // virtual void Deserialize(Buffer::Iterator start);
    // virtual void Print(std::ostream& os) const;

    virtual void Serialize(TagBuffer i) const;
    virtual void Deserialize(TagBuffer i);
    virtual uint32_t GetSerializedSize() const;
    virtual void Print(std::ostream& os) const;

    /**
     * Get destination function
     */
    Address GetDestinationAddress() const;

    /**
     * Get protocol type number function
     */
    uint16_t GetProtocolTypeNumber() const;

    /**
     * @brief Get destination rnti
     * 
     */
    uint16_t GetDestinationRnti() const;

    /**
     * @brief Get destination rnti
     * 
     */
    uint16_t GetSourceRnti() const;

    /**
     * @brief Get destination rnti
     * 
     */
    uint16_t GetIntermediateRnti() const;
    
    /**
     * Set protocol type number function
     */
    void SetProtocolTypeNumber(uint16_t protocolTypeNumber);

    /**
     * Set address of the destination
     */
    void SetDestinationAddress(Address& destination);

    /**
     * @brief Set destination Rnti
     * 
     */
    void SetDestinationRnti(uint16_t destinationRnti);


    /**
     * @brief Set destination Rnti
     * 
     */
    void SetIntermediateRnti(uint16_t intermediateRnti);

    /**
     * @brief Set source Rnti
     * 
     */
    void SetSourceRnti(uint16_t sourceRnti);

    /**
     * Equality operator.
     *
     * \param b MmWavePacketRelayTag object to compare
     * \returns true of equal
     */
    bool operator==(const MmWavePacketRelayTag& b) const;

  private:
    
    /**
     * This field contains the ip address of the destination node
     */
    Address m_address;
    /**
     * This field indicates the protocol number, to distinguish between TCP & UDP
     */
    uint16_t m_protocolNumber {UINT16_MAX};

    /**
     * @brief Set destination rnti
     * 
     */

    uint16_t m_destinationRnti {UINT16_MAX};

    uint16_t m_sourceRnti {UINT16_MAX};
    
    uint16_t m_intermediateRnti {UINT16_MAX};
};

} // namespace ns3

#endif /* MMWAVE_PACKET_RELAY_TAG_H */