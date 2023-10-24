/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
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
 *        Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                               Integration of Carrier Aggregation
 */

#ifndef SRC_MMWAVE_MODEL_MMWAVE_MILLICAR_UE_NET_DEVICE_H_
#define SRC_MMWAVE_MODEL_MMWAVE_MILLICAR_UE_NET_DEVICE_H_

#include <ns3/mmwave-enb-net-device.h>
#include <ns3/mmwave-ue-net-device.h>
#include <ns3/mmwave-phy.h>
#include <ns3/mmwave-ue-mac.h>

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include <ns3/epc-ue-nas.h>
#include <ns3/lte-ue-rrc.h>
#include <map>
#include <vector>

// millicar

#include <ns3/net-device.h>
#include <ns3/mmwave-sidelink-phy.h>
#include <ns3/mmwave-sidelink-mac.h>
#include <ns3/lte-pdcp.h>
#include <ns3/lte-radio-bearer-info.h>
#include <ns3/epc-tft-classifier.h>
#include <ns3/uniform-planar-array.h>
#include <ns3/mmwave-vehicular-net-device.h>
#include <ns3/mmwave-vehicular-traces-helper.h>



namespace ns3
{

class Packet;
class PacketBurst;
class Node;
class LteUeComponentCarrierManager;
// class MmWaveSidelinkMac;
// class MmWaveSidelinkPhy;
class UniformPlanarArray;

// class MmWavePhy;

namespace mmwave
{
class MmWaveUePhy;
class MmWaveUeMac;
class MmWaveEnbNetDevice;


class MmWaveMillicarUeNetDevice : public MmWaveUeNetDevice
{
  public:
    static TypeId GetTypeId(void);

    MmWaveMillicarUeNetDevice(void);
    /**
   * \brief Class constructor
   * \param phy pointer to the PHY
   * \param mac pointer to the MAC
   *
   */
    MmWaveMillicarUeNetDevice (Ptr<millicar::MmWaveSidelinkPhy> phy, Ptr<millicar::MmWaveSidelinkMac> mac);

    virtual ~MmWaveMillicarUeNetDevice(void);
    virtual void DoDispose() override;

    // modified
    void RegisterToMillicarEnbTraces();

    void PrintRelay();
    // end modification

    uint32_t GetCsgId() const;
    void SetCsgId(uint32_t csgId);

    void UpdateConfig(void);

    virtual bool DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;

    /**
   * \brief Send a packet to the vehicular stack
   * \param address MAC address of the destination device
   * \param protocolNumber identifies if NetDevice is using IPv4 or IPv6
   */
    virtual bool Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;
    
    void MillicarSinrMapReport(uint16_t rnti, std::map<uint64_t, double> mapReport); 

    // modified
    // temporary
    uint16_t GetRnti() const;

    void SendE2Msg(E2AP_PDU *pdu);

    void BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params);

    Ptr<KpmIndicationHeader> BuildRicIndicationHeader (std::string plmId, std::string gnbId, uint16_t nrCellId);

    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCp(std::string plmId, uint16_t nrCellId);

    void ProcessE2Message(uint8_t* buffer, size_t buffSize);

    void TestRelay(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti);

    void DecentralizedAddRelayPath(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti);

    void DecentralizedRemoveRelayPath(uint16_t localRnti, uint16_t destRnti);

    // Ptr<MmWaveUePhy> GetPhy(void) const;

    // Ptr<MmWaveUePhy> GetPhy(uint8_t index) const;

    // Ptr<MmWaveUeMac> GetMac(void) const;

    // uint64_t GetImsi() const;

    // Ptr<EpcUeNas> GetNas(void) const;

    // Ptr<LteUeComponentCarrierManager> GetComponentCarrierManager(void) const;

    // Ptr<LteUeRrc> GetRrc() const;

    // void SetTargetEnb(Ptr<MmWaveEnbNetDevice> enb);

    // Ptr<MmWaveEnbNetDevice> GetTargetEnb(void);

    ////////////// millicar 

    std::vector<uint16_t> _debug_vecto = {34, 35, 37, 39};

    /**
   * \brief Returns pointer to MAC
   */
  virtual Ptr<millicar::MmWaveSidelinkMac> GetMacMillicar (void) const;

  /**
   * \brief Returns pointer to PHY
   */
  virtual Ptr<millicar::MmWaveSidelinkPhy> GetPhyMillicar (void) const;

  /**
   * \brief Send a packet to the vehicular stack
   * \param address MAC address of the destination device
   * \param protocolNumber identifies if NetDevice is using IPv4 or IPv6
   */
  virtual bool SendMillicar (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber);

  /**
   * \brief Set MTU associated to the NetDevice
   * \param mtu size of the Maximum Transmission Unit
   */
  virtual bool SetMtuMillicar (const uint16_t mtu);

  /**
   * \brief Returns MTU associated to the NetDevice
   */
  virtual uint16_t GetMtuMillicar (void) const;

  /**
   * \brief Returns valid LteRlc TypeId based on the parameter passed
   * \param rlcType string representing the selected RLC type
   * \return the type id of the proper RLC class
   */
  TypeId GetRlcTypeMillicar (std::string rlcType);

  /**
   * \brief Packet forwarding to the MAC layer by calling DoTransmitPdu
   * \param p packet to be used to fire the callbacks
   */
  void ReceiveMillicar (Ptr<Packet> p);

  void Receive (Ptr<Packet> p);

  void RegisterReceivedPacketMillicar (Ptr<Packet> p);

  /**
   * \brief a logical channel with instances of PDCP/RLC layers is created and associated to a specific receiving device
   * \param bearerId identifier of the tunnel between two devices
   * \param destRnti the rnti of the destination
   * \param dest IP destination address
  */
  void ActivateBearerMillicar (const uint8_t bearerId, const uint16_t destRnti, const Address& dest);
  
  /**
   * \brief Set UniformPlanarArray object 
   * \param antenna antenna to mount on the device 
   */
  void SetAntennaArrayMillicar (Ptr<UniformPlanarArray> antenna);
  
  /**
   * \brief Get a UniformPlanarArray object
   * \return the antenna mounted on the devide
   */
  Ptr<UniformPlanarArray> GetAntennaArrayMillicar () const;

  protected:
    // inherited from Object
    virtual void DoInitialize(void) override;

  private:
    MmWaveMillicarUeNetDevice(const MmWaveMillicarUeNetDevice&);

    static void RegisterRicControlMessageCallback(Ptr<MmWaveMillicarUeNetDevice> netDev, std::string context, uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti);

    void RegisterRicControlMessage(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti);

    std::map<uint16_t, std::map<uint16_t, uint16_t>> m_relayPaths;

    bool RelayMillicar (Ptr<Packet> packet, uint16_t destinationRnti);

    // modified
    void UpdateDecentralizedRelayPath(mmwave::SfnSf timingInfo, uint16_t rnti);

    void UpdateDecentralizedAllRelayPaths(mmwave::SfnSf timingInfo);
    // end modification


    // Ptr<MmWaveEnbNetDevice> m_targetEnb;

    // Ptr<LteUeRrc> m_rrc;
    // Ptr<EpcUeNas> m_nas;
    // Ptr<LteUeComponentCarrierManager> m_componentCarrierManager; ///< the component carrier manager

    // uint64_t m_imsi;

    // uint32_t m_csgId;

    /////// millicar
    EpcTftClassifier m_tftClassifierMillicar;
    std::string m_rlcTypeMillicar;
    Ptr<millicar::MmWaveSidelinkMac> m_macMillicar; //!< pointer to the MAC instance to be associated to the NetDevice
    Ptr<millicar::MmWaveSidelinkPhy> m_phyMillicar; //!< pointer to the PHY instance to be associated to the NetDevice
    std::map<uint8_t, Ptr<millicar::SidelinkRadioBearerInfo>> m_bearerToInfoMapMillicar; //!< map to store RLC and PDCP instances associated to a specific bearer ID
    Ptr<UniformPlanarArray> m_antennaMillicar; //!< antenna mounted on the device
    std::map<uint8_t,uint8_t> m_bid2lcidMillicar; //!< map from the BID to a unique LCID

    mutable uint16_t m_mtuMillicar; //!< MTU associated to the NetDevice

    // device rnti, peer rnti, sinr
    TracedCallback<uint16_t, uint64_t, long double> m_notifyMillicarSinrTrace;

    // packet, source, destination, intermediate node, position
    TracedCallback<Ptr<Packet>, uint16_t, uint16_t, uint16_t, uint16_t, Vector> m_sendPacketTrace;

    // TracedCallback<Ptr<Packet>, uint16_t, uint16_t, uint16_t, Vector> m_relayPacketTrace;
    TracedCallback<Ptr<Packet>, uint16_t, Vector> m_relayPacketTrace;

    Ptr<millicar::MmWaveVehicularTracesHelper> m_phyTraceHelperMillicar; //!< Ptr to an helper for the physical layer traces

    /**
   * Return the LCID associated to a certain bearer
   * \param bearerId the bearer
   * \return the logical channel ID
   */
    uint8_t BidToLcidMillicar(const uint8_t bearerId) const;

    std::string m_tracesPath;


};

class PdcpSpecificSidelinkPdcpSapUser : public LtePdcpSapUser
{
public:
  /**
   * Constructor
   *
   * \param netDevice the specific netDevice to be connected to the PDCP SAP
   */
   PdcpSpecificSidelinkPdcpSapUser (Ptr<MmWaveMillicarUeNetDevice> netDevice);

  // Interface implemented from LtePdcpSapUser
  virtual void ReceivePdcpSdu (ReceivePdcpSduParameters params);

  uint16_t GetRnti() const;

private:
  PdcpSpecificSidelinkPdcpSapUser ();
  Ptr<MmWaveMillicarUeNetDevice> m_netDevice; ///< NetDevice
};

} // namespace mmwave
} // namespace ns3
#endif /* SRC_MMWAVE_MODEL_MMWAVE_MILLICAR_UE_NET_DEVICE_H_ */
