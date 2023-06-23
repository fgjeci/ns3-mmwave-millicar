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
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *                 Dual Connectivity and Handover functionalities
 *
 * Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                Integration of Carrier Aggregation
 */

#ifndef MMWAVE_VEHICULAR_5G_HELPER_H
#define MMWAVE_VEHICULAR_5G_HELPER_H

#include <ns3/mmwave-mac-trace.h>
#include <ns3/mmwave-phy-trace.h>

#include <ns3/boolean.h>
#include <ns3/config.h>
#include <ns3/core-network-stats-calculator.h>
#include <ns3/epc-enb-s1-sap.h>
#include <ns3/epc-helper.h>
#include <ns3/epc-ue-nas.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/lte-anr.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/lte-enb-net-device-oran.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-ffr-algorithm.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/lte-spectrum-value-helper.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/mc-stats-calculator.h>
#include <ns3/mc-ue-net-device.h>
#include <ns3/mmwave-bearer-stats-calculator.h>
#include <ns3/mmwave-bearer-stats-connector.h>
#include <ns3/mmwave-component-carrier-enb.h>
#include <ns3/mmwave-enb-net-device.h>
#include <ns3/mmwave-enb-phy.h>
#include <ns3/mmwave-phy-mac-common.h>
#include <ns3/mmwave-phy.h>
#include <ns3/mmwave-rrc-protocol-ideal.h>
#include <ns3/mmwave-spectrum-value-helper.h>
#include <ns3/mmwave-ue-net-device.h>
#include <ns3/mmwave-ue-phy.h>
#include <ns3/mobility-model.h>
#include <ns3/names.h>
#include <ns3/net-device-container.h>
#include <ns3/net-device.h>
#include <ns3/node-container.h>
#include <ns3/node.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/spectrum-phy.h>

// millicar part
#include <ns3/mmwave-vehicular-5g-net-device.h>
#include <ns3/spectrum-channel.h>
#include <ns3/mmwave-vehicular-traces-helper.h>
#include <ns3/object-factory.h>
#include <ns3/mmwave-enb-net-device.h>
// #include "ns3/mmwave-vehicular-traces-helper.h"

namespace ns3
{

class SpectrumChannel;
class SpectrumpropagationLossModel;
class PropagationLossModel;

namespace mmwave
{

/* ... */
class MmWaveUePhy;
class MmWaveEnbPhy;
class MmWaveSpectrumValueHelper;

// class MmWave3gppChannel;

class MmWaveMillicarHelper : public Object
{
  public:
    MmWaveMillicarHelper(void);
    virtual ~MmWaveMillicarHelper(void);

    static TypeId GetTypeId(void);
    virtual void DoDispose(void);

    NetDeviceContainer InstallUeDevice(NodeContainer c);
private:
    NetDeviceContainer InstallMcUeDevice(NodeContainer c);
public:
    NetDeviceContainer InstallInterRatHoCapableUeDevice(NodeContainer c);
    NetDeviceContainer InstallEnbDevice(NodeContainer c);
    NetDeviceContainer InstallLteEnbDevice(NodeContainer c);
    void SetChannelConditionModelType(std::string type);
    void SetPathlossModelType(std::string type);
    void SetChannelModelType(std::string type);
    void SetUePhasedArrayModelType(std::string type);
    void SetEnbPhasedArrayModelType(std::string type);

    void RegisterMillicarDevicesToEnb(NetDeviceContainer ueNetDevices, Ptr<MmWaveEnbNetDevice> enbNetDevice);

    /**
     * Set an attribute to the PhasedArrayModel for the UEs
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetUePhasedArrayModelAttribute(std::string name, const AttributeValue& value);

    /**
     * Set an attribute to the PhasedArrayModel for the eNBs
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetEnbPhasedArrayModelAttribute(std::string name, const AttributeValue& value);

    /**
     * Set an attribute to the SpectrumPropagationLossModels
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetChannelModelAttribute(std::string name, const AttributeValue& value);

    /**
     * Set an attribute to the MmWaveEnbNetDevice
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetMmWaveEnbNetDeviceAttribute(std::string name, const AttributeValue& value);

    /**
     * Set an attribute to the MmWaveUeNetDevice
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetMmWaveUeNetDeviceAttribute(std::string name, const AttributeValue& value);

    /**
     * Set an attribute to the McUeNetDevice
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetMcUeNetDeviceAttribute(std::string name, const AttributeValue& value);

    void SetLtePathlossModelType(std::string type);

    /**
     * Set the type of beamforming model to be used
     * \param type beamforming model type
     */
    void SetBeamformingModelType(std::string type);

    /**
     * Set an attribute to the MmWaveBeamformingModel
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetBeamformingModelAttribute(std::string name, const AttributeValue& value);

    /**
     * Set the type of beamforming codebook to be used by UEs
     * \param type beamforming codebook type
     */
    void SetUeBeamformingCodebookType(std::string type);

    /**
     * Set an attribute to the BeamformingCodebook for UEs
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetUeBeamformingCodebookAttribute(std::string name, const AttributeValue& value);

    /**
     * Set the type of beamforming codebook to be used by eNBs
     * \param type beamforming codebook type
     */
    void SetEnbBeamformingCodebookType(std::string type);

    /**
     * Set an attribute to the BeamformingCodebook for eNBs
     * \param name name of the attribute to set
     * \param value value to set
     */
    void SetEnbBeamformingCodebookAttribute(std::string name, const AttributeValue& value);

    /**
     * This method is used to set the MmWaveComponentCarrier map.
     * The structure will be used within InstallSingleEnbDevice,
     * InstallSingleUeNetDevice and InstallSingleMcUeDevice.
     *
     * \param ccMapParams the component carrier map
     */
    void SetCcPhyParams(std::map<uint8_t, MmWaveComponentCarrier> ccMapParams);

    /**
     * This method is used to get the MmWaveComponentCarrier map.
     *
     * \return ccmap the component carrier map
     */
    std::map<uint8_t, MmWaveComponentCarrier> GetCcPhyParams();

    /**
     * This method is used to set the ComponentCarrier map.
     * The structure will be used within InstallSingleLteEnbDevice,
     * and InstallSingleMcUeDevice.
     *
     * \param ccMapParams the component carrier map
     */
    void SetLteCcPhyParams(std::map<uint8_t, ComponentCarrier> ccMapParams);

    /**
     * Attach mmWave-only ueDevices to the closest enbDevice
     */
    void AttachToClosestEnb(NetDeviceContainer ueDevices, NetDeviceContainer enbDevices);
    /**
     * Attach MC ueDevices to the closest LTE enbDevice, register all MmWave eNBs to the MmWaveUePhy
     */
    void AttachToClosestEnb(NetDeviceContainer ueDevices,
                            NetDeviceContainer mmWaveEnbDevices,
                            NetDeviceContainer lteEnbDevices);

    /**
     * Attach to an eNB selecting which one with an index
     * \param ueDevice the ueNetDevice
     * \param enbDevices all the eNBs
     * \param index an index to select the eNB (cellId - 1)
     */
    void AttachToEnbWithIndex(Ptr<NetDevice> ueDevice,
                              NetDeviceContainer enbDevices,
                              uint32_t index);

    /**
     * Attach MC ueDevices to the closest MmWave eNB device, register all MmWave eNBs to the
     * MmWaveUePhy, store all cellId in each LteUeRrc layer. This must be used when attaching
     * InterRatHandover capable devices \param ueDevices the ueNetDevice \param mmWaveEnbDevices all
     * the mmwave eNBs \param lteEnbDevices all the lte eNBs
     */
    void AttachIrToClosestEnb(NetDeviceContainer ueDevices,
                              NetDeviceContainer mmWaveEnbDevices,
                              NetDeviceContainer lteEnbDevices);

    void EnableTraces();

    void SetSchedulerType(std::string type);
    std::string GetSchedulerType() const;

    void SetLteSchedulerType(std::string type);
    std::string GetLteSchedulerType() const;

    void ActivateDataRadioBearer(NetDeviceContainer ueDevices, EpsBearer bearer);
    void ActivateDataRadioBearer(Ptr<NetDevice> ueDevice, EpsBearer bearer);
    void SetEpcHelper(Ptr<EpcHelper> epcHelper);

    void SetHarqEnabled(bool harqEnabled);
    bool GetHarqEnabled();
    void SetSnrTest(bool snrTest);
    bool GetSnrTest();
    Ptr<PropagationLossModel> GetPathLossModel(uint8_t index);

    /**
     * Set the type of FFR algorithm to be used by LTE eNodeB devices.
     *
     * \param type type of FFR algorithm, must be a type name of any class
     *             inheriting from ns3::LteFfrAlgorithm, for example:
     *             "ns3::LteFrNoOpAlgorithm"
     *
     * Equivalent with setting the `FfrAlgorithm` attribute.
     */
    void SetLteFfrAlgorithmType(std::string type);

    /**
     *
     * \return the LTE FFR algorithm type
     */
    std::string GetLteFfrAlgorithmType() const;

    /**
     * Set the type of handover algorithm to be used by LTE eNodeB devices.
     *
     * \param type type of handover algorithm, must be a type name of any class
     *             inheriting from ns3::LteHandoverAlgorithm, for example:
     *             "ns3::NoOpHandoverAlgorithm"
     *
     * Equivalent with setting the `HandoverAlgorithm` attribute.
     */
    void SetLteHandoverAlgorithmType(std::string type);

    /**
     *
     * \return the LTE handover algorithm type
     */
    std::string GetLteHandoverAlgorithmType() const;

    void AddX2Interface(NodeContainer enbNodes);
    void AddX2Interface(NodeContainer lteEnbNodes, NodeContainer mmWaveEnbNodes);
    void AddX2Interface(Ptr<Node> enbNode1, Ptr<Node> enbNode2);

    /**
     * Set the type of carrier component algorithm to be used by gNodeB devices.
     *
     * \param type type of carrier component manager
     *
     */
    void SetEnbComponentCarrierManagerType(std::string type);

    /**
     *
     * \return the carrier gnb component carrier manager type
     */
    std::string GetEnbComponentCarrierManagerType() const;

    /**
     * Set the type of carrier component algorithm to be used by eNodeB devices.
     *
     * \param type type of carrier component manager
     *
     */
    void SetLteEnbComponentCarrierManagerType(std::string type);

    /**
     *
     * \return the carrier enb component carrier manager type
     */
    std::string GetLteEnbComponentCarrierManagerType() const;

    /**
     * Set the type of Component Carrier Manager to be used by Ue devices.
     *
     * \param type type of UE Component Carrier Manager
     *
     */
    void SetUeComponentCarrierManagerType(std::string type);

    /**
     *
     * \return the carrier ue component carrier manager type
     */
    std::string GetUeComponentCarrierManagerType() const;

    /**
     * Set the blockage attribute of each channel if MmWave3gppChannel is used.
     *
     * \param blockageMap (CC ID, blockage attribute value)
     */
    void SetBlockageMap(std::map<uint8_t, bool> blockageMap);

    void EnablePdcpTraces(void);
    void EnableMcTraces(void);
    void EnableRlcTraces(void);
    void EnableDlPhyTrace();
    void EnableUlPhyTrace();
    void EnableEnbSchedTrace();

    // millicar part

    /**
   * Install a MmWaveVehicularNetDevice on each node in the container
   * \param nodes the node container
   * \return a NetDeviceContainer containing the installed devices
   */
  NetDeviceContainer InstallMmWaveVehicularNetDevicesMillicar (NodeContainer nodes);

  /**
   * Set the configuration parameters
   * \param conf pointer to mmwave::MmWavePhyMacCommon
   */
  void SetConfigurationParametersMillicar (Ptr<mmwave::MmWavePhyMacCommon> conf);

  /**
   * Retrieve pointer to the object that lists all the configuration parameters
   * \return a pointer to a MmWavePhyMacCommon object
   */
  Ptr<mmwave::MmWavePhyMacCommon> GetConfigurationParametersMillicar () const;
  
  /**
   * Set the beamforming delay model type
   * \param pdm the type id of the beamforming model to use
   */
  void SetBeamformingModelTypeMillicar (std::string type);

  /**
   * Associate the devices in the container
   * \param devices the NetDeviceContainer with the devices
   */
  void PairDevicesMillicar (NetDeviceContainer devices);

  /**
   * Configure the numerology index
   * \param index numerology index, used to define PHY layer parameters
   */
  void SetNumerologyMillicar (uint8_t index);
                                   
  /**
   * Configure the type of channel model to be used
   * \param model string representing the channel model to be used
   */
  void SetChannelModelTypeMillicar (std::string model);

  /**
   * Configure the scheduling pattern for a specific group of devices
   * \param devices the NetDeviceContainer with the devices
   * \return a vector of integers representing the scheduling pattern
  */
  std::vector<uint16_t> CreateSchedulingPatternMillicar (NetDeviceContainer devices);

  /**
   * Identifies the supported scheduling pattern policies
   */
  enum SchedulingPatternOption_t {DEFAULT = 1,
                                   OPTIMIZED = 2};

  /**
  * Set the scheduling pattern option type
  * \param spo the enum representing the scheduling pattern policy to be adopted
  */
  void SetSchedulingPatternOptionTypeMillicar (SchedulingPatternOption_t spo);

  /**
  * Returns the adopted scheduling pattern policy
  * \return the adopted scheduling pattern policy
  */
  SchedulingPatternOption_t GetSchedulingPatternOptionTypeMillicar () const;

  // end millicar

  protected:
    virtual void DoInitialize();

  private:
    void MmWaveChannelModelInitialization();
    void LteChannelModelInitialization();

    Ptr<NetDevice> InstallSingleUeDevice(Ptr<Node> n, uint16_t millicarRnti);
    Ptr<NetDevice> InstallSingleMcUeDevice(Ptr<Node> n, uint16_t millicarRnti);
    Ptr<NetDevice> InstallSingleEnbDevice(Ptr<Node> n);
    Ptr<NetDevice> InstallSingleLteEnbDevice(Ptr<Node> n);
    Ptr<NetDevice> InstallSingleInterRatHoCapableUeDevice(Ptr<Node> n);

    void AttachToClosestEnb(Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices);
    void AttachMcToClosestEnb(Ptr<NetDevice> ueDevice,
                              NetDeviceContainer mmWaveEnbDevices,
                              NetDeviceContainer lteEnbDevices);
    void AttachIrToClosestEnb(Ptr<NetDevice> ueDevice,
                              NetDeviceContainer mmWaveEnbDevices,
                              NetDeviceContainer lteEnbDevices);

    // void EnableDlPhyTrace ();
    // void EnableUlPhyTrace ();
    void EnableEnbPacketCountTrace();
    void EnableUePacketCountTrace();
    void EnableTransportBlockTrace();
    // void EnableRlcTraces (void);
    Ptr<MmWaveBearerStatsCalculator> GetRlcStats(void);
    // void EnablePdcpTraces (void);
    Ptr<MmWaveBearerStatsCalculator> GetPdcpStats(void);
    // void EnableMcTraces (void);
    Ptr<McStatsCalculator> GetMcStats(void);

    std::map<uint8_t, Ptr<SpectrumChannel>> m_channel; // mmWave TDD channel
    Ptr<SpectrumChannel> m_downlinkChannel; /// The downlink LTE channel used in the simulation.
    Ptr<SpectrumChannel> m_uplinkChannel;   /// The uplink LTE channel used in the simulation.

    std::string m_channelConditionModelType; //!< the type of the channel condition model to be used
                                             //!< (empty string means no channel condition model)

    std::map<uint8_t, Ptr<Object>> m_pathlossModel;
    std::string m_pathlossModelType;
    Ptr<Object> m_downlinkPathlossModel; /// The path loss model used in the LTE downlink channel.
    Ptr<Object> m_uplinkPathlossModel;   /// The path loss model used in the LTE uplink channel.

    std::string
        m_spectrumPropagationLossModelType; //!< the type of the SpectrumPropagationLossModel to use
                                            //!< (if needed)

    ObjectFactory m_enbNetDeviceFactory;
    ObjectFactory m_lteEnbNetDeviceFactory;
    ObjectFactory m_ueNetDeviceFactory;
    ObjectFactory m_mcUeNetDeviceFactory;
    ObjectFactory m_channelFactory; // TODO check if one factory for the channel is enough
    ObjectFactory
        m_channelConditionModelFactory;   //!< the factory for the ChannelConditionModel objects
    ObjectFactory m_pathlossModelFactory; // Each channel (mmWave, LteUl & LteDl) may have a
                                          // different pathloss with diff attributes
    ObjectFactory m_spectrumPropagationLossModelFactory; //!< the factory for the
                                                         //!< SpectrumPropagationLossModel objects
    ObjectFactory m_schedulerFactory;
    ObjectFactory m_lteSchedulerFactory; // Factory for LTE scheduler
    ObjectFactory m_ffrAlgorithmFactory;
    ObjectFactory m_lteFfrAlgorithmFactory;
    ObjectFactory m_lteHandoverAlgorithmFactory;

    ObjectFactory m_lteChannelFactory; /// Factory of both the downlink and uplink LTE channels.
    ObjectFactory
        m_dlPathlossModelFactory; /// Factory of path loss model object for the downlink channel.
    ObjectFactory
        m_ulPathlossModelFactory; /// Factory of path loss model object for the uplink channel.

    ObjectFactory
        m_enbComponentCarrierManagerFactory; /// Factory of enb component carrier manager object.
    ObjectFactory
        m_ueComponentCarrierManagerFactory; /// Factory of ue component carrier manager object.

    ObjectFactory
        m_lteEnbComponentCarrierManagerFactory; /// Factory of enb component carrier manager object.

    uint64_t m_imsiCounter;
    uint16_t m_cellIdCounter;

    Ptr<MmWavePhyTrace> m_phyStats;
    Ptr<MmWaveMacTrace> m_enbStats;

    ObjectFactory m_lteUeAntennaModelFactory;  /// Factory of antenna object for Lte UE.
    ObjectFactory m_lteEnbAntennaModelFactory; /// Factory of antenna objects for Lte eNB.

    ObjectFactory m_uePhasedArrayModelFactory;  /// Factory of PhasedArrayModel objects for UEs
    ObjectFactory m_enbPhasedArrayModelFactory; /// Factory of PhasedArrayModel objects for eNBs

    ObjectFactory m_ueBeamformingCodebookFactory;  /// Factory of beamforming codebook for UEs
    ObjectFactory m_enbBeamformingCodebookFactory; /// Factory of beamforming codebooks for eNBs

    ObjectFactory m_bfModelFactory; //!< Factory for the beamforming model
    /**
     * From lte-helper.h
     * The `UsePdschForCqiGeneration` attribute. If true, DL-CQI will be
     * calculated from PDCCH as signal and PDSCH as interference. If false,
     * DL-CQI will be calculated from PDCCH as signal and PDCCH as interference.
     */
    bool m_usePdschForCqiGeneration;
    bool m_isAnrEnabled;

    Ptr<EpcHelper> m_epcHelper;

    bool m_harqEnabled;
    bool m_rlcAmEnabled;
    bool m_snrTest;
    bool m_useIdealRrc; // Initialized as true in the constructor

    Ptr<MmWaveBearerStatsCalculator> m_rlcStats;
    Ptr<MmWaveBearerStatsCalculator> m_pdcpStats;
    Ptr<McStatsCalculator> m_mcStats;
    Ptr<MmWaveBearerStatsConnector> m_radioBearerStatsConnector;
    Ptr<CoreNetworkStatsCalculator> m_cnStats;

    /**
     * The `LteUseCa` attribute. If true, Carrier Aggregation is enabled in the
     * LTE stack.
     */
    bool m_lteUseCa;

    /**
     * The `UseCa` attribute. If true, Carrier Aggregation is enabled in the MmWave
     * stack.
     */
    bool m_useCa;

    /**
     * This contains all the informations about each LTE component carrier
     */
    std::map<uint8_t, ComponentCarrier> m_lteComponentCarrierPhyParams;

    /**
     * This contains all the informations about each mmWave component carrier
     */
    std::map<uint8_t, MmWaveComponentCarrier> m_componentCarrierPhyParams;

    /**
     * Number of LTE component carriers that will be installed by default at LTE
     * eNodeB and MC-UE devices.
     */
    uint16_t m_noOfLteCcs;

    /**
     * Number of component carriers that will be installed by default at gNodeB
     * and UE devices.
     */
    uint16_t m_noOfCcs;
    /*
    * The `E2Mode` attribute. If true, enable E2 interface
    */
    bool m_e2mode_nr;
    bool m_e2mode_lte;
    std::string m_e2ip;
    uint16_t m_e2port;
    uint16_t m_e2localPort;

    // modified
    std::string m_tracesPath;

    std::string m_ltePlmnId;
    // end modification


    //////////////////////////////// millicar part 

    /**
   * Install a MmWaveVehicularNetDevice on the node
   * \param n the node
   * \param rnti the RNTI
   * \return pointer to the installed NetDevice
   */
    Ptr<MmWaveMillicarUeNetDevice> InstallSingleMmWaveVehicularNetDeviceMillicar (Ptr<Node> n, uint16_t rnti);
    
    /**
     * Create and configure the spectrum channel
     * \param model string representing the type of channel model to be created
     * \return pointer to the SpectrumChannel object
     */
    Ptr<SpectrumChannel> CreateSpectrumChannelMillicar (std::string model) const;

    Ptr<SpectrumChannel> m_channelMillicar; //!< the SpectrumChannel
    Ptr<mmwave::MmWavePhyMacCommon> m_phyMacConfigMillicar; //!< the configuration parameters
    uint16_t m_rntiCounterMillicar; //!< a counter to set the RNTIs
    uint8_t m_numerologyIndexMillicar; //!< numerology index
    double m_bandwidthMillicar; //!< system bandwidth
    std::string m_channelModelTypeMillicar; //!< the type of channel model to be used
    SchedulingPatternOption_t m_schedulingOptMillicar; //!< the type of scheduling pattern policy to be adopted
    
    ObjectFactory m_bfModelFactoryMillicar; //!< beamforming model object factory
    Ptr<millicar::MmWaveVehicularTracesHelper> m_phyTraceHelperMillicar; //!< Ptr to an helper for the physical layer traces
};

} // namespace mmwave

} // namespace ns3

#endif /* MMWAVE_HELPER_H */
