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


#ifndef SRC_MMWAVE_SIDELINK_PHY_H_
#define SRC_MMWAVE_SIDELINK_PHY_H_

#include "mmwave-sidelink-spectrum-phy.h"
#include "mmwave-sidelink-sap.h"

namespace ns3 {

typedef std::pair<uint64_t, uint64_t> millicarPairDevices_t;

namespace millicar {

class MmWaveSidelinkPhy : public Object
{

public:

  /**
   * Dummy constructor, it is not used
   */
  MmWaveSidelinkPhy ();

  /**
   * MmWaveSidelinkPhy real constructor
   * \param channelPhy spectrum phy
   * \param confParams instance of mmwave::MmWavePhyMacCommon containing the
   *        configuration parameters
   *
   * Usually called by the helper. It starts the event loop for the device.
   */
  MmWaveSidelinkPhy (Ptr<MmWaveSidelinkSpectrumPhy> spectrumPhy, Ptr<mmwave::MmWavePhyMacCommon> confParams);

  /**
   * Desctructor
   */
  virtual ~MmWaveSidelinkPhy ();

  // inherited from Object
  static TypeId GetTypeId (void);
  virtual void DoInitialize (void);
  virtual void DoDispose (void);

  // modified
  void UpdateUePeerSinrEstimate();

  void PrintRelay();

  std::vector<double> MakeFilter(std::vector<double>,
                                   std::vector<double>,
                                   std::pair<uint64_t, uint64_t>);
  
  std::pair<uint64_t, uint64_t> ApplyFilter(std::vector<double>);

  double MakeAvg(std::vector<double>);

  double MakeVar(std::vector<double>, double);

  double AddGaussianNoise(double sample);

  /**
  * \brief assign a proper value to the RNTI associated to a specific user
  * \param rnti value of the rnti
  */
  void SetRnti (uint16_t rnti);

  /**
  * \brief return the RNTI associated to a specific user
  * \return the RNTI
  */
  uint16_t GetRnti () const;

  void ConfigureBeamformingByRnti(uint16_t rnti);
  // end modification

  /**
   * Set the tx power
   * \param the tx power in dBm
   */
  void SetTxPower (double power);

  /**
   * Returns the tx power
   * \return the tx power in dBm
   */
  double GetTxPower () const;

  /**
   * Set the noise figure
   * \param the noise figure in dB
   */
  void SetNoiseFigure (double pf);

  /**
   * Returns the noise figure
   * \return the noise figure in dB
   */
  double GetNoiseFigure () const;

  /**
   * Returns the mmwave::MmWavePhyMacCommon instance associated with this phy containing
   * the configuration parameters
   * \return the mmwave::MmWavePhyMacCommon instance
   */
  Ptr<mmwave::MmWavePhyMacCommon> GetConfigurationParameters (void) const;

  /**
   * Returns the SpectrumPhy instance associated with this phy
   * \return the SpectrumPhy instance
   */
  Ptr<MmWaveSidelinkSpectrumPhy> GetSpectrumPhy () const;

  /**
  * Get the PHY SAP provider
  * \return a pointer to the SAP provider to the MAC
  */
  MmWaveSidelinkPhySapProvider* GetPhySapProvider () const;

  /**
  * Set the PHY SAP user
  * \param sap the PHY SAP user
  */
  void SetPhySapUser (MmWaveSidelinkPhySapUser* sap);

  /**
   * Add a <rnti, device> pair to m_deviceMap. All the devices we want to
   * communicate with must be inserted in this map, otherwise we would not
   * be able to correctly configure the beamforming.
   * \param rnti the RNTI identifier
   * \param dev pointer to the NetDevice object
   */
  void AddDevice (uint64_t rnti, Ptr<NetDevice> dev);

  // modified

  void SetDevice(Ptr<NetDevice> d);
  Ptr<NetDevice> GetDevice();

  
  void SetLastCommuncatingDeviceRnti(uint16_t rnti);

  uint16_t GetLastCommuncatingDeviceRnti();

  void AddRelayPath(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti);

  void SetPropagationLossModel(Ptr<PropagationLossModel> model);

  // end modification

  void
  SetSpectrumPropagationLossModel(Ptr<PhasedArraySpectrumPropagationLossModel> propagationLossModel);


  /**
   * Add a transport block to the transmission buffer, which will be sent in the
   * current slot.
   * \param pb the packet burst containing the packets to be sent
   * \param info the mmwave::TtiAllocInfo instance containg the transmission information
   */
  void DoAddTransportBlock (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info);

  /**
   * Prepare for the reception from another device by properly configuring
   * the beamforming vector
   * \param rnti the RNTI of the transmitting device
   */
  void DoPrepareForReceptionFrom (uint16_t rnti);

  /**
  * Receive the packet from SpectrumPhy and forward it up to the MAC
  * \param p received packet
  */
  void Receive (Ptr<Packet> p);

  void SetMillicarSinrMapReportCallback(Callback<void, uint16_t, std::map<uint64_t, double>> cb);

  /**
  * \brief This method generates a new SINR report and sends it to the MAC layer.
           It is hooked to the callback MmWaveSidelinkSpectrumPhy::m_slSinrReportCallback
  * \param sinr pointer to the SpectrumValue instance representing the SINR
            measured on all the spectrum chunks
  * \param rnti the RNTI of the tranmitting device
  * \param numSym size of the transport block that generated the report in
            number of OFDM symbols
  * \param tbSize size of the transport block that generated the report in bytes
  * \param mcs the MCS of the transmission
  */
  void GenerateSinrReport (const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize, uint8_t mcs);

private:

  /**
   * Start a slot. Send all the transport blocks in the buffer.
   * \param timingInfo the structure containing the timing information
   */
  void StartSlot (mmwave::SfnSf timingInfo);

  

  /**
   * Transmit a transport block
   * \param pb the packet burst containing the packets to be sent
   * \param info the mmwave::TtiAllocInfo instance containg the transmission information
   * \return the number of symbols used to send this TB
   */
  uint8_t SlData (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info);

  /**
   * Set the transmission mask and creates the power spectral density for the
   * transmission
   * \return mask indicating the suchannels used for the transmission
   */
  std::vector<int> SetSubChannelsForTransmission ();

  /**
   * Send the packet burts
   * \param pb the packet burst
   * \param duration the duration of the transmissin
   * \param info the mmwave::TtiAllocInfo instance containg the transmission information
   * \param rbBitmap the mask indicating the suchannels to be used for the
            transmission
   */
  void SendDataChannels (Ptr<PacketBurst> pb, Time duration, mmwave::TtiAllocInfo info, std::vector<int> rbBitmap);

  /**
   * TODO: this can be done by overloading the operator ++ of the mmwave::SfnSf struct
   * Update the mmwave::SfnSf structure to point to the next slot. If the current slot
   * the last slot of the subframe, the next slot index will be 0 and the
   * subframe index will be incremented. If the current subframe is the last
   * subframe of the frame, the next subframe index will be 0 and the frame
   * frame index will be incremented.
   * \param info the mmwave::SfnSf structure containg frame, subframe and slot indeces
   * \return the updated SnfSn structure pointing to the next slot
   */
  mmwave::SfnSf UpdateTimingInfo (mmwave::SfnSf info) const;

  // report of the estimated sinr between millicar device pairs
  void ReportSinrMillicarPairs();

  // trace reporting the sinr received
  // local rnti, dest rnti, num sym, tbsize, avg sinr, position (x, y)
  TracedCallback<uint16_t, uint16_t, uint8_t, uint32_t, double> m_slSinrReportTrace; // , double, double

  // device rnti, peer rnti, sinr
  TracedCallback<uint16_t, uint64_t, double> m_notifyMillicarPairsSinrTrace;

  MmWaveSidelinkPhySapUser* m_phySapUser; //!< Sidelink PHY SAP user
  MmWaveSidelinkPhySapProvider* m_phySapProvider; //!< Sidelink PHY SAP provider
  double m_txPower; //!< the transmission power in dBm
  double m_noiseFigure; //!< the noise figure in dB
  Ptr<MmWaveSidelinkSpectrumPhy> m_sidelinkSpectrumPhy; //!< the SpectrumPhy instance associated with this PHY
  Ptr<mmwave::MmWavePhyMacCommon> m_phyMacConfig; //!< the configuration parameters
  typedef std::pair<Ptr<PacketBurst>, mmwave::TtiAllocInfo> PhyBufferEntry; //!< type of the phy buffer entries
  std::list<PhyBufferEntry> m_phyBuffer; //!< buffer of transport blocks to send in the current slot
  std::map<uint64_t, Ptr<NetDevice>> m_deviceMap; //!< map containing the <rnti, device> pairs of the nodes we want to communicate with


  std::map<uint64_t, double> m_sinrMillicarPairDevicesMap;

  Callback<void, uint16_t, std::map<uint64_t, double>> m_sinrMapDeviceForwardCallback;
  
  std::map<uint64_t, Ptr<SpectrumValue>> m_rxPsdMillicarPairMap;

  Ptr<NetDevice> m_netDevice;

  // modified
  // hack to allow eNB to compute the SINR, periodically, without pilots
    Ptr<SpectrumPropagationLossModel> m_spectrumPropagationLossModel;
    Ptr<PropagationLossModel> m_propagationLoss;
  Ptr<PhasedArraySpectrumPropagationLossModel> m_phasedArraySpectrumPropagationLossModel;
   bool m_noiseAndFilter; // If true, use noisy SINR samples, filtered. If false, just use the SINR
                           // measure
  
  std::map<millicarPairDevices_t, std::vector<double>>
      m_sinrVector; // array containing all SINR values for a specific pair (UE-eNB)
  std::map<millicarPairDevices_t, std::vector<double>>
      m_sinrVectorToFilter; // array containing the  SINR values that must be filtered
  std::map<millicarPairDevices_t, std::vector<double>>
      m_sinrVectorNoisy; // array containing the  noisy SINR values that must be filteredF
  std::map<millicarPairDevices_t, std::vector<double>>
      m_finalSinrVector; // array containing all  SINR values after the filtering for a specific
                          // pair (UE-eNB)
  std::map<millicarPairDevices_t, std::pair<uint64_t, uint64_t>>
        m_samplesFilter; // array containing all noisy SINR values for a specific pair (UE-eNB)
  
  int m_updateSinrPeriod;               // the period of SINR update for eNBs
  double m_ueUpdateSinrPeriod;          // the period of SINR reporting to the UEs
  double m_updateSinrCollect;           // the period of SINR collection, for a pair (UE-eNB)
  uint16_t m_roundFromLastUeSinrUpdate; // the ratio between the two above

  double m_transient;                   // after m_transient, we can start apply the filter
  
  uint16_t m_rnti; //!< radio network temporary identifier

  uint16_t m_lastCommuncatingDeviceRnti{UINT16_MAX};

  
  // end modification

  // std::map<uint64_t, Ptr<SpectrumValue>> m_rxPsdMap;

  // map to be used when relay functionality is implemented
  std::map<uint16_t, std::map<uint16_t, uint16_t>> m_relayPaths;
};

class MacSidelinkMemberPhySapProvider : public MmWaveSidelinkPhySapProvider
{

public:
  MacSidelinkMemberPhySapProvider (Ptr<MmWaveSidelinkPhy> phy);

  void AddTransportBlock (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info) override;

  void PrepareForReception (uint16_t rnti) override;

  Ptr<MmWaveSidelinkSpectrumPhy> GetSpectrum () const override;

private:
  Ptr<MmWaveSidelinkPhy> m_phy;

};

} // namespace millicar
} // namespace ns3

#endif /* SRC_MMWAVE_SIDELINK_PHY_H_ */
