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

#define NS_LOG_APPEND_CONTEXT                                            \
do                                                                     \
  {                                                                    \
    std::clog << Simulator::Now() << " [ Rnti " << GetRnti() << "] ";  \
  }                                                                    \
while (false);


#include "mmwave-sidelink-phy.h"
#include <ns3/mmwave-spectrum-value-helper.h>
#include <ns3/mmwave-mac-pdu-tag.h>
#include <ns3/mmwave-mac-pdu-header.h>
#include <ns3/double.h>
#include <ns3/pointer.h>
#include <ns3/mmwave-vehicular-5g-net-device.h>
#include <ns3/node.h>

namespace ns3 {

namespace millicar {

MacSidelinkMemberPhySapProvider::MacSidelinkMemberPhySapProvider (Ptr<MmWaveSidelinkPhy> phy)
  : m_phy (phy)
{

}

void
MacSidelinkMemberPhySapProvider::AddTransportBlock (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info)
{
  m_phy->DoAddTransportBlock (pb, info);
}

void
MacSidelinkMemberPhySapProvider::PrepareForReception (uint16_t rnti)
{
  m_phy->DoPrepareForReceptionFrom (rnti);
}

Ptr<MmWaveSidelinkSpectrumPhy>
MacSidelinkMemberPhySapProvider::GetSpectrum () const
{
  return m_phy->GetSpectrumPhy ();
}




//-----------------------------------------------------------------------

NS_LOG_COMPONENT_DEFINE ("MmWaveSidelinkPhy");

NS_OBJECT_ENSURE_REGISTERED (MmWaveSidelinkPhy);

MmWaveSidelinkPhy::MmWaveSidelinkPhy ()
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("This constructor should not be called");
  m_roundFromLastUeSinrUpdate = 0;
}

MmWaveSidelinkPhy::MmWaveSidelinkPhy (Ptr<MmWaveSidelinkSpectrumPhy> spectrumPhy, Ptr<mmwave::MmWavePhyMacCommon> confParams)
{
  NS_LOG_FUNCTION (this);
  m_sidelinkSpectrumPhy = spectrumPhy;
  m_phyMacConfig = confParams;

  // create the PHY SAP provider
  m_phySapProvider = new MacSidelinkMemberPhySapProvider (this);

  // create the noise PSD
  Ptr<SpectrumValue> noisePsd = mmwave::MmWaveSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_phyMacConfig, m_noiseFigure);
  m_sidelinkSpectrumPhy->SetNoisePowerSpectralDensity (noisePsd);

  // schedule the first slot
  Simulator::ScheduleNow (&MmWaveSidelinkPhy::StartSlot, this, mmwave::SfnSf (0, 0, 0));
}

MmWaveSidelinkPhy::~MmWaveSidelinkPhy ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MmWaveSidelinkPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MmWaveSidelinkPhy")
    .SetParent<Object> ()
    .AddConstructor<MmWaveSidelinkPhy> ()
    .AddAttribute ("TxPower",
                   "Transmission power in dBm",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&MmWaveSidelinkPhy::SetTxPower,
                                       &MmWaveSidelinkPhy::GetTxPower),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NoiseFigure",
                    "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                    " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                    "\"the difference in decibels (dB) between"
                    " the noise output of the actual receiver to the noise output of an "
                    " ideal receiver with the same overall gain and bandwidth when the receivers "
                    " are connected to sources at the standard noise temperature T0.\" "
                   "In this model, we consider T0 = 290K.",
                    DoubleValue (5.0),
                    MakeDoubleAccessor (&MmWaveSidelinkPhy::SetNoiseFigure,
                                        &MmWaveSidelinkPhy::GetNoiseFigure),
                    MakeDoubleChecker<double> ())
    // modified
    .AddAttribute(
                "NoiseAndFilter",
                "If true, use noisy SINR samples, filtered. If false, just use the SINR measure",
                BooleanValue(false),
                MakeBooleanAccessor(&MmWaveSidelinkPhy::m_noiseAndFilter),
                MakeBooleanChecker())
    .AddAttribute("UpdateSinrEstimatePeriod",
                          "Period (in microseconds) of update of SINR estimate of all the UE",
                          IntegerValue(1600), // TODO considering refactoring in MmWavePhyMacCommon
                          MakeIntegerAccessor(&MmWaveSidelinkPhy::m_updateSinrPeriod),
                          MakeIntegerChecker<int>())
    .AddAttribute("UpdateUeSinrEstimatePeriod",
                  "Period (in ms) of reporting of SINR estimate of all the UE",
                  DoubleValue(25.6),
                  MakeDoubleAccessor(&MmWaveSidelinkPhy::m_ueUpdateSinrPeriod),
                  MakeDoubleChecker<double>())
    .AddAttribute("Transient",
                  "Transient period (in microseconds) in which just collect SINR values "
                  "without filtering the sample",
                  IntegerValue(320000),
                  MakeIntegerAccessor(&MmWaveSidelinkPhy::m_transient),
                  MakeIntegerChecker<int>())
    .AddTraceSource("SlSinrReport",
                    "Report of sinr received from communicating",
                    MakeTraceSourceAccessor (&MmWaveSidelinkPhy::m_slSinrReportTrace),
                     "ns3::millicar::MmWaveSidelinkPhy::SlSinrReportTracedCallback"
                    ) 
    .AddTraceSource("NotifyMillicarPairsSinr",
                    "trace fired when measurement report is received from millicar device, "
                    "for each peer-ue pair",
                    MakeTraceSourceAccessor(&MmWaveSidelinkPhy::m_notifyMillicarPairsSinrTrace),
                    "ns3::millicar::MmWaveSidelinkPhy::NotifyMillicarPairsSinrTracedCallback")
    // end modification                
    ;
  return tid;
}

void
MmWaveSidelinkPhy::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG("Activating the peer sinr estimate");
  Simulator::Schedule(Seconds(0), &MmWaveSidelinkPhy::UpdateUePeerSinrEstimate, this);
}

void
MmWaveSidelinkPhy::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  delete m_phySapProvider;
}

void
MmWaveSidelinkPhy::SetTxPower (double power)
{
  m_txPower = power;
}
double
MmWaveSidelinkPhy::GetTxPower () const
{
  return m_txPower;
}

void
MmWaveSidelinkPhy::SetNoiseFigure (double nf)
{
  m_noiseFigure = nf;

  // update the noise PSD
  Ptr<SpectrumValue> noisePsd = mmwave::MmWaveSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_phyMacConfig, m_noiseFigure);
  m_sidelinkSpectrumPhy->SetNoisePowerSpectralDensity (noisePsd);
}

double
MmWaveSidelinkPhy::GetNoiseFigure () const
{
  return m_noiseFigure;
}

Ptr<MmWaveSidelinkSpectrumPhy>
MmWaveSidelinkPhy::GetSpectrumPhy () const
{
  return m_sidelinkSpectrumPhy;
}

Ptr<mmwave::MmWavePhyMacCommon>
MmWaveSidelinkPhy::GetConfigurationParameters (void) const
{
  return m_phyMacConfig;
}

MmWaveSidelinkPhySapProvider*
MmWaveSidelinkPhy::GetPhySapProvider () const
{
  NS_LOG_FUNCTION (this);
  return m_phySapProvider;
}

void
MmWaveSidelinkPhy::SetPhySapUser (MmWaveSidelinkPhySapUser* sap)
{
  NS_LOG_FUNCTION (this);
  m_phySapUser = sap;
}

void
MmWaveSidelinkPhy::SetRnti (uint16_t rnti)
{
  m_rnti = rnti;
}

uint16_t
MmWaveSidelinkPhy::GetRnti () const
{
  return m_rnti;
}

void
MmWaveSidelinkPhy::DoAddTransportBlock (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info)
{
  // create a new entry for the PHY buffer
  PhyBufferEntry e = std::make_pair (pb, info);

  // add the new entry to the buffer
  m_phyBuffer.push_back (e);
}

// modified

// change the rnti of all the packet when there is an intermediate
void 
MmWaveSidelinkPhy::AddRelayPath(uint16_t localRnti, uint16_t destRnti, uint16_t intermediateRnti){
  // NS_LOG_FUNCTION(this << localRnti << destRnti<<intermediateRnti);
  NS_LOG_LOGIC("Adding relay path from rnti " << localRnti << " to " << destRnti << " through " << intermediateRnti);
  m_relayPaths[localRnti][destRnti] = intermediateRnti;
  m_relayPaths[destRnti][localRnti] = intermediateRnti;
}

double
MmWaveSidelinkPhy::MakeAvg(std::vector<double> v)
{
    double return_value = 0.0;
    int n = v.size();

    for (int i = 0; i < n; i++)
    {
        return_value += v.at(i);
    }

    return (return_value / v.size());
}

//****************End of average funtion****************

// Function for variance
double
MmWaveSidelinkPhy::MakeVar(std::vector<double> v, double mean)
{
    double sum = 0.0;
    double temp = 0.0;
    double var = 0.0;
    int n = v.size();

    for (int j = 0; j < n; j++)
    {
        temp = std::pow((v.at(j) - mean), 2);
        sum += temp;
    }

    return var = sum / (v.size());
}

//****************End of variance funtion****************

double
MmWaveSidelinkPhy::AddGaussianNoise(double LastSinrValue)
{
    double N0 = 3.98107170e-12;
    std::complex<double> gaussianNoise;
    double signalEnergy;
    double noisySample;

    Ptr<NormalRandomVariable> randomVariable = CreateObject<NormalRandomVariable>();
    double gaussianSampleRe = randomVariable->GetValue();
    double gaussianSampleIm = randomVariable->GetValue();
    gaussianNoise = std::complex<double>(sqrt(0.5) * sqrt(N0) * gaussianSampleRe,
                                         sqrt(0.5) * sqrt(N0) * gaussianSampleIm);

    signalEnergy = LastSinrValue * N0;

    noisySample = (std::pow(std::abs(sqrt(signalEnergy) + gaussianNoise), 2) - N0) / N0;

    return noisySample;
}


std::pair<uint64_t, uint64_t>
MmWaveSidelinkPhy::ApplyFilter(std::vector<double> noisySinr)
{
    std::vector<double> noisySinrdB;
    for (uint64_t i = 0; i < noisySinr.size(); ++i)
    {
        noisySinrdB.push_back(10 * std::log10(noisySinr.at(i)));
    }

    std::vector<double> vectorVar;
    NS_LOG_DEBUG("noisySinrdBSize() " << noisySinrdB.size());
    for (uint64_t i = 0; i < noisySinrdB.size() - 1; ++i)
    {
        std::vector<double> partialSamples;
        partialSamples.push_back(noisySinrdB.at(i));
        partialSamples.push_back(noisySinrdB.at(i + 1));
        double meanValue = MakeAvg(partialSamples);
        double varValue = MakeVar(partialSamples, meanValue);
        vectorVar.push_back(varValue);
        partialSamples.clear();
    }

    uint64_t startFilter = 1e6;
    uint64_t endFilter = 1e6;

    bool flagStartFilter = true;
    bool flagEndFilter = true;

    uint64_t noisySinrIndex = 0;

    for (uint64_t varIndex = vectorVar.size() - 1; varIndex > 0;
         varIndex--) // start filter when variance of the noisy trace is high
    {
        NS_LOG_DEBUG("varIndex " << varIndex);
        noisySinrIndex = varIndex + 1;
        NS_LOG_DEBUG("vectorVar[i] " << vectorVar.at(varIndex));
        NS_LOG_DEBUG("vectorVar[i] == NaN " << (std::isnan(vectorVar.at(varIndex))));
        bool highVariance = (vectorVar.at(varIndex) > 5 || std::isnan(vectorVar.at(varIndex)));
        bool lowSinr = noisySinr.at(noisySinrIndex) < 10;

        if (highVariance ||
            (lowSinr && !highVariance)) // filter is applied only for low-SINR regimes [dB]
        {
            endFilter = noisySinrIndex;
            flagEndFilter = false; // a "start" sample has been identified
            break;
        }
    }

    if (flagEndFilter) // in this case, we can avoid the filtering
    {
        endFilter = 0;
    }

    /* in this case, consider at least a window of 15 samples, after which we can consider
     * as we are leaving the blockage phase and we start coming back to LOS PL regimes
     */
    uint64_t varIndex = 0;
    uint64_t numberOfVarWindow = 16;
    NS_LOG_DEBUG("VectorVarSize() " << vectorVar.size());
    for (uint64_t noisySinrIndex = endFilter; noisySinrIndex > numberOfVarWindow;
         --noisySinrIndex) // must be at least after the beginnning of the blocakge
    {
        NS_LOG_DEBUG("noisySinrIndex " << noisySinrIndex);
        varIndex = noisySinrIndex - 1;

        std::vector<double>::const_iterator first = vectorVar.begin() + varIndex;
        std::vector<double>::const_iterator last =
            vectorVar.begin() + varIndex -
            (numberOfVarWindow - 1); // vectorVar has one sample less than noisySinrdB
        std::vector<double> prov(last, first);

        std::vector<double>::const_iterator firstNoisy = noisySinrdB.begin() + noisySinrIndex;
        std::vector<double>::const_iterator lastNoisy =
            noisySinrdB.begin() + noisySinrIndex - numberOfVarWindow;
        std::vector<double> provNoisy(lastNoisy, firstNoisy);

        NS_LOG_INFO("provNoisy.size " << provNoisy.size());
        for (std::vector<double>::const_iterator h = provNoisy.begin(); h != provNoisy.end(); h++)
        {
            NS_LOG_INFO("h " << *h);
            NS_LOG_INFO("i " << noisySinrIndex);
            NS_LOG_INFO("hh " << *(noisySinrdB.begin() + noisySinrIndex - numberOfVarWindow));
        }

        /* the filtering ends when the variance of the noisy trace is almost the same, so when
         * the SINR is on sufficiently high values
         */

        if (Simulator::Now() > Seconds(2.1) && Simulator::Now() < Seconds(2.3))
        {
            NS_LOG_DEBUG(
                "(std::all_of(prov.begin(),prov.end(), [](double j){return j < 1;})) "
                << (std::all_of(prov.begin(), prov.end(), [](double j) { return j < 1; })));
            NS_LOG_DEBUG("(std::all_of(prov.begin(),prov.end(), [](double j){nan;})) "
                         << (std::all_of(prov.begin(), prov.end(), [](double k) {
                                return !std::isnan(k);
                            })));
            NS_LOG_DEBUG(
                "(std::all_of(provNoisy.begin(),provNoisy.end(), [](double p){return p > 10;})) "
                << (std::all_of(provNoisy.begin(), provNoisy.end(), [](double p) {
                       return p > 10;
                   })));
        }

        if (((std::all_of(prov.begin(), prov.end(), [](double j) { return j < 1; })) &&
             (std::all_of(prov.begin(), prov.end(), [](double k) { return !std::isnan(k); }))) ||
            (std::all_of(provNoisy.begin(), provNoisy.end(), [](double p) { return p > 10; })))
        {
            startFilter = noisySinrIndex;
            flagStartFilter = false; // a "end" sample has been identified
            break;
        }

        // bool lowVariance = (vectorVar.at(varIndex) < 1 && !std::isnan(vectorVar.at(varIndex)));
        // bool highSinr = noisySinr.at(noisySinrIndex) > 10;

        // if (highSinr && lowVariance)  // filter is applied only for low-SINR regimes [dB]
        // {
        //        startFilter = noisySinrIndex;
        //        flagStartFilter = false; // a "start" sample has been identified
        //        break;
        // }
    }

    if (flagStartFilter) // in this case, filter till the end of the trace
    {
        startFilter = 0;
    }

    std::pair<uint64_t, uint64_t> pairFiltering = std::make_pair(startFilter, endFilter);

    return pairFiltering;
}


std::vector<double>
MmWaveSidelinkPhy::MakeFilter(std::vector<double> noisySinr,
                         std::vector<double> realSinr,
                         std::pair<uint64_t, uint64_t> pairFiltering)
{
    for (uint64_t i = 0; i < noisySinr.size(); ++i)
    {
        NS_LOG_DEBUG("() " << noisySinr.at(i));
    }
    // const uint64_t lengthFiltering  = (std::get<1>(pairFiltering) - std::get<0>(pairFiltering));
    /* find best alpha parameter for the Kalman estimation */
    int rep = 0;
    std::array<double, 100> meanError;
    for (double alpha = 0; alpha < 1; alpha = alpha + 0.01)
    {
        std::vector<double> x;
        x.push_back(0); // initialization of array
        int counter = 0;

        for (uint64_t i = std::get<0>(pairFiltering); i < std::get<1>(pairFiltering); i++)
        {
            x.push_back((1 - alpha) * x.at(counter) + alpha * (noisySinr.at(i)));
            counter++;
        }

        std::vector<double> errorEstimation;
        counter = 0;
        for (uint64_t i = std::get<0>(pairFiltering); i < std::get<1>(pairFiltering); i++)
        {
            errorEstimation.push_back(std::abs(x.at(counter + 1) - realSinr.at(i)));
            counter++;
        }

        meanError.at(rep) = MakeAvg(errorEstimation);
        if (Simulator::Now() > Seconds(2.1) && Simulator::Now() < Seconds(2.3))
        {
            NS_LOG_DEBUG("meanError " << meanError.at(rep) << " rep " << rep);
        }
        rep++;
    }

    int posMinAlpha =
        std::distance(meanError.begin(), std::min_element(meanError.begin(), meanError.end()));
    double minAlpha = (posMinAlpha + 1) * 0.01;
    if (minAlpha > 0.5)
    {
        minAlpha = 0.2;
    }
    NS_LOG_DEBUG("! " << minAlpha);

    std::vector<double> blockageTrace;
    blockageTrace.push_back(0);
    int counter = 0;
    for (uint64_t i = std::get<0>(pairFiltering); i < std::get<1>(pairFiltering); i++)
    {
        NS_LOG_DEBUG(noisySinr.at(i));
        blockageTrace.push_back((1 - minAlpha) * blockageTrace.at(counter) +
                                minAlpha * (noisySinr.at(i)));
        NS_LOG_DEBUG("fff " << blockageTrace.at(counter));
        counter++;
    }

    std::vector<double> retFinalTrace;
    /* first piece */
    std::vector<double>::const_iterator firstPieceStart = noisySinr.begin();
    std::vector<double>::const_iterator firstPieceEnd =
        noisySinr.begin() + std::get<0>(pairFiltering) + 1;
    std::vector<double> firstPiece;
    firstPiece.insert(firstPiece.begin(), firstPieceStart, firstPieceEnd);
    for (std::vector<double>::const_iterator i = firstPiece.begin(); i != firstPiece.end(); i++)
    {
        NS_LOG_DEBUG("/ " << *i);
    }

    /*last piece*/
    std::vector<double>::const_iterator lastPieceStart =
        noisySinr.begin() + std::get<1>(pairFiltering) + 1;
    std::vector<double>::const_iterator lastPieceEnd = noisySinr.end();
    std::vector<double> lastPiece;

    firstPiece.insert(firstPiece.end(), blockageTrace.begin() + 1, blockageTrace.end() - 1);
    for (std::vector<double>::const_iterator i = firstPiece.begin(); i != firstPiece.end(); i++)
    {
        NS_LOG_DEBUG("// " << *i);
    }

    firstPiece.insert(firstPiece.end(), lastPieceStart, lastPieceEnd);
    for (std::vector<double>::const_iterator i = firstPiece.begin(); i != firstPiece.end(); i++)
    {
        NS_LOG_DEBUG("/// " << *i);
    }

    /* insert blockageTrace (from begin + 1, in order to AVOID THE TRANSIENT, to end) in the
     * noisySinr trace, after std::get<0>(pairFiltering) +1 samples, which are the samples in which
     * we estimate that a blockage occurs and the Kalmn filter is applied
     */
    // std::copy(blockageTrace.begin(),blockageTrace.end(),noisySinr.begin()+std::get<0>(pairFiltering)
    // );
    // std::copy(blockageTrace.begin()+1,blockageTrace.end(),noisySinr.begin()+std::get<0>(pairFiltering)+1
    // ); // AVOID TRANSIENT

    return firstPiece; // this noisySinr trace has already been updated with the filtered samples,
                       // where applied.
}






void
MmWaveSidelinkPhy::SetLastCommuncatingDeviceRnti(uint16_t rnti){
  m_lastCommuncatingDeviceRnti = rnti;
}

uint16_t
MmWaveSidelinkPhy::GetLastCommuncatingDeviceRnti(){
  return m_lastCommuncatingDeviceRnti;
}

// modified
void
MmWaveSidelinkPhy::ConfigureBeamformingByRnti(uint16_t rnti){
  NS_LOG_FUNCTION(this);
  m_sidelinkSpectrumPhy->ConfigureBeamforming(m_deviceMap.at(rnti));
}

void
MmWaveSidelinkPhy::SetSpectrumPropagationLossModel(Ptr<PhasedArraySpectrumPropagationLossModel> propagationLossModel){
  m_phasedArraySpectrumPropagationLossModel = propagationLossModel;
}

void
MmWaveSidelinkPhy::SetPropagationLossModel(Ptr<PropagationLossModel> model)
{
    m_propagationLoss = model;
}
// end modification

void
MmWaveSidelinkPhy::UpdateUePeerSinrEstimate(){
  NS_LOG_FUNCTION(this);

    m_sinrMillicarPairDevicesMap.clear();
    m_rxPsdMillicarPairMap.clear();

    Ptr<SpectrumValue> noisePsd =
        mmwave::MmWaveSpectrumValueHelper::CreateNoisePowerSpectralDensity(m_phyMacConfig, m_noiseFigure);
    Ptr<SpectrumValue> totalReceivedPsd =
        Create<SpectrumValue>(SpectrumValue(noisePsd->GetSpectrumModel()));

    // all ue nodes have the same config and use all the rb for transmiting   
    std::vector<int> subChannelsForTx (m_phyMacConfig->GetNumRb ());
    for (uint32_t i = 0; i < subChannelsForTx.size (); i++)
    {
      subChannelsForTx.at(i) = i;
    }

    // iterate over all devices and get their spectral details
    for (std::map<uint64_t, Ptr<NetDevice>>::iterator ue = m_deviceMap.begin();
         ue != m_deviceMap.end();
         ++ue)
    {
      Ptr<mmwave::MmWaveMillicarUeNetDevice> uePeerNetDevice =
            DynamicCast<mmwave::MmWaveMillicarUeNetDevice>(ue->second);

      Ptr<MmWaveSidelinkPhy> uePhy;
      // get tx power
      double ueTxPower = 0;
      uint16_t peerLastComunicatingDeviceRnti = UINT16_MAX;
      
      
      
      if (uePeerNetDevice!=nullptr)
      {
          uePhy = uePeerNetDevice->GetPhyMillicar();
          ueTxPower = uePhy->GetTxPower();
          peerLastComunicatingDeviceRnti = uePhy->GetLastCommuncatingDeviceRnti();
      }

      NS_LOG_LOGIC("UE Tx power = " << ueTxPower);
      double powerTxW = std::pow(10., (ueTxPower - 30) / 10);
      double txPowerDensity = 0;  
      txPowerDensity = (powerTxW / (m_phyMacConfig->GetBandwidth()));

      NS_LOG_LOGIC("Linear UE Tx power = " << powerTxW);
      NS_LOG_LOGIC("System bandwidth = " << m_phyMacConfig->GetBandwidth());
      NS_LOG_LOGIC("txPowerDensity = " << txPowerDensity);

      // all ue nodes have the same conf, m_listOfSubchannels contains all the subch
      
      Ptr<SpectrumValue> txPsd = mmwave::MmWaveSpectrumValueHelper::CreateTxPowerSpectralDensity(m_phyMacConfig,
                                                                    ueTxPower,
                                                                    subChannelsForTx);

      // NS_LOG_LOGIC("TxPsd " << *txPsd);

      // get this node and remote node mobility
      Ptr<MobilityModel> thisNodeMob = m_netDevice->GetNode()->GetObject<MobilityModel>();
      NS_LOG_LOGIC("eNB mobility " << thisNodeMob->GetPosition());
      Ptr<MobilityModel> peerNodeMob = ue->second->GetNode()->GetObject<MobilityModel>();
      NS_LOG_LOGIC("UE mobility " << peerNodeMob->GetPosition());


      // compute rx psd
      // adjuts beamforming of antenna model wrt user
      if (uePeerNetDevice!=nullptr)
      {
        // NS_LOG_DEBUG("Configure beamforming peer net device");
        m_sidelinkSpectrumPhy->ConfigureBeamforming(uePeerNetDevice);
        // NS_LOG_DEBUG("Configure beamforming local net device");
        uePhy->GetSpectrumPhy()->ConfigureBeamforming(m_netDevice);
      }

      // m_sidelinkSpectrumPhy->GetBeamformingModel()->GetAttributeFailSafe("SpectrumPropagationLossModel", propagationLoss); // ->GetAntenna()
      
      // Dl, since the Ul is not actually used (TDD device)
      double pathLossDb = 0;
      if (m_propagationLoss)
      {
        double propagationGainDb = m_propagationLoss->CalcRxPower(0, thisNodeMob, peerNodeMob);
        NS_LOG_LOGIC("propagationGainDb = " << propagationGainDb << " dB");
        pathLossDb -= propagationGainDb;
      }

      NS_LOG_DEBUG("Total pathLoss = " << pathLossDb << " dB");

      double pathGainLinear = std::pow(10.0, (-pathLossDb) / 10.0);
      Ptr<SpectrumValue> rxPsd = txPsd->Copy();
      *(rxPsd) *= pathGainLinear;

      // Not actually used for the gain, but needed for the call to CalcRxPowerSpectralDensity
      // anyway
      Ptr<PhasedArrayModel> rxPam =
          DynamicCast<PhasedArrayModel>(GetSpectrumPhy()->GetAntenna());
      Ptr<PhasedArrayModel> txPam =
          DynamicCast<PhasedArrayModel>(uePhy->GetSpectrumPhy()->GetAntenna());

      Ptr<SpectrumSignalParameters> rxParams = Create<SpectrumSignalParameters>();
      rxParams->psd = rxPsd->Copy();

      // if (m_spectrumPropagationLossModel)
      // {
      //     rxPsd =
      //         m_spectrumPropagationLossModel->CalcRxPowerSpectralDensity(rxParams, thisNodeMob, peerNodeMob);
      // }
      // else
      // only this is defined in sideling
      // channemode 
      if (m_phasedArraySpectrumPropagationLossModel)
      {
          rxPsd = m_phasedArraySpectrumPropagationLossModel->CalcRxPowerSpectralDensity(rxParams,
                                                                                        thisNodeMob,
                                                                                        peerNodeMob,
                                                                                        txPam,
                                                                                        rxPam);
      }

      // NS_LOG_LOGIC("RxPsd " << *rxPsd);

      m_rxPsdMillicarPairMap[ue->first] = rxPsd;
      *totalReceivedPsd += *rxPsd;

      // set back the bf vector to the main eNB
      if (uePeerNetDevice)
      { // have to reset the beamforming to the previous connection
        if (peerLastComunicatingDeviceRnti!= UINT16_MAX){
          // NS_LOG_DEBUG("Setting back the beamforming of peer device " << peerLastComunicatingDeviceRnti);
          uePhy->ConfigureBeamformingByRnti(peerLastComunicatingDeviceRnti);
        }
      }
      else
      {
          NS_FATAL_ERROR("Unrecognized device");
      }
  }

  for (std::map<uint64_t, Ptr<SpectrumValue>>::iterator ue = m_rxPsdMillicarPairMap.begin();
         ue != m_rxPsdMillicarPairMap.end();
         ++ue)
  {
      SpectrumValue interference = *totalReceivedPsd - *(ue->second);
      // NS_LOG_DEBUG("interference " << interference);
      SpectrumValue sinr = *(ue->second) / (*noisePsd); // + interference);
      // we consider the SNR only!
      // NS_LOG_DEBUG("sinr " << sinr);
      double sinrAvg = Sum(sinr) / (sinr.GetSpectrumModel()->GetNumBands());
      NS_LOG_LOGIC("Time " << Simulator::Now().GetSeconds() << " m_rnti " << m_rnti  << " UE " 
                            << ue->first << "Average SINR " << 10 * std::log10(sinrAvg));

      if (m_noiseAndFilter)
        {
            millicarPairDevices_t pairDevices =
                std::make_pair(ue->first, m_rnti);            // this is the current pair (UE-peer)
            auto iteratorSinr = m_sinrVector.find(pairDevices); // pair [pairDevices,Sinrvalue]
            if (iteratorSinr != m_sinrVector.end()){ // this map has already been initialized, so I
                                                    // can add a new element for the SINR collection
            
                if (Now().GetMicroSeconds() <= m_transient)
                {
                    m_sinrVector.at(pairDevices)
                        .push_back(sinrAvg); // before transient, so just collect SINR values
                }
                else
                {
                    m_sinrVector.at(pairDevices).erase(m_sinrVector.at(pairDevices).begin());
                    m_sinrVector.at(pairDevices)
                        .push_back(sinrAvg); // before transient, so just collect SINR values
                }

                NS_LOG_DEBUG("At time " << Now().GetMicroSeconds() << " push back the REAL SINR "
                                        << 10 * std::log10(sinrAvg) << " for pair with m_rnti "
                                        << m_rnti<< " and UE " << ue->first);
            }
            else{ // vector is not initialized, so it means that we are still in the initial
                 // transient phase, for that pair
            
                m_sinrVector.insert(std::make_pair(pairDevices, std::vector<double>()));
                m_sinrVector.at(pairDevices).push_back(sinrAvg); // push back a new SINR value
                NS_LOG_DEBUG("At time " << Now().GetMicroSeconds()
                                        << " first initializazion and push back the SINR "
                                        << 10 * std::log10(sinrAvg) << " for pair with m_rnti "
                                        << m_rnti << " and UE " << ue->first);
            }

            auto iteratorFinalTrace =
                m_finalSinrVector.find(pairDevices); // pair [pairDevices,Sinrvalue]
            if (iteratorFinalTrace == m_finalSinrVector.end())
            {
                m_samplesFilter.insert(
                    std::make_pair(pairDevices, std::pair<uint64_t, uint64_t>()));
                m_finalSinrVector.insert(std::make_pair(pairDevices, std::vector<double>()));
            }

            auto iteratorSinrToFilter =
                m_sinrVectorToFilter.find(pairDevices); // pair [pairDevices,Sinrvalue]
            if (iteratorSinrToFilter == m_sinrVectorToFilter.end())
            {
                m_sinrVectorToFilter.insert(std::make_pair(pairDevices, std::vector<double>()));
                m_sinrVectorNoisy.insert(std::make_pair(pairDevices, std::vector<double>()));
            }

            /* generate Gaussian noise for the last SINR value (that is the current one) */
            double sinrNoisy = AddGaussianNoise(m_sinrVector.at(pairDevices).back());

            /* UPDATE TRACE TO BE FILTERED */
            if (Now().GetMicroSeconds() <= m_transient)
            {
                m_sinrVectorNoisy.at(pairDevices).push_back(sinrNoisy);

                // if (sinrNoisy < 0)
                // {
                //        NS_LOG_DEBUG("Old SINR value was " << 10*std::log10(sinrNoisy) << " while
                //        now is " << 10*std::log10(0.1)); sinrNoisy = 0.1;
                // }
                m_sinrVectorToFilter.at(pairDevices).push_back(sinrNoisy);
            }
            else
            {
                m_sinrVectorNoisy.at(pairDevices).erase(m_sinrVectorNoisy.at(pairDevices).begin());
                m_sinrVectorNoisy.at(pairDevices).push_back(sinrNoisy);

                // if (sinrNoisy < 0)
                // {
                //        NS_LOG_DEBUG("Old SINR value was " << 10*std::log10(sinrNoisy) << " while
                //        now is " << 10*std::log10(0.1)); sinrNoisy = 0.1;
                // }

                double toPlot = *m_sinrVectorToFilter.at(pairDevices).begin();
                double toPlotbis = sinrNoisy;
                NS_LOG_DEBUG("(Remove SINR)  " << toPlot);
                NS_LOG_DEBUG("(Add SINR)  " << toPlotbis);

                m_sinrVectorToFilter.at(pairDevices)
                    .erase(m_sinrVectorToFilter.at(pairDevices).begin());
                m_sinrVectorToFilter.at(pairDevices).push_back(sinrNoisy);
            }

            if (Now().GetMicroSeconds() > m_transient){ // apply filter only when I have a
                                                       // sufficiently large set of SINR samples
            
                std::vector<double> vectorNoisy = m_sinrVectorNoisy.at(pairDevices);
                std::pair<uint64_t, uint64_t> pairFiltering = ApplyFilter(
                    vectorNoisy); // find where to apply the filter, according to the variance
                m_samplesFilter.at(pairDevices) = pairFiltering; // where to apply the linear filter
                NS_LOG_DEBUG("Noisy vector start at sample " << std::get<0>(pairFiltering));
                NS_LOG_DEBUG("Noisy vector end at sample " << std::get<1>(pairFiltering));

                /* just apply filter where the SINR is too low and we are in a blockage situation */
                if (std::get<0>(m_samplesFilter.at(pairDevices)) ==
                    std::get<1>(m_samplesFilter.at(pairDevices))) // if start = end
                {
                    m_finalSinrVector.at(pairDevices) =
                        m_sinrVectorToFilter.at(pairDevices); // no need to apply the filter
                    NS_LOG_DEBUG("At time "
                                 << Now().GetMicroSeconds()
                                 << " there is no need to apply the Kalman filter for mmWave ue m_rnti "
                                 << m_rnti << " and UE " << ue->first);
                }
                else
                {
                    std::vector<double> vectorToFilter = m_sinrVectorToFilter.at(pairDevices);
                    std::vector<double> sinrVector = m_sinrVector.at(pairDevices);
                    std::pair<uint64_t, uint64_t> filterPair = m_samplesFilter.at(pairDevices);
                    m_finalSinrVector.at(pairDevices) =
                        MakeFilter(vectorNoisy, sinrVector, filterPair);
                    NS_LOG_DEBUG("finaltrace " << m_finalSinrVector.at(pairDevices).back());
                }

                /* the last sample in the filtered sequence is referred to the current time instant,
                 * referred to the uplink reference signal that is used to build the RT in the
                 * LteEnbRrc class */
                double sampleToForward = m_finalSinrVector.at(pairDevices).back();
                if (sampleToForward < 0) // this would be converted in NaN, in the log scale
                {
                    sampleToForward = 1e-20;
                }
                NS_LOG_DEBUG(" mmWave ue m_rnti " << m_rnti << " reports the SINR "
                                            << 10 * std::log10(sampleToForward) << " for UE "
                                            << ue->first);
                m_sinrMillicarPairDevicesMap[ue->first] =
                    sampleToForward; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< in order to
                                     // FORWARD to LteEnbRrc the value of SINR for the RT

                // m_sinrVectorToFilter.at(pairDevices).erase(m_sinrVectorToFilter.at(pairDevices).begin());
                // m_sinrVectorToFilter.at(pairDevices).push_back(m_finalSinrVector.at(pairDevices).back());
                m_sinrVectorToFilter.at(pairDevices).pop_back();
                m_sinrVectorToFilter.at(pairDevices)
                    .push_back(m_finalSinrVector.at(pairDevices).back());
            }
            else{ // before the transient is over, just forwart the (last) noisy sample, without
                 // having filtered
            
                double sampleToForward = m_sinrVectorToFilter.at(pairDevices).back();
                if (sampleToForward < 0) // this would be converted in NaN, in the log scale
                {
                    sampleToForward = 1e-20;
                }
                NS_LOG_DEBUG(" mmWave ue m_rnti " << m_rnti << " FIRST reports the SINR "
                                            << 10 * std::log10(sampleToForward) << " for UE "
                                            << ue->first);
                m_sinrMillicarPairDevicesMap[ue->first] =
                    sampleToForward; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< in order to
                                     // FORWARD to LteEnbRrc the value of SINR for the RT
            }

            /* after the SINR sample is forwarded, I need to REFRESH all the maps, so that
             * a new SINR sequence can be created, to generate a new SINR sample
             * for the next RT
             */
            m_finalSinrVector.erase(pairDevices);
            m_samplesFilter.erase(pairDevices);
            NS_LOG_DEBUG("ERASE MAPS FOR RT");
        }
        else // noise and filtering processes are not applied!
        {
            m_sinrMillicarPairDevicesMap[ue->first] = sinrAvg;
        }

  }

  // m_sinrMapDeviceForwardCallback(m_rnti ,m_sinrMillicarPairDevicesMap);
  // report to traces
  ReportSinrMillicarPairs();


  // LteEnbCphySapUser::UeAssociatedSinrInfo info;
  // info.ueImsiSinrMap = m_sinrMillicarPairDevicesMap;
  // info.componentCarrierId = m_componentCarrierId;
  // m_enbCphySapUser->UpdateUeSinrEstimate(info);

  Simulator::Schedule(MicroSeconds(m_updateSinrPeriod),
                      &MmWaveSidelinkPhy::UpdateUePeerSinrEstimate,
                      this); // recall after m_updateSinrPeriod microseconds
}


void 
MmWaveSidelinkPhy::ReportSinrMillicarPairs(){
  NS_LOG_FUNCTION (this);
  for (std::map<uint64_t, double>::iterator peerMapIter = m_sinrMillicarPairDevicesMap.begin();
         peerMapIter != m_sinrMillicarPairDevicesMap.end();
         ++peerMapIter)
    {
      // rnti is the id of the device generating the report; in the map first index is the peer id
      // second index is the sinr
      m_notifyMillicarPairsSinrTrace(m_rnti, peerMapIter->first, peerMapIter->second);
    }
}


void
MmWaveSidelinkPhy::SetMillicarSinrMapReportCallback(Callback<void, uint16_t, std::map<uint64_t, double>> sinrMapDeviceForwardCallback){
  m_sinrMapDeviceForwardCallback = sinrMapDeviceForwardCallback;
}

void
MmWaveSidelinkPhy::SetDevice(Ptr<NetDevice> d)
{
    m_netDevice = d;
}

Ptr<NetDevice>
MmWaveSidelinkPhy::GetDevice()
{
    return m_netDevice;
}

void 
MmWaveSidelinkPhy::PrintRelay(){
  for(auto it = m_relayPaths.begin(); it != m_relayPaths.end(); ++it)
  {
    for (auto secondIt = it->second.begin(); secondIt != it->second.end(); ++secondIt){
      std::cout << it->first << " " << secondIt->first << " " << secondIt->second << "\n";
    }
  }
}

// end modification

void
MmWaveSidelinkPhy::StartSlot (mmwave::SfnSf timingInfo)
{
   NS_LOG_FUNCTION (this << " frame " << timingInfo.m_frameNum << " subframe " << timingInfo.m_sfNum << " slot " << timingInfo.m_slotNum);

  // trigger the MAC
  m_phySapUser->SlotIndication (timingInfo);

  while (m_phyBuffer.size () != 0)
  {
    uint8_t usedSymbols = 0; // the symbol index

    // retrieve the first element in the list
    Ptr<PacketBurst> pktBurst;
    mmwave::TtiAllocInfo info;
    std::tie (pktBurst, info) = m_phyBuffer.front ();

    // modified
    // check if we have rnti in the relay path; if so, change rnti to intermediate
    uint16_t intermediateDest = UINT16_MAX;
    // check if rntiDest is in relay path; if it is, then we have to change the destination
    // for the path
    auto relayPathThisRntiIt =  m_relayPaths.find(GetRnti());
    if (relayPathThisRntiIt != m_relayPaths.end()){
      auto relayPathDestIt = relayPathThisRntiIt->second.find(info.m_rnti);
      if (relayPathDestIt != relayPathThisRntiIt->second.end()){
        // means there is a relay path, thus the destRnti is not the former
        // but the intermediate node
        intermediateDest = relayPathDestIt->second;
      }
    }

    if (intermediateDest != UINT16_MAX){
      info.m_rnti = intermediateDest;
    }
    // end modification

    // send the transport block
    if (info.m_ttiType == mmwave::TtiAllocInfo::DATA)
    {
      usedSymbols += SlData (pktBurst, info);
    }
    else if (info.m_ttiType == mmwave::TtiAllocInfo::CTRL)
    {
      NS_FATAL_ERROR ("Control messages are not currently supported");
    }
    else
    {
      NS_FATAL_ERROR ("Unknown TB type");
    }

    // check if we exceeded the slot boundaries
    NS_ASSERT_MSG (usedSymbols <= m_phyMacConfig->GetSymbPerSlot (), "Exceeded number of available symbols");

    // remove the transport block from the buffer
    m_phyBuffer.pop_front ();
  }

  // update the timing information
  timingInfo = UpdateTimingInfo (timingInfo);
  Simulator::Schedule (m_phyMacConfig->GetSlotPeriod (), &MmWaveSidelinkPhy::StartSlot, this, timingInfo);
}

uint8_t
MmWaveSidelinkPhy::SlData (Ptr<PacketBurst> pb, mmwave::TtiAllocInfo info)
{
  NS_LOG_FUNCTION (this);

  // create the tx PSD
  //TODO do we need to create a new psd at each TTI?
  std::vector<int> subChannelsForTx = SetSubChannelsForTransmission ();

  // compute the tx start time (IndexOfTheFirstSymbol * SymbolDuration)
  Time startTime = info.m_dci.m_symStart * m_phyMacConfig->GetSymbolPeriod ();
  NS_ASSERT_MSG (startTime == info.m_dci.m_symStart * m_phyMacConfig->GetSymbolPeriod (), "startTime was not been correctly set");

  // compute the duration of the transmission (NumberOfSymbols * SymbolDuration)
  Time duration = info.m_dci.m_numSym * m_phyMacConfig->GetSymbolPeriod ();

  NS_ASSERT_MSG (duration == info.m_dci.m_numSym * m_phyMacConfig->GetSymbolPeriod (), "duration was not been correctly set");

  // send the transport block
  Simulator::Schedule (startTime, &MmWaveSidelinkPhy::SendDataChannels, this,
                       pb,
                       duration,
                       info,
                       subChannelsForTx);

  return info.m_dci.m_numSym;
}

void
MmWaveSidelinkPhy::SendDataChannels (Ptr<PacketBurst> pb,
  Time duration,
  mmwave::TtiAllocInfo info,
  std::vector<int> rbBitmap)
{
  NS_LOG_FUNCTION(this);
  // if ((GetRnti() == 5) & (info.m_rnti == 1)){
  //   NS_LOG_UNCOND(Simulator::Now() <<  " Sending data from 5 to 1");
  // }
  // retrieve the RNTI of the device we want to communicate with and properly
  // configure the beamforming
  // NOTE: this information is contained in mmwave::TtiAllocInfo.m_rnti parameter
  NS_ASSERT_MSG (m_deviceMap.find (info.m_rnti) != m_deviceMap.end (), "Device not found " << info.m_rnti << " in " << m_rnti);
  m_lastCommuncatingDeviceRnti = info.m_rnti;
  m_sidelinkSpectrumPhy->ConfigureBeamforming (m_deviceMap.at (info.m_rnti));

  m_sidelinkSpectrumPhy->StartTxDataFrames (pb, duration, info.m_dci.m_mcs, info.m_dci.m_tbSize, info.m_dci.m_numSym, info.m_dci.m_rnti, info.m_rnti, rbBitmap);
}

std::vector<int>
MmWaveSidelinkPhy::SetSubChannelsForTransmission ()
  {
    NS_LOG_FUNCTION(this);
    // create the transmission mask, use all the available subchannels
    std::vector<int> subChannelsForTx (m_phyMacConfig->GetNumRb ());
    for (uint32_t i = 0; i < subChannelsForTx.size (); i++)
    {
      subChannelsForTx.at(i) = i;
    }

    // create the tx PSD
    Ptr<SpectrumValue> txPsd = mmwave::MmWaveSpectrumValueHelper::CreateTxPowerSpectralDensity (m_phyMacConfig, m_txPower, subChannelsForTx);

    // set the tx PSD in the spectrum phy
    m_sidelinkSpectrumPhy->SetTxPowerSpectralDensity (txPsd);

    return subChannelsForTx;
  }

mmwave::SfnSf
MmWaveSidelinkPhy::UpdateTimingInfo (mmwave::SfnSf info) const
{
  NS_LOG_INFO (this);

  uint32_t nextSlot = info.m_slotNum + 1;
  uint32_t nextSf = info.m_sfNum;
  uint32_t nextFrame = info.m_frameNum;

  if (nextSlot == m_phyMacConfig->GetSlotsPerSubframe ())
  {
    nextSlot = 0;
    ++nextSf;

    if (nextSf == m_phyMacConfig->GetSubframesPerFrame ())
    {
      nextSf = 0;
      ++nextFrame;
    }
  }

  // update the mmwave::SfnSf structure
  info.m_slotNum = nextSlot;
  info.m_sfNum = nextSf;
  info.m_frameNum = nextFrame;

  return info;
}

void
MmWaveSidelinkPhy::DoPrepareForReceptionFrom (uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_deviceMap.find (rnti) != m_deviceMap.end (), "Cannot find device with rnti " << rnti);
  m_lastCommuncatingDeviceRnti = rnti;
  m_sidelinkSpectrumPhy->ConfigureBeamforming (m_deviceMap.at (rnti));
}

void
MmWaveSidelinkPhy::AddDevice (uint64_t rnti, Ptr<NetDevice> dev)
{
  NS_LOG_FUNCTION (this);

  if (m_deviceMap.find (rnti) == m_deviceMap.end ())
  {
    m_deviceMap.insert (std::make_pair (rnti,dev));
  }
  else
  {
    NS_FATAL_ERROR ("Device with rnti " << rnti << " already present in the map");
  }
}

void
MmWaveSidelinkPhy::Receive (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this);

  // Forward the received packet to the MAC layer using the PHY SAP USER
  m_phySapUser->ReceivePhyPdu(p);
}

void
MmWaveSidelinkPhy::GenerateSinrReport (const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize, uint8_t mcs)
{
  NS_LOG_FUNCTION (this << rnti << (uint32_t)numSym << tbSize << (uint32_t)mcs);

  double sinrAvg = Sum (sinr) / (sinr.GetSpectrumModel ()->GetNumBands ());
  NS_LOG_INFO ("Average SINR with dev " << rnti << " = " << 10 * std::log10 (sinrAvg));

  // forward the report to the MAC layer
  m_phySapUser->SlSinrReport (sinr, rnti, numSym, tbSize);

  // Ptr<MobilityModel> mobility = m_phySapProvider->GetSpectrum()->GetMobility();

  double x = -1;
  double y = -1;

  if (m_netDevice != nullptr){
    Ptr<Node> node = m_netDevice->GetNode();
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    // if (mobility!=nullptr){
      x = mobility->GetPosition().x;
      y = mobility->GetPosition().y;
    // }
    // NS_LOG_DEBUG("Mobility found " << mobility->GetPosition().y);
  }
  m_slSinrReportTrace(m_rnti, rnti, numSym, tbSize, sinrAvg); // , mobility->GetPosition().x, mobility->GetPosition().y
}

} // namespace millicar
} // namespace ns3
