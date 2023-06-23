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
*                Dual Connectivity and Handover functionalities
*
* Modified by: Tommaso Zugno <tommasozugno@gmail.com>
*								 Integration of Carrier Aggregation
*/


#include "ns3/mmwave-bearer-stats-calculator.h"
#include "mmwave-bearer-stats-connector.h"

#include <ns3/log.h>
#include <ns3/config.h>

#include "ns3/string.h"
#include "ns3/nstime.h"

#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-ue-net-device.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveBearerStatsConnector");

namespace mmwave {

NS_OBJECT_ENSURE_REGISTERED (MmWaveBearerStatsConnector);

/**
  * Less than operator for CellIdRnti, because it is used as key in map
  */
bool
operator < (const MmWaveBearerStatsConnector::CellIdRnti& a, const MmWaveBearerStatsConnector::CellIdRnti& b)
{
  return ( (a.cellId < b.cellId) || ( (a.cellId == b.cellId) && (a.rnti < b.rnti) ) );
}

/**
 * This structure is used as interface between trace
 * sources and MmWaveBearerStatsCalculator. It stores
 * and provides calculators with cellId and IMSI,
 * because most trace sources do not provide it.
 */
struct MmWaveBoundCallbackArgument : public SimpleRefCount<MmWaveBoundCallbackArgument>
{
public:
  Ptr<MmWaveBearerStatsCalculator> stats;  //!< statistics calculator
  uint64_t imsi; //!< imsi
  uint16_t cellId; //!< cellId
};

struct McMmWaveBoundCallbackArgument : public SimpleRefCount<McMmWaveBoundCallbackArgument>
{
public:
  Ptr<McStatsCalculator> stats;
};

/**
 * Callback function for DL TX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 */
void
DlTxPduCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize);
  arg->stats->DlTxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize);
}

/**
 * Callback function for DL RX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
DlRxPduCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize << delay);
  arg->stats->DlRxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize, delay);
}

/**
 * Callback function for UL TX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 */
void
UlTxPduCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize);

  arg->stats->UlTxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize);
}

/**
 * Callback function for UL RX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
UlRxPduCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize << delay);

  arg->stats->UlRxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize, delay);
}

/**
 * Callback function for rlc buffer size statistics
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
GnbRlcBufferSizeCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
 uint32_t rlcBufferSize, uint32_t rlcMaxBufferSize)
{
  NS_LOG_FUNCTION (path << " rlc buff size " << rlcBufferSize << " max size " << rlcMaxBufferSize);
  arg->stats->GnbRlcBufferSize (arg->cellId, rlcBufferSize, rlcMaxBufferSize);
}

// modified

/**
 * Callback function for ue rlc buffer size statistics
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
UeRlcBufferSizeCallback (Ptr<MmWaveBoundCallbackArgument> arg, std::string path,
 uint32_t rlcBufferSize, uint32_t rlcMaxBufferSize)
{
  NS_LOG_FUNCTION (path << " rlc buff size " << rlcBufferSize << " max size " << rlcMaxBufferSize);
  arg->stats->UeRlcBufferSize (arg->cellId, arg->imsi, rlcBufferSize, rlcMaxBufferSize);
}


/**
 * Callback function for rlc tx packet drop
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
// void TxPacketDropCallBack(Ptr<NrBoundCallbackArgument> arg, std::string path, Ptr<const Packet> packet){
//   NS_LOG_FUNCTION (path << " rlc buff size " << rlcBufferSize << " max size " << rlcMaxBufferSize);
// }

// end modification

void
SwitchToLteCallback (Ptr<McMmWaveBoundCallbackArgument> arg, std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (path << rnti << cellId << imsi);

  arg->stats->SwitchToLte (imsi, cellId, rnti);
}

void
SwitchToMmWaveCallback (Ptr<McMmWaveBoundCallbackArgument> arg, std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (path << rnti << cellId << imsi);

  arg->stats->SwitchToMmWave (imsi, cellId, rnti);
}


MmWaveBearerStatsConnector::MmWaveBearerStatsConnector ()
  : m_connected (false),
    m_enbHandoverStartFilename ("EnbHandoverStartStats.txt"),
    m_enbHandoverEndFilename ("EnbHandoverEndStats.txt"),
    m_ueHandoverStartFilename ("UeHandoverStartStats.txt"),
    m_ueHandoverEndFilename ("UeHandoverEndStats.txt"),
    m_cellIdInTimeHandoverFilename ("CellIdStatsHandover.txt"),
    m_ueRrcStateFileName ("UeRrcStateTrace.txt"),
    m_enbRrcStateFileName ("EnbRrcStateTrace.txt")
{

}

MmWaveBearerStatsConnector::~MmWaveBearerStatsConnector ()
{
  NS_LOG_FUNCTION (this);
  if (m_enbHandoverStartOutFile.is_open ())
    {
      m_enbHandoverStartOutFile.close ();
    }
  if (m_ueHandoverStartOutFile.is_open ())
    {
      m_ueHandoverStartOutFile.close ();
    }
  if (m_enbHandoverEndOutFile.is_open ())
    {
      m_enbHandoverEndOutFile.close ();
    }
  if (m_ueHandoverEndOutFile.is_open ())
    {
      m_ueHandoverEndOutFile.close ();
    }

  if (m_cellIdInTimeHandoverOutFile.is_open ())
    {
      m_cellIdInTimeHandoverOutFile.close ();
    }
  if (m_mmWaveSinrOutFile.is_open ())
    {
      m_mmWaveSinrOutFile.close ();
    }
  if (m_lteSinrOutFile.is_open ())
    {
      m_lteSinrOutFile.close ();
    }
  if (m_enbRrcStateFile.is_open ())
    {
      m_enbRrcStateFile.close ();
    }
  if (m_ueRrcStateFile.is_open ())
    {
      m_ueRrcStateFile.close ();
    }
    
}

TypeId
MmWaveBearerStatsConnector::GetTypeId (void)
{
  static TypeId tid =
    TypeId ("ns3::MmWaveBearerStatsConnector")
    .SetParent<Object> ()
    .AddConstructor<MmWaveBearerStatsConnector> ()
    .SetGroupName ("Lte")
    .AddAttribute ("EnbHandoverStartOutputFilename",
                   "Name of the file where the eNB handover start traces will be saved.",
                   StringValue ("EnbHandoverStartStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetEnbHandoverStartOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("EnbHandoverEndOutputFilename",
                   "Name of the file where the eNB handover end traces will be saved.",
                   StringValue ("EnbHandoverEndStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetEnbHandoverEndOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("MmWaveSinrOutputFilename",
                   "Name of the file where the mmWave eNB sinr will be saved.",
                   StringValue ("MmWaveSinrTime.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetMmWaveSinrOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("LteSinrOutputFilename",
                   "Name of the file where the LTE eNB sinr will be saved.",
                   StringValue ("LteSinrTime.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetLteSinrOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeHandoverStartOutputFilename",
                   "Name of the file where the UE handover start events will be saved.",
                   StringValue ("UeHandoverStartStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetUeHandoverStartOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeHandoverEndOutputFilename",
                   "Name of the file where the UE handover end events will be saved.",
                   StringValue ("UeHandoverEndStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetUeHandoverEndOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("CellIdStatsHandoverOutputFilename",
                   "Name of the file where the current cellId for the UE will be stored.",
                   StringValue ("CellIdStatsHandover.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetCellIdStatsOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeStateTransitionOutputFilename",
                   "Name of the file where the UE state transition.",
                   StringValue ("UeRrcStateTrace.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetUeRrcStateTransitionFileName),
                   MakeStringChecker ())
    .AddAttribute ("EnbStateTransitionOutputFilename",
                   "Name of the file where the Enb state transition.",
                   StringValue ("EnbRrcStateTrace.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsConnector::SetEnbRrcStateTransitionFileName),
                   MakeStringChecker ())
    
  ;
  return tid;
}

void
MmWaveBearerStatsConnector::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

void
MmWaveBearerStatsConnector::EnableRlcStats (Ptr<MmWaveBearerStatsCalculator> rlcStats)
{
  m_rlcStats = rlcStats;
  EnsureConnected ();
}

void
MmWaveBearerStatsConnector::EnablePdcpStats (Ptr<MmWaveBearerStatsCalculator> pdcpStats)
{
  m_pdcpStats = pdcpStats;
  EnsureConnected ();
}

void
MmWaveBearerStatsConnector::EnableE2PdcpStats (Ptr<MmWaveBearerStatsCalculator> e2PdcpStats)
{
  if(m_e2PdcpStatsVector.empty()) {
    Simulator::Schedule(Seconds(0), &MmWaveBearerStatsConnector::EnsureConnected, this);
  }

  m_e2PdcpStatsVector.push_back(e2PdcpStats);
}

void
MmWaveBearerStatsConnector::EnableE2RlcStats (Ptr<MmWaveBearerStatsCalculator> e2RlcStats)
{
  if(m_e2RlcStatsVector.empty()) {
    Simulator::Schedule(Seconds(0), &MmWaveBearerStatsConnector::EnsureConnected, this);
  }

  m_e2RlcStatsVector.push_back(e2RlcStats);
}

void
MmWaveBearerStatsConnector::EnableMcStats (Ptr<McStatsCalculator> mcStats)
{
  m_mcStats = mcStats;
  EnsureConnected ();
}

void
MmWaveBearerStatsConnector::EnsureConnected ()
{
  NS_LOG_FUNCTION (this);
  if (!m_connected)
    {
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/NewUeContext",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyNewUeContextEnb, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyRandomAccessSuccessfulUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveUeRrc/RandomAccessSuccessful",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyRandomAccessSuccessfulUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionReconfiguration",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyConnectionReconfigurationEnb, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionReconfiguration",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyConnectionReconfigurationUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveUeRrc/ConnectionReconfiguration",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyConnectionReconfigurationUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverStartEnb, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverStartUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveUeRrc/HandoverStart",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverStartUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverEndOkEnb, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverEndOkUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveUeRrc/HandoverEndOk",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyHandoverEndOkUe, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/SwitchToMmWave",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifySwitchToMmWaveUe, this));
      // mmWave SINR from RT, LTE SINR from the PHY callbacks
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteEnbRrc/NotifyMmWaveSinr",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyMmWaveSinr, this));
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyLteSinr, this));
      // state transition
      Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                       MakeBoundCallback (&MmWaveBearerStatsConnector::NotifyUeRrcStateTransition, this));
      m_connected = true;
    }
}



void
MmWaveBearerStatsConnector::NotifyRandomAccessSuccessfulUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSrb0Traces (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyConnectionSetupUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSrb1TracesUe (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyConnectionReconfigurationUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesUeIfFirstTime (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyHandoverStartUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  c->PrintUeStartHandover (imsi, cellId, targetCellId, rnti);
  c->DisconnectTracesUe (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyHandoverEndOkUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->PrintUeEndHandover (imsi, cellId, rnti);
  c->ConnectSrb1TracesUe (context, imsi, cellId, rnti);
  c->ConnectDrbTracesUe (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyNewUeContextEnb (MmWaveBearerStatsConnector* c, std::string context, uint16_t cellId, uint16_t rnti)
{
  c->StoreUeManagerPath (context, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyConnectionReconfigurationEnb (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesEnbIfFirstTime (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyHandoverStartEnb (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  c->PrintEnbStartHandover (imsi, cellId, targetCellId, rnti);
  c->DisconnectTracesEnb (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyHandoverEndOkEnb (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->PrintEnbEndHandover (imsi, cellId, rnti);
  c->ConnectSrb1TracesEnb (context, imsi, cellId, rnti);
  c->ConnectDrbTracesEnb (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifySwitchToMmWaveUe (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSecondaryTracesUe (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifySecondaryMmWaveEnbAvailable (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSecondaryTracesEnb (context, imsi, cellId, rnti);
}

void
MmWaveBearerStatsConnector::NotifyMmWaveSinr (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, long double sinr)
{
  c->PrintMmWaveSinr (imsi, cellId, sinr);
}

// modified

void
MmWaveBearerStatsConnector::NotifyUeRrcStateTransition (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, LteUeRrc::State oldState, LteUeRrc::State newState)
{
  c->SwitchStateUe (imsi, cellId, rnti, oldState, newState);
}

void
MmWaveBearerStatsConnector::NotifyEnbRrcStateTransition (MmWaveBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, UeManager::State oldState, UeManager::State newState)
{
  c->SwitchStateEnb (imsi, cellId, rnti, oldState, newState);
}

const std::string MmWaveBearerStatsConnector::g_ueRrcStateName[] = {
		"IDLE_START",
		"IDLE_CELL_SEARCH",
		"IDLE_WAIT_MIB_SIB1",
		"IDLE_WAIT_MIB",
		"IDLE_WAIT_SIB1",
		"IDLE_CAMPED_NORMALLY",
		"IDLE_WAIT_SIB2",
		"IDLE_RANDOM_ACCESS",
		"IDLE_CONNECTING",
		"CONNECTED_NORMALLY",
		"CONNECTED_HANDOVER",
		"CONNECTED_PHY_PROBLEM",
		"CONNECTED_REESTABLISHING"};


void 
MmWaveBearerStatsConnector::SwitchStateUe(uint64_t imsi, uint16_t cellId, uint16_t rnti, LteUeRrc::State oldState, LteUeRrc::State newState)
{
	if (!m_ueRrcStateFile.is_open())
	{
		m_ueRrcStateFile.open(m_ueRrcStateFileName.c_str());
		m_ueRrcStateFile << "Time" << "\t" << "Imsi" << "\t" << "CellId" << "\t" << "Rnti" << "\t" << "OldState" << "\t" << "NewState" << std::endl;

		if (!m_ueRrcStateFile.is_open())
		{
			NS_FATAL_ERROR("Could not open tracefile");
		}
	}

	m_ueRrcStateFile << Simulator::Now().GetNanoSeconds() << // (double) 1e9
		"\t" << imsi << "\t" << (uint32_t)cellId << "\t" << (uint32_t)rnti << "\t" << MmWaveBearerStatsConnector::ueToStringState(oldState) << "\t" << MmWaveBearerStatsConnector::ueToStringState(newState) << std::endl;
}

const std::string MmWaveBearerStatsConnector::g_ueManagerStateName[] = {
    "INITIAL_RANDOM_ACCESS",
    "CONNECTION_SETUP",
    "CONNECTION_REJECTED",
    "CONNECTED_NORMALLY",
    "CONNECTION_RECONFIGURATION",
    "CONNECTION_REESTABLISHMENT",
    "HANDOVER_PREPARATION",
    "HANDOVER_JOINING",
    "HANDOVER_PATH_SWITCH",
    "HANDOVER_LEAVING",
    "PREPARE_MC_CONNECTION_RECONFIGURATION",
    "MC_CONNECTION_RECONFIGURATION"};
void 
MmWaveBearerStatsConnector::SwitchStateEnb(uint64_t imsi, uint16_t cellId, uint16_t rnti, UeManager::State oldState, UeManager::State newState)
{
  NS_LOG_FUNCTION (this << " Switching state " << Simulator::Now ().GetSeconds ());
	if (!m_enbRrcStateFile.is_open())
	{
		m_enbRrcStateFile.open(m_enbRrcStateFileName.c_str());
		m_enbRrcStateFile << "Time" << "\t" << "Imsi" << "\t" << "CellId" << "\t" << "Rnti" << "\t" << "OldState" << "\t" << "NewState" << std::endl;
	}

	m_enbRrcStateFile << Simulator::Now().GetNanoSeconds() << // (double) 1e9
		"\t" << imsi << "\t" << (uint32_t)cellId << "\t" << (uint32_t)rnti << "\t" << gnbToStringState(oldState) << "\t" << gnbToStringState(newState) << std::endl;
}
// end modification

void
MmWaveBearerStatsConnector::PrintMmWaveSinr (uint64_t imsi, uint16_t cellId, long double sinr)
{
  NS_LOG_FUNCTION (this << " PrintMmWaveSinr " << Simulator::Now ().GetSeconds ());
  if (!m_mmWaveSinrOutFile.is_open ())
    {
      m_mmWaveSinrOutFile.open (GetMmWaveSinrOutputFilename ().c_str ());
    }
  m_mmWaveSinrOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << cellId << " " << 10 * std::log10 (sinr) << std::endl;
}

void
MmWaveBearerStatsConnector::NotifyLteSinr (MmWaveBearerStatsConnector* c, std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t cc)
{
  c->PrintLteSinr (rnti, cellId, sinr);
}

void
MmWaveBearerStatsConnector::PrintLteSinr (uint16_t rnti, uint16_t cellId, double sinr)
{
  NS_LOG_FUNCTION (this << " PrintLteSinr " << Simulator::Now ().GetSeconds ());
  if (!m_lteSinrOutFile.is_open ())
    {
      m_lteSinrOutFile.open (GetLteSinrOutputFilename ().c_str ());
    }
  m_lteSinrOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << rnti << " " << cellId << " " << sinr << std::endl;
}

std::string
MmWaveBearerStatsConnector::GetEnbHandoverStartOutputFilename (void)
{
  return m_enbHandoverStartFilename;
}

std::string
MmWaveBearerStatsConnector::GetEnbHandoverEndOutputFilename (void)
{
  return m_enbHandoverEndFilename;
}

std::string
MmWaveBearerStatsConnector::GetMmWaveSinrOutputFilename (void)
{
  return m_mmWaveSinrOutputFilename;
}

std::string
MmWaveBearerStatsConnector::GetLteSinrOutputFilename (void)
{
  return m_lteSinrOutputFilename;
}

std::string
MmWaveBearerStatsConnector::GetUeHandoverStartOutputFilename (void)
{
  return m_ueHandoverStartFilename;
}

std::string
MmWaveBearerStatsConnector::GetUeHandoverEndOutputFilename (void)
{
  return m_ueHandoverEndFilename;
}

std::string
MmWaveBearerStatsConnector::GetCellIdStatsOutputFilename (void)
{
  return m_cellIdInTimeHandoverFilename;
}

std::string
MmWaveBearerStatsConnector::GetEnbRrcStateTransitionFileName(void){
  return m_enbRrcStateFileName;
}

std::string
MmWaveBearerStatsConnector::GetUeRrcStateTransitionFileName(void){
  return m_ueRrcStateFileName;
}

void MmWaveBearerStatsConnector::SetEnbRrcStateTransitionFileName (std::string outputFilename)
{
  m_enbRrcStateFileName = outputFilename;
}

void MmWaveBearerStatsConnector::SetUeRrcStateTransitionFileName (std::string outputFilename)
{
  m_ueRrcStateFileName = outputFilename;
}

void MmWaveBearerStatsConnector::SetEnbHandoverEndOutputFilename (std::string outputFilename)
{
  m_enbHandoverEndFilename = outputFilename;
}

void MmWaveBearerStatsConnector::SetEnbHandoverStartOutputFilename (std::string outputFilename)
{
  m_enbHandoverStartFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::SetUeHandoverStartOutputFilename (std::string outputFilename)
{
  m_ueHandoverStartFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::SetUeHandoverEndOutputFilename (std::string outputFilename)
{
  m_ueHandoverEndFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::SetCellIdStatsOutputFilename (std::string outputFilename)
{
  m_cellIdInTimeHandoverFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::SetMmWaveSinrOutputFilename (std::string outputFilename)
{
  m_mmWaveSinrOutputFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::SetLteSinrOutputFilename (std::string outputFilename)
{
  m_lteSinrOutputFilename = outputFilename;
}

void
MmWaveBearerStatsConnector::PrintEnbStartHandover (uint64_t imsi, uint16_t sourceCellid, uint16_t targetCellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << " NotifyHandoverStartEnb " << Simulator::Now ().GetSeconds ());
  if (!m_enbHandoverStartOutFile.is_open ())
    {
      m_enbHandoverStartOutFile.open (GetEnbHandoverStartOutputFilename ().c_str ());
      m_enbHandoverStartOutFile  << "Timestamp" << " " << "imsi" << " " << "rnti" << " " <<  "sourceCellId" << " " << "targetCellId"<< std::endl;
    }
  m_enbHandoverStartOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << rnti << " " << sourceCellid << " " << targetCellId << std::endl;
}

void
MmWaveBearerStatsConnector::PrintEnbEndHandover (uint64_t imsi, uint16_t targetCellId, uint16_t rnti)
{
  NS_LOG_FUNCTION ("NotifyHandoverOkEnb " << Simulator::Now ().GetSeconds ());
  if (!m_enbHandoverEndOutFile.is_open ())
    {
      m_enbHandoverEndOutFile.open (GetEnbHandoverEndOutputFilename ().c_str ());
      m_enbHandoverStartOutFile  << "Timestamp" << " " << "imsi" << " " << "rnti" << " " <<  "sourceCellId" << " " << "targetCellId"<< std::endl;
    }
  m_enbHandoverEndOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << rnti << " " << targetCellId << std::endl;
}

void
MmWaveBearerStatsConnector::PrintUeStartHandover (uint64_t imsi, uint16_t sourceCellid, uint16_t targetCellId, uint16_t rnti)
{
  NS_LOG_FUNCTION ("NotifyHandoverStartUe " << Simulator::Now ().GetSeconds ());
  if (!m_ueHandoverStartOutFile.is_open ())
    {
      m_ueHandoverStartOutFile.open (GetUeHandoverStartOutputFilename ().c_str ());
      m_ueHandoverStartOutFile  << "Timestamp" << " " << "imsi" << " " << "rnti" << " " <<  "sourceCellId" << " " << "targetCellId"<< std::endl;
    }
  m_ueHandoverStartOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << rnti << " " << sourceCellid << " " << targetCellId << std::endl;
}

void
MmWaveBearerStatsConnector::PrintUeEndHandover (uint64_t imsi, uint16_t targetCellId, uint16_t rnti)
{
  NS_LOG_FUNCTION ("NotifyHandoverOkUe " << Simulator::Now ().GetSeconds ());
  if (!m_ueHandoverEndOutFile.is_open ())
    {
      m_ueHandoverEndOutFile.open (GetUeHandoverEndOutputFilename ().c_str ());
      m_ueHandoverEndOutFile<< "Timestamp" << " " << "imsi" << " " << "rnti" << " " << "targetCellId"<< std::endl;
    }
  m_ueHandoverEndOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << rnti << " " << targetCellId << std::endl;

  if (!m_cellIdInTimeHandoverOutFile.is_open ())
    {
      m_cellIdInTimeHandoverOutFile.open (GetCellIdStatsOutputFilename ().c_str ());
      m_cellIdInTimeHandoverOutFile<< "Timestamp" << " " << "imsi" << " " << "rnti" << " " << "targetCellId"<< std::endl;
    }
  m_cellIdInTimeHandoverOutFile << Simulator::Now ().GetNanoSeconds () / 1.0e9 << " " << imsi << " " << rnti << " " << targetCellId << std::endl;
}

void
MmWaveBearerStatsConnector::StoreUeManagerPath (std::string context, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context << cellId << rnti);
  std::ostringstream ueManagerPath;
  ueManagerPath <<  context.substr (0, context.rfind ("/")) << "/UeMap/" << (uint32_t) rnti;
  CellIdRnti key;
  key.cellId = cellId;
  key.rnti = rnti;
  NS_LOG_DEBUG(ueManagerPath.str());
  m_ueManagerPathByCellIdRnti[key] = ueManagerPath.str ();

  if(m_rlcStats)
  {
      Config::ConnectFailSafe (ueManagerPath.str () + "/SecondaryRlcCreated",
                 MakeBoundCallback (&NotifySecondaryMmWaveEnbAvailable, this));
  }

  // modified
  // connect the state transition trace
  Config::ConnectFailSafe (ueManagerPath.str () + "/StateTransition",
                 MakeBoundCallback (&NotifyEnbRrcStateTransition, this));
  // end modification
}

void
MmWaveBearerStatsConnector::ConnectSrb0Traces (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  std::string ueRrcPath =  context.substr (0, context.rfind ("/"));
  CellIdRnti key;
  key.cellId = cellId;
  key.rnti = rnti;
  std::map<CellIdRnti, std::string>::iterator it = m_ueManagerPathByCellIdRnti.find (key);
  NS_ASSERT (it != m_ueManagerPathByCellIdRnti.end ());
  std::string ueManagerPath = it->second;
  NS_LOG_LOGIC (this << " ueManagerPath: " << ueManagerPath);
  m_ueManagerPathByCellIdRnti.erase (it);

  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;

      // diconnect eventually previously connected SRB0 both at UE and eNB
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                          MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                          MakeBoundCallback (&DlRxPduCallback, arg));

      // modified
      // the buffer size of ue rlc
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                          MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
      // end modification

      Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                          MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                          MakeBoundCallback (&UlRxPduCallback, arg));
      // modified
      Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // adding the packet drop
      // Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/TxDrop",
      //                  MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // end modification

      // connect SRB0 both at UE and eNB
      Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      // modified
      Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                          MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
      // end modification
      Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      // modified
      Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // adding the packet drop
      // Config::Connect (ueManagerPath + "/Srb0/LteRlc/TxDrop",
      //                  MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // end modification

      // connect SRB1 at eNB only (at UE SRB1 will be setup later)
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));

    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        // diconnect eventually previously connected SRB0 both at UE and eNB
        Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                            MakeBoundCallback (&UlTxPduCallback, arg));
        Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                            MakeBoundCallback (&DlRxPduCallback, arg));

        // modified
        // the buffer size of ue rlc
        Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                            MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
        // end modification
        
        Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                            MakeBoundCallback (&DlTxPduCallback, arg));
        Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                            MakeBoundCallback (&UlRxPduCallback, arg));
        // modified
        Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/BufferSize",
                        MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
        // adding the packet drop
        // Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/TxDrop",
        //                  MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
        // end modification
  
        // connect SRB0 both at UE and eNB
        Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                         MakeBoundCallback (&UlTxPduCallback, arg));
        Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                         MakeBoundCallback (&DlRxPduCallback, arg));

        // modified
        Config::ConnectFailSafe (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                            MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
        // end modification
        
        Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
        Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
        // modified
        Config::ConnectFailSafe (ueManagerPath + "/Srb0/LteRlc/BufferSize",
                        MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
        // adding the packet drop
        // Config::Connect (ueManagerPath + "/Srb0/LteRlc/TxDrop",
        //                  MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
        // end modification
  
        // connect SRB1 at eNB only (at UE SRB1 will be setup later)
        Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
        Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      }
    }
  if (m_pdcpStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;

      // connect SRB1 at eNB only (at UE SRB1 will be setup later)
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (ueManagerPath + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
    }

  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
          Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
          arg->imsi = imsi;
          arg->cellId = cellId;
          arg->stats = e2PdcpStats;
    
          // connect SRB1 at eNB only (at UE SRB1 will be setup later)
          Config::ConnectFailSafe (ueManagerPath + "/Srb1/LtePdcp/RxPDU",
                           MakeBoundCallback (&UlRxPduCallback, arg));
          Config::ConnectFailSafe (ueManagerPath + "/Srb1/LtePdcp/TxPDU",
                           MakeBoundCallback (&DlTxPduCallback, arg));
        }
    }
}

void
MmWaveBearerStatsConnector::ConnectTracesUeIfFirstTime (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);

  //Connect PDCP and RLC traces for SRB1
  if (m_imsiSeenUeSrb.find (imsi) == m_imsiSeenUeSrb.end ())
    {
      m_imsiSeenUeSrb.insert (imsi);
      ConnectSrb1TracesUe (context, imsi, cellId, rnti);
    }

  std::string basePath = context.substr (0, context.rfind ("/"));
  Config::MatchContainer rlc_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LteRlc/");
  uint16_t numberOfRlc = rlc_container.GetN ();

  //Connect PDCP and RLC for data radio bearers
  std::map<uint64_t,uint16_t>::iterator it = m_imsiSeenUeDrb.find (imsi);
  if (m_imsiSeenUeDrb.find (imsi) == m_imsiSeenUeDrb.end () && rlc_container.GetN () > 0)
    {
      //If it is the first time for this imsi
      NS_LOG_DEBUG ("Insert imsi " + std::to_string (imsi));
      m_imsiSeenUeDrb.insert (m_imsiSeenUeDrb.end (), std::pair<uint64_t,uint16_t> (imsi, 1));
      ConnectDrbTracesUe (context, imsi, cellId, rnti);
    }
  else
    {
      if (it->second < numberOfRlc)
        {
          //If this imsi has already been connected but a new DRB is established
          NS_LOG_DEBUG ("There is a new RLC. Call ConnectDrbTracesUe to connect the traces.");
          it->second++; //TODO Check if there could be more than one RLC to connect
          DisconnectDrbTracesUe (context, imsi, cellId, rnti);
          ConnectDrbTracesUe (context, imsi, cellId, rnti);
        }
      else
        {
          //it->second = numberOfRlc; //One or more DRBs could have been removed
          NS_LOG_DEBUG ("All RLCs traces are already connected. No need for a call to ConnectDrbTracesUe.");
        }
    }
}


void
MmWaveBearerStatsConnector::ConnectTracesEnbIfFirstTime (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);

  //NB SRB1 traces are already connected

  //Connect PDCP and RLC for data radio bearers
  //Look for the RLCs
  std::string basePath = context.substr (0, context.rfind ("/")) + "/UeMap/" + std::to_string ((uint32_t) rnti);
  Config::MatchContainer rlc_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LteRlc/");

  if (m_imsiSeenEnbDrb.find (imsi) == m_imsiSeenEnbDrb.end () && rlc_container.GetN () > 0)
    {
      //it is executed only if there exist at least one rlc layer
      m_imsiSeenEnbDrb.insert (imsi);
      ConnectDrbTracesEnb (context, imsi, cellId, rnti);
    }
}



void
MmWaveBearerStatsConnector::ConnectDrbTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;

      m_rlcDrbDlRxCb[imsi] = MakeBoundCallback (&DlRxPduCallback, arg);
      m_rlcDrbUlTxCb[imsi] = MakeBoundCallback (&UlTxPduCallback, arg);

      Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                       m_rlcDrbUlTxCb.at (imsi));
      Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                       m_rlcDrbDlRxCb.at (imsi));

    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        m_rlcDrbDlRxCb[imsi] = MakeBoundCallback (&DlRxPduCallback, arg);
        m_rlcDrbUlTxCb[imsi] = MakeBoundCallback (&UlTxPduCallback, arg);

        Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                         m_rlcDrbUlTxCb.at (imsi));
        Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                         m_rlcDrbDlRxCb.at (imsi));
      }
    }
  if (m_pdcpStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;

      m_pdcpDrbDlRxCb[imsi] = MakeBoundCallback (&DlRxPduCallback, arg);
      m_pdcpDrbUlTxCb[imsi] = MakeBoundCallback (&UlTxPduCallback, arg);

      Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                       m_pdcpDrbDlRxCb.at (imsi));
      Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                       m_pdcpDrbUlTxCb.at (imsi));
    }
  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
          Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
          arg->imsi = imsi;
          arg->cellId = cellId;
          arg->stats = e2PdcpStats;
    
          m_pdcpDrbDlRxCb[imsi] = MakeBoundCallback (&DlRxPduCallback, arg);
          m_pdcpDrbUlTxCb[imsi] = MakeBoundCallback (&UlTxPduCallback, arg);

          Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                           m_pdcpDrbDlRxCb.at (imsi));
          Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                           m_pdcpDrbUlTxCb.at (imsi));
        }
    }
}

void
MmWaveBearerStatsConnector::ConnectSrb1TracesUe (std::string ueRrcPath, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << ueRrcPath);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = ueRrcPath.substr (0, ueRrcPath.rfind ("/"));
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      // modified
      Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
      // end modification
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        m_rlcDrbDlRxCb[imsi] = MakeBoundCallback (&DlRxPduCallback, arg);
        m_rlcDrbUlTxCb[imsi] = MakeBoundCallback (&UlTxPduCallback, arg);

        Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/TxPDU",
                         MakeBoundCallback (&UlTxPduCallback, arg));
        Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/RxPDU",
                         MakeBoundCallback (&DlRxPduCallback, arg));
        // modified
        Config::ConnectFailSafe (basePath + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                        MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
        Config::ConnectFailSafe (basePath + "/Srb0/LteRlc/BufferSize",
                        MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
        Config::ConnectFailSafe (basePath + "/Srb1/LteRlc/BufferSize",
                        MakeBoundCallback (&UeRlcBufferSizeCallback, arg));
        // end modification
      }
    }
  if (m_pdcpStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::ConnectFailSafe (basePath + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      Config::ConnectFailSafe (basePath + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
    }
  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
          Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
          arg->imsi = imsi;
          arg->cellId = cellId;
          arg->stats = e2PdcpStats;
    
          Config::ConnectFailSafe (basePath + "/Srb1/LtePdcp/RxPDU",
                           MakeBoundCallback (&DlRxPduCallback, arg));
          Config::ConnectFailSafe (basePath + "/Srb1/LtePdcp/TxPDU",
                           MakeBoundCallback (&UlTxPduCallback, arg));
        }
    }
  if (m_mcStats)
    {
      Ptr<McMmWaveBoundCallbackArgument> arg = Create<McMmWaveBoundCallbackArgument> ();
      arg->stats = m_mcStats;
      Config::ConnectFailSafe (basePath + "/SwitchToLte",
                       MakeBoundCallback (&SwitchToLteCallback, arg));
      Config::ConnectFailSafe (basePath + "/SwitchToMmWave",
                       MakeBoundCallback (&SwitchToMmWaveCallback, arg));
    }
}


void
MmWaveBearerStatsConnector::ConnectSrb1TracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context  should match /NodeList/*/DeviceList/*/LteEnbRrc/");
  std::ostringstream basePath;
  basePath <<  context.substr (0, context.rfind ("/")) << "/UeMap/" << (uint32_t) rnti;
  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      // modified
      // the rlc buffer
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // end modification
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
        Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
        Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
        Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
        // modified
      // the rlc buffer
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // end modification
      }
    }
  if (m_pdcpStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
    }
  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
          Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
          arg->imsi = imsi;
          arg->cellId = cellId;
          arg->stats = e2PdcpStats;
    
          Config::ConnectFailSafe (basePath.str () + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
          Config::ConnectFailSafe (basePath.str () + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
        }
    }
}

void
MmWaveBearerStatsConnector::ConnectDrbTracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context  should match /NodeList/*/DeviceList/*/LteEnbRrc/");
  std::ostringstream basePath;
  basePath <<  context.substr (0, context.rfind ("/")) << "/UeMap/" << (uint32_t) rnti;
  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      // modified
      // the rlc buffer
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                       MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
      // end modification
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
        Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
        // modified
        // the rlc buffer
        Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                        MakeBoundCallback (&GnbRlcBufferSizeCallback, arg));
        // end modification
      }
    }
  if (m_pdcpStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
    }
  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
          Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
          arg->imsi = imsi;
          arg->cellId = cellId;
          arg->stats = e2PdcpStats;
    
          Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
          Config::ConnectFailSafe (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
        }
    }
}

void
MmWaveBearerStatsConnector::DisconnectTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  Config::MatchContainer objects = Config::LookupMatches (basePath + "/DataRadioBearerMap/*/LteRlc/");
  NS_LOG_LOGIC ("basePath " << basePath);

  if (m_mcStats)
    {
      Ptr<McMmWaveBoundCallbackArgument> arg = Create<McMmWaveBoundCallbackArgument> ();
      arg->stats = m_mcStats;
      Config::Disconnect (basePath + "/SwitchToLte",
                          MakeBoundCallback (&SwitchToLteCallback, arg));
      Config::Disconnect (basePath + "/SwitchToMmWave",
                          MakeBoundCallback (&SwitchToMmWaveCallback, arg));
    }
}

void
MmWaveBearerStatsConnector::DisconnectDrbTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  NS_LOG_LOGIC ("basePath " << basePath);

  if (m_rlcStats)
    {
      Config::MatchContainer rlc_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LteRlc/");
      NS_LOG_LOGIC ("Number of RLC to disconnect " << rlc_container.GetN ());

      rlc_container.Disconnect ("RxPDU",m_rlcDrbDlRxCb.at (imsi));
      rlc_container.Disconnect ("TxPDU",m_rlcDrbUlTxCb.at (imsi));
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Config::MatchContainer rlc_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LteRlc/");
        NS_LOG_LOGIC ("Number of RLC to disconnect " << rlc_container.GetN ());

        rlc_container.Disconnect ("RxPDU",m_rlcDrbDlRxCb.at (imsi));
        rlc_container.Disconnect ("TxPDU",m_rlcDrbUlTxCb.at (imsi));
      }
    }

  if (m_pdcpStats)
    {
      Config::MatchContainer pdcp_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LtePdcp/");
      NS_LOG_LOGIC ("Number of PDCP to disconnect " << pdcp_container.GetN ());

      pdcp_container.Disconnect ("RxPDU",m_pdcpDrbDlRxCb.at (imsi));
      pdcp_container.Disconnect ("TxPDU",m_pdcpDrbUlTxCb.at (imsi));
    }
  if (!m_e2PdcpStatsVector.empty())
    {
      for (auto e2PdcpStats : m_e2PdcpStatsVector) {
        Config::MatchContainer pdcp_container = Config::LookupMatches (basePath +  "/DataRadioBearerMap/*/LtePdcp/");
        NS_LOG_LOGIC ("Number of PDCP to disconnect " << pdcp_container.GetN ());

        pdcp_container.Disconnect ("RxPDU",m_pdcpDrbDlRxCb.at (imsi));
        pdcp_container.Disconnect ("TxPDU",m_pdcpDrbUlTxCb.at (imsi));
      }
    }
}


void
MmWaveBearerStatsConnector::DisconnectTracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
}


void
MmWaveBearerStatsConnector::ConnectSecondaryTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_LOGIC (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  Config::MatchContainer objects = Config::LookupMatches (basePath + "/DataRadioRlcMap/*");
  NS_LOG_LOGIC ("basePath " << basePath);

  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      // for MC devices
      Config::ConnectFailSafe (basePath + "/DataRadioRlcMap/*/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::ConnectFailSafe (basePath + "/DataRadioRlcMap/*/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        // for MC devices
        Config::ConnectFailSafe (basePath + "/DataRadioRlcMap/*/LteRlc/TxPDU",
                         MakeBoundCallback (&UlTxPduCallback, arg));
        Config::ConnectFailSafe (basePath + "/DataRadioRlcMap/*/LteRlc/RxPDU",
                         MakeBoundCallback (&DlRxPduCallback, arg));
      }
    }
}

void
MmWaveBearerStatsConnector::ConnectSecondaryTracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*");
  std::ostringstream basePath;
  basePath <<  context.substr (0, context.rfind ("/"));
  Config::MatchContainer objects = Config::LookupMatches (basePath.str () + "/DataRadioRlcMap/*/LteRlc/");
  NS_LOG_LOGIC ("basePath " << basePath.str ());

  if (m_rlcStats)
    {
      Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      // for MC devices
      Config::ConnectFailSafe (basePath.str () + "/DataRadioRlcMap/*/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::ConnectFailSafe (basePath.str () + "/DataRadioRlcMap/*/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
    }
  if (!m_e2RlcStatsVector.empty())
    {
      for(auto rlcStats : m_e2RlcStatsVector)
      {
        Ptr<MmWaveBoundCallbackArgument> arg = Create<MmWaveBoundCallbackArgument> ();
        arg->imsi = imsi;
        arg->cellId = cellId;
        arg->stats = rlcStats;
  
        // for MC devices
        Config::ConnectFailSafe (basePath.str () + "/DataRadioRlcMap/*/LteRlc/RxPDU",
                         MakeBoundCallback (&UlRxPduCallback, arg));
        Config::ConnectFailSafe (basePath.str () + "/DataRadioRlcMap/*/LteRlc/TxPDU",
                         MakeBoundCallback (&DlTxPduCallback, arg));
      }
    }
}

} // namespace mmwave

} // namespace ns3
