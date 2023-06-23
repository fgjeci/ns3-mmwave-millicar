/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*   Copyright (c) 2016, University of Padova, Dep. of Information Engineering, SIGNET lab.
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
*/


#include "mmwave-bearer-stats-calculator.h"
#include "ns3/string.h"
#include "ns3/nstime.h"
#include <ns3/boolean.h>
#include <ns3/log.h>
#include <vector>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveBearerStatsCalculator");

namespace mmwave {

NS_OBJECT_ENSURE_REGISTERED ( MmWaveBearerStatsCalculator);

MmWaveBearerStatsCalculator::MmWaveBearerStatsCalculator ()
  : m_firstWrite (true),
    m_pendingOutput (false),
    m_aggregatedStats (false),
    m_protocolType ("RLC")
{
  NS_LOG_FUNCTION (this);
}

MmWaveBearerStatsCalculator::MmWaveBearerStatsCalculator (std::string protocolType)
  : m_firstWrite (true),
    m_pendingOutput (false),
    m_aggregatedStats (false)
{
  NS_LOG_FUNCTION (this);
  m_protocolType = protocolType;
}

MmWaveBearerStatsCalculator::~MmWaveBearerStatsCalculator ()
{
  NS_LOG_FUNCTION (this);
  // modified
  // close the file of trace
  if (m_ulOutFile.is_open ())
    {
      m_ulOutFile.close ();
    }
  if (m_dlOutFile.is_open ())
    {
      m_dlOutFile.close ();
    }
  if(m_gnbRlcBufferSizeFile.is_open()){
    m_gnbRlcBufferSizeFile.close();
  }
  if(m_ueRlcBufferSizeFile.is_open()){
    m_ueRlcBufferSizeFile.close();
  }
  // end modification
}

TypeId
MmWaveBearerStatsCalculator::GetTypeId (void)
{
  static TypeId tid =
    TypeId ("ns3::MmWaveBearerStatsCalculator")
    .SetParent<LteStatsCalculator> ().AddConstructor<MmWaveBearerStatsCalculator> ()
    .SetGroupName ("Lte")
    .AddAttribute ("StartTime", "Start time of the on going epoch.",
                   TimeValue (Seconds (0.)),
                   MakeTimeAccessor (&MmWaveBearerStatsCalculator::SetStartTime,
                                     &MmWaveBearerStatsCalculator::GetStartTime),
                   MakeTimeChecker ())
    .AddAttribute ("EpochDuration", "Epoch duration.",
                   TimeValue (Seconds (0.25)),
                   MakeTimeAccessor (&MmWaveBearerStatsCalculator::GetEpoch,
                                     &MmWaveBearerStatsCalculator::SetEpoch),
                   MakeTimeChecker ())
    .AddAttribute ("DlRlcOutputFilename",
                   "Name of the file where the downlink results will be saved.",
                   StringValue ("DlRlcStats.txt"),
                   MakeStringAccessor (&LteStatsCalculator::SetDlOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UlRlcOutputFilename",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("UlRlcStats.txt"),
                   MakeStringAccessor (&LteStatsCalculator::SetUlOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("DlPdcpOutputFilename",
                   "Name of the file where the downlink results will be saved.",
                   StringValue ("DlPdcpStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsCalculator::SetDlPdcpOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UlPdcpOutputFilename",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("UlPdcpStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsCalculator::SetUlPdcpOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("AggregatedStats",
                   "Choice to show the results aggregated of disaggregated.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MmWaveBearerStatsCalculator::m_aggregatedStats),
                   MakeBooleanChecker ())
    // modified
    // added
    .AddAttribute ("GnbRlcBufferSizeOutputFilename",
                   "Name of the file where the gnb rlc buffer size results will be saved.",
                   StringValue ("GnbRlcBufferSizeStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsCalculator::SetGnbRlcBufferSizeFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeRlcBufferSizeOutputFilename",
                   "Name of the file where the ue rlc buffer size results will be saved.",
                   StringValue ("UeRlcBufferSizeStats.txt"),
                   MakeStringAccessor (&MmWaveBearerStatsCalculator::SetUeRlcBufferSizeFilename),
                   MakeStringChecker ())
    // modified
    .AddAttribute ("TracesPath",
                    "The path where to store the path. ",
                    StringValue ("./"),
                    MakeStringAccessor (&MmWaveBearerStatsCalculator::m_tracesPath),
                    MakeStringChecker ())
                            
    // end modification
  ;
  return tid;
}

void
MmWaveBearerStatsCalculator::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  if (m_pendingOutput)
    {
      ShowResults ();
    }
}

void
MmWaveBearerStatsCalculator::SetStartTime (Time t)
{
  m_startTime = t;
  if (m_aggregatedStats)
  {
    RescheduleEndEpoch ();
  }
}

Time
MmWaveBearerStatsCalculator::GetStartTime () const
{
  return m_startTime;
}

void
MmWaveBearerStatsCalculator::SetEpoch (Time e)
{
  m_epochDuration = e;
  if (m_aggregatedStats)
  {
    RescheduleEndEpoch ();
  }
}

Time
MmWaveBearerStatsCalculator::GetEpoch () const
{
  return m_epochDuration;
}

void
MmWaveBearerStatsCalculator::UlTxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (this << "UlTxPdu" << cellId << imsi << rnti << (uint32_t) lcid << packetSize);
  if(m_aggregatedStats)
  {
    ImsiLcidPair_t p (imsi, lcid);
    if (Simulator::Now () >= m_startTime)
      {
        m_ulCellId[p] = cellId;
        m_flowId[p] = LteFlowId_t (rnti, lcid);
        m_ulTxPackets[p]++;
        m_ulTxData[p] += packetSize;
      }
    m_pendingOutput = true;
  }
  else
  {
    if (!m_ulOutFile.is_open ())
      {
        m_ulOutFile.open (GetUlOutputFilename ().c_str ());
        m_ulOutFile<< "RLC_PDCP\t" << "TYPE\tTIME\tCellId\tIMSI\tRNTI\tLCID\tSIZE\tDELAY\t" << std::endl;
      }
    if (m_protocolType == "RLC")
    {
      m_ulOutFile << "RLC\t";
    }
  else
    {
      m_ulOutFile << "PDCP\t";
    }
    m_ulOutFile << "UlTx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << 0 << "\t" << std::endl;
  }
}

void
MmWaveBearerStatsCalculator::DlTxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (this << "DlTxPDU" << cellId << imsi << rnti << (uint32_t) lcid << packetSize);

  if(m_aggregatedStats)
  {
    ImsiLcidPair_t p (imsi, lcid);
    if (Simulator::Now () >= m_startTime)
      {
        m_dlCellId[p] = cellId;
        m_flowId[p] = LteFlowId_t (rnti, lcid);
        m_dlTxPackets[p]++;
        m_dlTxData[p] += packetSize;
      }
    m_pendingOutput = true;
  }            
  else
  {
    if (!m_dlOutFile.is_open ())
      {
        m_dlOutFile.open (GetDlOutputFilename ().c_str ());
        m_dlOutFile << "RLC_PDCP\t" << "TYPE\tTIME\tCellId\tIMSI\tRNTI\tLCID\tSIZE\tDELAY\t" << std::endl;
      }
    if (m_protocolType == "RLC")
    {
      m_dlOutFile << "RLC\t";
    }
  else
    {
      m_dlOutFile << "PDCP\t";
    }
    m_dlOutFile << "DlTx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << 0 << "\t" << std::endl;
  }
}

void
MmWaveBearerStatsCalculator::UlRxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti, uint8_t lcid, uint32_t packetSize,
                                      uint64_t delay)
{
  NS_LOG_FUNCTION (this << "UlRxPDU" << cellId << imsi << rnti << (uint32_t) lcid << packetSize << delay);
  if(m_aggregatedStats)
  {
    ImsiLcidPair_t p (imsi, lcid);
    if (Simulator::Now () >= m_startTime)
      {
        m_ulCellId[p] = cellId;
        m_ulRxPackets[p]++;
        m_ulRxData[p] += packetSize;

        Uint64StatsMap::iterator it = m_ulDelay.find (p);
        if (it == m_ulDelay.end ())
          {
            NS_LOG_DEBUG (this << " Creating UL stats calculators for IMSI " << p.m_imsi << " and LCID " << (uint32_t) p.m_lcId);
            m_ulDelay[p] = CreateObject<MinMaxAvgTotalCalculator<uint64_t> > ();
            m_ulPduSize[p] = CreateObject<MinMaxAvgTotalCalculator<uint32_t> > ();
          }
        m_ulDelay[p]->Update (delay);
        m_ulPduSize[p]->Update (packetSize);
      }
    m_pendingOutput = true;
  }
  else
  {
    if (!m_ulOutFile.is_open ())
      {
        m_ulOutFile.open (GetUlOutputFilename ().c_str ());
        m_ulOutFile <<"RLC_PDCP\t" <<   "TYPE\tTIME\tCellId\tIMSI\tRNTI\tLCID\tSIZE\tDELAY\t" << std::endl;
      }
    if (m_protocolType == "RLC")
    {
      m_ulOutFile << "RLC\t";
    }
  else
    {
      m_ulOutFile << "PDCP\t";
    }
    m_ulOutFile << "UlRx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << delay << "\t" << std::endl;

    NS_LOG_DEBUG ("Rx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << delay << "\t" << std::endl);
  }
}

void
MmWaveBearerStatsCalculator::DlRxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
{
  NS_LOG_FUNCTION (this << "DlRxPDU" << cellId << imsi << rnti << (uint32_t) lcid << packetSize << delay);

  if(m_aggregatedStats)
  {
    ImsiLcidPair_t p (imsi, lcid);
    if (Simulator::Now () >= m_startTime)
    {
      m_dlCellId[p] = cellId;
      m_dlRxPackets[p]++;
      m_dlRxData[p] += packetSize;
      
      Uint64StatsMap::iterator it = m_dlDelay.find (p);
      if (it == m_dlDelay.end ())
      {
        NS_LOG_DEBUG (this << " Creating DL stats calculators for IMSI " << p.m_imsi << " and LCID " << (uint32_t) p.m_lcId);
        m_dlDelay[p] = CreateObject<MinMaxAvgTotalCalculator<uint64_t> > ();
        m_dlPduSize[p] = CreateObject<MinMaxAvgTotalCalculator<uint32_t> > ();
      }
      m_dlDelay[p]->Update (delay);
      m_dlPduSize[p]->Update (packetSize);
    }
    m_pendingOutput = true;
  }
  else
  {
    if (!m_dlOutFile.is_open ())
    {
      m_dlOutFile.open (GetDlOutputFilename ().c_str ());
      m_dlOutFile << "RLC_PDCP\t" << "TYPE\tTIME\tCellId\tIMSI\tRNTI\tLCID\tSIZE\tDELAY\t" << std::endl;
    }
    if (m_protocolType == "RLC")
    {
      m_dlOutFile << "RLC\t";
    }
  else
    {
      m_dlOutFile << "PDCP\t";
    }
    m_dlOutFile << "DlRx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << delay << "\t" << std::endl;
    NS_LOG_DEBUG ("Rx\t" << Simulator::Now ().GetNanoSeconds () / 1.0e9 << "\t" 
    << cellId << "\t" << imsi << "\t" << rnti << "\t" << (uint32_t) lcid << "\t" 
    << packetSize << "\t" << delay << "\t" << std::endl);
  }
}

// modified
void
MmWaveBearerStatsCalculator::GnbRlcBufferSize (uint16_t cellId, uint32_t bufferSize, uint32_t maxBufferSize)
{
  NS_LOG_FUNCTION (this << " cell " << cellId << " buff size "  << bufferSize << " max size " << maxBufferSize);

  if (!m_gnbRlcBufferSizeFile.is_open ())
    {
      m_gnbRlcBufferSizeFile.open (m_gnbRlcbufferSizeOutputFilename.c_str ()); // , std::ios_base::app
      m_gnbRlcBufferSizeFile << "CellId" << " " <<  "RlcBufferSize" << " " << "RlcMaxBufferSize"<< " " << "Timestamp" << std::endl;
    }
  m_gnbRlcBufferSizeFile <<  cellId << " " << bufferSize << " " << maxBufferSize << " " << Simulator::Now () << std::endl;
}

// ue buffer size

void
MmWaveBearerStatsCalculator::UeRlcBufferSize (uint16_t cellId, uint64_t imsi, uint32_t bufferSize, uint32_t maxBufferSize)
{
  NS_LOG_FUNCTION (this << " cell " << cellId << " buff size "  << bufferSize << " max size " << maxBufferSize);

  if (!m_ueRlcBufferSizeFile.is_open ())
    {
      m_ueRlcBufferSizeFile.open (m_ueRlcbufferSizeOutputFilename.c_str ()); // , std::ios_base::app
      m_ueRlcBufferSizeFile << "CellId"  << " " << "Imsi"  << " " <<  "RlcBufferSize" << " " << "RlcMaxBufferSize"<< " " << "Timestamp" << std::endl;
    }
  m_ueRlcBufferSizeFile << cellId << " " << imsi << " " << bufferSize << " " << maxBufferSize << " " << Simulator::Now () << std::endl;
}

// end modification

void
MmWaveBearerStatsCalculator::ShowResults (void)
{

  NS_LOG_FUNCTION (this << GetUlOutputFilename ().c_str () << GetDlOutputFilename ().c_str ());
  NS_LOG_INFO ("Write stats in " << GetUlOutputFilename ().c_str () << " and in " << GetDlOutputFilename ().c_str ());

  std::ofstream ulOutFile;
  std::ofstream dlOutFile;

  if (m_firstWrite == true)
    {
      ulOutFile.open (GetUlOutputFilename ().c_str ());
      if (!ulOutFile.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << GetUlOutputFilename ().c_str ());
          return;
        }

      dlOutFile.open (GetDlOutputFilename ().c_str ());
      if (!dlOutFile.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << GetDlOutputFilename ().c_str ());
          return;
        }
      m_firstWrite = false;
      ulOutFile << "% start\tend\tCellId\tIMSI\tRNTI\tLCID\tnTxPDUs\tTxBytes\tnRxPDUs\tRxBytes\t";
      ulOutFile << "delay\tstdDev\tmin\tmax\t";
      ulOutFile << "PduSize\tstdDev\tmin\tmax";
      ulOutFile << std::endl;
      dlOutFile << "% start\tend\tCellId\tIMSI\tRNTI\tLCID\tnTxPDUs\tTxBytes\tnRxPDUs\tRxBytes\t";
      dlOutFile << "delay\tstdDev\tmin\tmax\t";
      dlOutFile << "PduSize\tstdDev\tmin\tmax";
      dlOutFile << std::endl;
    }
  else
    {
      ulOutFile.open (GetUlOutputFilename ().c_str (), std::ios_base::app);
      if (!ulOutFile.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << GetUlOutputFilename ().c_str ());
          return;
        }

      dlOutFile.open (GetDlOutputFilename ().c_str (), std::ios_base::app);
      if (!dlOutFile.is_open ())
        {
          NS_LOG_ERROR ("Can't open file " << GetDlOutputFilename ().c_str ());
          return;
        }
    }

  WriteUlResults (ulOutFile);
  WriteDlResults (dlOutFile);
  m_pendingOutput = false;

}

void
MmWaveBearerStatsCalculator::WriteUlResults (std::ofstream& outFile)
{
  NS_LOG_FUNCTION (this);

  // Get the unique IMSI / LCID list

  std::vector < ImsiLcidPair_t > pairVector;
  for (Uint32Map::iterator it = m_ulTxPackets.begin (); it != m_ulTxPackets.end (); ++it)
    {
      if (find (pairVector.begin (), pairVector.end (), (*it).first) == pairVector.end ())
        {
          pairVector.push_back ((*it).first);
        }
    }

  Time endTime = m_startTime + m_epochDuration;
  for (std::vector<ImsiLcidPair_t>::iterator it = pairVector.begin (); it != pairVector.end (); ++it)
    {
      ImsiLcidPair_t p = *it;
      outFile << m_startTime.GetNanoSeconds () / 1.0e9 << "\t";
      outFile << endTime.GetNanoSeconds () / 1.0e9 << "\t";
      outFile << GetUlCellId (p.m_imsi, p.m_lcId) << "\t";
      outFile << p.m_imsi << "\t";
      outFile << m_flowId[p].m_rnti << "\t";
      outFile << (uint32_t) m_flowId[p].m_lcId << "\t";
      outFile << GetUlTxPackets (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetUlTxData (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetUlRxPackets (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetUlRxData (p.m_imsi, p.m_lcId) << "\t";
      std::vector<double> stats = GetUlDelayStats (p.m_imsi, p.m_lcId);
      for (std::vector<double>::iterator it = stats.begin (); it != stats.end (); ++it)
        {
          outFile << (*it) * 1e-9 << "\t";
        }
      stats = GetUlPduSizeStats (p.m_imsi, p.m_lcId);
      for (std::vector<double>::iterator it = stats.begin (); it != stats.end (); ++it)
        {
          outFile << (*it) << "\t";
        }
      outFile << std::endl;
    }

  outFile.close ();
}

void
MmWaveBearerStatsCalculator::WriteDlResults (std::ofstream& outFile)
{
  NS_LOG_FUNCTION (this);

  // Get the unique IMSI list
  std::vector < ImsiLcidPair_t > pairVector;
  for (Uint32Map::iterator it = m_dlTxPackets.begin (); it != m_dlTxPackets.end (); ++it)
    {
      if (find (pairVector.begin (), pairVector.end (), (*it).first) == pairVector.end ())
        {
          pairVector.push_back ((*it).first);
        }
    }

  Time endTime = m_startTime + m_epochDuration;
  for (std::vector<ImsiLcidPair_t>::iterator pair = pairVector.begin (); pair != pairVector.end (); ++pair)
    {
      ImsiLcidPair_t p = *pair;
      outFile << m_startTime.GetNanoSeconds () / 1.0e9 << "\t";
      outFile << endTime.GetNanoSeconds () / 1.0e9 << "\t";
      outFile << GetDlCellId (p.m_imsi, p.m_lcId) << "\t";
      outFile << p.m_imsi << "\t";
      outFile << m_flowId[p].m_rnti << "\t";
      outFile << (uint32_t) m_flowId[p].m_lcId << "\t";
      outFile << GetDlTxPackets (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetDlTxData (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetDlRxPackets (p.m_imsi, p.m_lcId) << "\t";
      outFile << GetDlRxData (p.m_imsi, p.m_lcId) << "\t";
      std::vector<double> stats = GetDlDelayStats (p.m_imsi, p.m_lcId);
      for (std::vector<double>::iterator it = stats.begin (); it != stats.end (); ++it)
        {
          outFile << (*it) * 1e-9 << "\t";
        }
      stats = GetDlPduSizeStats (p.m_imsi, p.m_lcId);
      for (std::vector<double>::iterator it = stats.begin (); it != stats.end (); ++it)
        {
          outFile << (*it) << "\t";
        }
      outFile << std::endl;
    }

  outFile.close ();
}

void
MmWaveBearerStatsCalculator::ResetResults (void)
{
  NS_LOG_FUNCTION (this);

  m_ulTxPackets.erase (m_ulTxPackets.begin (), m_ulTxPackets.end ());
  m_ulRxPackets.erase (m_ulRxPackets.begin (), m_ulRxPackets.end ());
  m_ulRxData.erase (m_ulRxData.begin (), m_ulRxData.end ());
  m_ulTxData.erase (m_ulTxData.begin (), m_ulTxData.end ());
  m_ulDelay.erase (m_ulDelay.begin (), m_ulDelay.end ());
  m_ulPduSize.erase (m_ulPduSize.begin (), m_ulPduSize.end ());

  m_dlTxPackets.erase (m_dlTxPackets.begin (), m_dlTxPackets.end ());
  m_dlRxPackets.erase (m_dlRxPackets.begin (), m_dlRxPackets.end ());
  m_dlRxData.erase (m_dlRxData.begin (), m_dlRxData.end ());
  m_dlTxData.erase (m_dlTxData.begin (), m_dlTxData.end ());
  m_dlDelay.erase (m_dlDelay.begin (), m_dlDelay.end ());
  m_dlPduSize.erase (m_dlPduSize.begin (), m_dlPduSize.end ());
}


void
MmWaveBearerStatsCalculator::ResetResultsForImsiLcid (uint64_t imsi, uint16_t lcid)
{
  NS_LOG_FUNCTION (this);

  auto ulTxPacketsEntry = m_ulTxPackets.find(ImsiLcidPair_t(imsi, lcid));
  if(ulTxPacketsEntry != m_ulTxPackets.end())
  {
    m_ulTxPackets.erase (ulTxPacketsEntry);
  }
  auto ulRxPacketsEntry = m_ulRxPackets.find(ImsiLcidPair_t(imsi, lcid));
  if(ulRxPacketsEntry != m_ulRxPackets.end())
  {
    m_ulRxPackets.erase (ulRxPacketsEntry);
  }
  auto ulRxDataEntry = m_ulRxData.find(ImsiLcidPair_t(imsi, lcid));
  if(ulRxDataEntry != m_ulRxData.end())
  {
    m_ulRxData.erase (ulRxDataEntry);
  }
  auto ulTxDataEntry = m_ulTxData.find(ImsiLcidPair_t(imsi, lcid));
  if(ulTxDataEntry != m_ulTxData.end())
  {
    m_ulTxData.erase (ulTxDataEntry);
  }
  auto ulDelayEntry = m_ulDelay.find(ImsiLcidPair_t(imsi, lcid));
  if(ulDelayEntry != m_ulDelay.end())
  {
    m_ulDelay.erase (ulDelayEntry);
  }
  auto ulPduSizeEntry = m_ulPduSize.find(ImsiLcidPair_t(imsi, lcid));
  if(ulPduSizeEntry != m_ulPduSize.end())
  {
    m_ulPduSize.erase (ulPduSizeEntry);
  }

  auto dlTxPacketsEntry = m_dlTxPackets.find(ImsiLcidPair_t(imsi, lcid));
  if(dlTxPacketsEntry != m_dlTxPackets.end())
  {
    m_dlTxPackets.erase (dlTxPacketsEntry);
  }
  auto dlRxPacketsEntry = m_dlRxPackets.find(ImsiLcidPair_t(imsi, lcid));
  if(dlRxPacketsEntry != m_dlRxPackets.end())
  {
    m_dlRxPackets.erase (dlRxPacketsEntry);
  }
  auto dlRxDatEntry = m_dlRxData.find(ImsiLcidPair_t(imsi, lcid));
  if(dlRxDatEntry != m_dlRxData.end())
  {
    m_dlRxData.erase (dlRxDatEntry);
  }
  auto dlTxDataEntry = m_dlTxData.find(ImsiLcidPair_t(imsi, lcid));
  if(dlTxDataEntry != m_dlTxData.end())
  {
    m_dlTxData.erase (dlTxDataEntry);
  }
  auto dlDelayEntry = m_dlDelay.find(ImsiLcidPair_t(imsi, lcid));
  if(dlDelayEntry != m_dlDelay.end())
  {
    m_dlDelay.erase (dlDelayEntry);
  }
  auto dlPduSizeEntry = m_dlPduSize.find(ImsiLcidPair_t(imsi, lcid));
  if(dlPduSizeEntry != m_dlPduSize.end())
  {
    m_dlPduSize.erase (dlPduSizeEntry);
  }
}

void
MmWaveBearerStatsCalculator::RescheduleEndEpoch (void)
{
  NS_LOG_FUNCTION (this);
  m_endEpochEvent.Cancel ();
  NS_ASSERT (Simulator::Now ().GetMilliSeconds () == 0); // below event time assumes this
  m_endEpochEvent = Simulator::Schedule (m_startTime + m_epochDuration, &MmWaveBearerStatsCalculator::EndEpoch, this);
}

void
MmWaveBearerStatsCalculator::EndEpoch (void)
{
  NS_LOG_FUNCTION (this);
  ShowResults ();
  ResetResults ();
  m_startTime += m_epochDuration;
  m_endEpochEvent = Simulator::Schedule (m_epochDuration, &MmWaveBearerStatsCalculator::EndEpoch, this);
}

uint32_t
MmWaveBearerStatsCalculator::GetUlTxPackets (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_ulTxPackets[p];
}

uint32_t
MmWaveBearerStatsCalculator::GetUlRxPackets (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_ulRxPackets[p];
}

uint64_t
MmWaveBearerStatsCalculator::GetUlTxData (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_ulTxData[p];
}

uint64_t
MmWaveBearerStatsCalculator::GetUlRxData (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_ulRxData[p];
}

double
MmWaveBearerStatsCalculator::GetUlDelay (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  Uint64StatsMap::iterator it = m_ulDelay.find (p);
  if (it == m_ulDelay.end ())
    {
      NS_LOG_ERROR ("UL delay for " << imsi << " - " << (uint16_t) lcid << " not found");
      return 0;

    }
  return m_ulDelay[p]->getMean ();
}

std::vector<double>
MmWaveBearerStatsCalculator::GetUlDelayStats (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  std::vector<double> stats;
  Uint64StatsMap::iterator it = m_ulDelay.find (p);
  if (it == m_ulDelay.end ())
    {
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      return stats;

    }
  stats.push_back (m_ulDelay[p]->getMean ());
  stats.push_back (m_ulDelay[p]->getStddev ());
  stats.push_back (m_ulDelay[p]->getMin ());
  stats.push_back (m_ulDelay[p]->getMax ());
  return stats;
}

std::vector<double>
MmWaveBearerStatsCalculator::GetUlPduSizeStats (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  std::vector<double> stats;
  Uint32StatsMap::iterator it = m_ulPduSize.find (p);
  if (it == m_ulPduSize.end ())
    {
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      return stats;

    }
  stats.push_back (m_ulPduSize[p]->getMean ());
  stats.push_back (m_ulPduSize[p]->getStddev ());
  stats.push_back (m_ulPduSize[p]->getMin ());
  stats.push_back (m_ulPduSize[p]->getMax ());
  return stats;
}

uint32_t
MmWaveBearerStatsCalculator::GetDlTxPackets (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_dlTxPackets[p];
}

uint32_t
MmWaveBearerStatsCalculator::GetDlRxPackets (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_dlRxPackets[p];
}

uint64_t
MmWaveBearerStatsCalculator::GetDlTxData (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_dlTxData[p];
}

uint64_t
MmWaveBearerStatsCalculator::GetDlRxData (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_dlRxData[p];
}

uint32_t
MmWaveBearerStatsCalculator::GetUlCellId (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_ulCellId[p];
}

uint32_t
MmWaveBearerStatsCalculator::GetDlCellId (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  return m_dlCellId[p];
}

double
MmWaveBearerStatsCalculator::GetDlDelay (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  Uint64StatsMap::iterator it = m_dlDelay.find (p);
  if (it == m_dlDelay.end ())
    {
      NS_LOG_ERROR ("DL delay for " << imsi << " not found");
      return 0;
    }
  return m_dlDelay[p]->getMean ();
}

std::vector<double>
MmWaveBearerStatsCalculator::GetDlDelayStats (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  std::vector<double> stats;
  Uint64StatsMap::iterator it = m_dlDelay.find (p);
  if (it == m_dlDelay.end ())
    {
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      return stats;

    }
  stats.push_back (m_dlDelay[p]->getMean ());
  stats.push_back (m_dlDelay[p]->getStddev ());
  stats.push_back (m_dlDelay[p]->getMin ());
  stats.push_back (m_dlDelay[p]->getMax ());
  return stats;
}

std::vector<double>
MmWaveBearerStatsCalculator::GetDlPduSizeStats (uint64_t imsi, uint8_t lcid)
{
  NS_LOG_FUNCTION (this << imsi << (uint16_t) lcid);
  ImsiLcidPair_t p (imsi, lcid);
  std::vector<double> stats;
  Uint32StatsMap::iterator it = m_dlPduSize.find (p);
  if (it == m_dlPduSize.end ())
    {
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      stats.push_back (0.0);
      return stats;

    }
  stats.push_back (m_dlPduSize[p]->getMean ());
  stats.push_back (m_dlPduSize[p]->getStddev ());
  stats.push_back (m_dlPduSize[p]->getMin ());
  stats.push_back (m_dlPduSize[p]->getMax ());
  return stats;
}

std::string
MmWaveBearerStatsCalculator::GetUlOutputFilename (void)
{
  if (m_protocolType == "RLC")
    {
      std::string lteOutputFile = LteStatsCalculator::GetUlOutputFilename ();
      if (lteOutputFile.find("/", 0) != std::string::npos){
        return lteOutputFile;
      }else{
        return m_tracesPath+lteOutputFile;
      }
    }
  else
    {
      return GetUlPdcpOutputFilename ();
    }
}

std::string
MmWaveBearerStatsCalculator::GetDlOutputFilename (void)
{
  if (m_protocolType == "RLC")
    {
      std::string lteOutputFile = LteStatsCalculator::GetDlOutputFilename ();
      if (lteOutputFile.find("/", 0) != std::string::npos){
        return lteOutputFile;
      }else{
        return m_tracesPath+lteOutputFile;
      }
    }
  else
    {
      return GetDlPdcpOutputFilename ();
    }
}

void
MmWaveBearerStatsCalculator::SetUlPdcpOutputFilename (std::string outputFilename)
{
  m_ulPdcpOutputFilename = outputFilename;
  if (m_ulPdcpOutputFilename.find("/", 0) != std::string::npos){
    m_ulPdcpOutputFilename = outputFilename;
  }else{
    m_ulPdcpOutputFilename = m_tracesPath+outputFilename;
  }
}

std::string
MmWaveBearerStatsCalculator::GetUlPdcpOutputFilename (void)
{
  return m_ulPdcpOutputFilename;
}

void
MmWaveBearerStatsCalculator::SetDlPdcpOutputFilename (std::string outputFilename)
{
  m_dlPdcpOutputFilename = outputFilename;
  if (m_dlPdcpOutputFilename.find("/", 0) != std::string::npos){
    m_dlPdcpOutputFilename = outputFilename;
  }else{
    m_dlPdcpOutputFilename = m_tracesPath+outputFilename;
  }
}

std::string
MmWaveBearerStatsCalculator::GetDlPdcpOutputFilename (void)
{
  return m_dlPdcpOutputFilename;
}

// modified

void
MmWaveBearerStatsCalculator::SetGnbRlcBufferSizeFilename (std::string outputFilename)
{
  m_gnbRlcbufferSizeOutputFilename = outputFilename;
  if (m_gnbRlcbufferSizeOutputFilename.find("/", 0) != std::string::npos){
      m_gnbRlcbufferSizeOutputFilename = outputFilename;
  }else{
    m_gnbRlcbufferSizeOutputFilename = m_tracesPath+outputFilename;
  }
}

std::string
MmWaveBearerStatsCalculator::GetGnbRlcBufferSizeFilename (void)
{
  return m_gnbRlcbufferSizeOutputFilename;
}

void
MmWaveBearerStatsCalculator::SetUeRlcBufferSizeFilename (std::string outputFilename)
{
  m_ueRlcbufferSizeOutputFilename = outputFilename;
  if (m_ueRlcbufferSizeOutputFilename.find("/", 0) != std::string::npos){
      m_ueRlcbufferSizeOutputFilename = outputFilename;
  }else{
    m_ueRlcbufferSizeOutputFilename = m_tracesPath+outputFilename;
  }
}

std::string
MmWaveBearerStatsCalculator::GetUeRlcBufferSizeFilename (void)
{
  return m_ueRlcbufferSizeOutputFilename;
}

// 

// end modification

} // namespace mmwave

} // namespace ns3
