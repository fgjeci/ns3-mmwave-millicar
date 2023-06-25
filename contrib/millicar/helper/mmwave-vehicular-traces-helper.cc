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

#include "mmwave-vehicular-traces-helper.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveVehicularTracesHelper");

namespace millicar {

NS_OBJECT_ENSURE_REGISTERED (MmWaveVehicularTracesHelper);

MmWaveVehicularTracesHelper::MmWaveVehicularTracesHelper (std::string filename)
: m_filename(filename)
{
  NS_LOG_FUNCTION (this);

  // if(!m_outputFile.is_open())
  // {
  //   m_outputFile.open(m_filename.c_str());
  //   if (!m_outputFile.is_open ())
  //   {
  //     NS_FATAL_ERROR ("Could not open tracefile");
  //   }
  //   m_outputFile
  //           << "time\trnti\tsinr\tnumsym\ttbsize\tmcs\tlocalRnti\n";
  // }
}

MmWaveVehicularTracesHelper::MmWaveVehicularTracesHelper (){
  NS_LOG_FUNCTION(this);
}

TypeId
MmWaveVehicularTracesHelper::GetTypeId (void)
{
  static TypeId tid =
    TypeId ("ns3::MmWaveVehicularTracesHelper")
    .SetParent<Object> ()
    .AddConstructor<MmWaveVehicularTracesHelper> ()
    // modified
    .AddAttribute ("TracesPath",
                    "The path where to store the path. ",
                    StringValue ("./"),
                    MakeStringAccessor (&MmWaveVehicularTracesHelper::m_tracesPath),
                    MakeStringChecker ())
    .AddAttribute ("UeRlcBufferSizeOutputFilename",
                   "Name of the file where the ue rlc buffer size results will be saved.",
                   StringValue ("UeRlcBufferSizeStats.txt"),
                   MakeStringAccessor (&MmWaveVehicularTracesHelper::SetUeRlcBufferSizeFilename),
                   MakeStringChecker ())
    .AddAttribute ("PhyDataOutputFilename",
                   "Name of the file where the ue phy reports size results will be saved.",
                   StringValue ("sinr-mcs.txt"),
                   MakeStringAccessor (&MmWaveVehicularTracesHelper::SetPhyDataFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeRlcRxOutputFilename",
                   "Name of the file where the ue rlc tx buffer.",
                   StringValue ("UeRlcRxSizeStats.txt"),
                   MakeStringAccessor (&MmWaveVehicularTracesHelper::SetUeRlcRxFilename),
                   MakeStringChecker ())
    .AddAttribute ("UeRlcTxOutputFilename",
                   "Name of the file where the ue rlc rx buffer.",
                   StringValue ("UeRlcTxStats.txt"),
                   MakeStringAccessor (&MmWaveVehicularTracesHelper::SetUeRlcTxFilename),
                   MakeStringChecker ())
    // end modification
  ;
  return tid;

}

MmWaveVehicularTracesHelper::~MmWaveVehicularTracesHelper ()
{
  NS_LOG_FUNCTION (this);
  if (m_outputFile.is_open ())
    {
      m_outputFile.close ();
    }
  if (m_ueRlcBufferSizeFile.is_open ())
    {
      m_ueRlcBufferSizeFile.close ();
    }

  if (m_ueRlcTxFile.is_open ())
  {
    m_ueRlcTxFile.close ();
  }

  if (m_ueRlcRxFile.is_open ())
  {
    m_ueRlcRxFile.close ();
  }
}

void
MmWaveVehicularTracesHelper::McsSinrCallback(const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize, uint8_t mcs, bool corrupt)
{
  if(!m_outputFile.is_open())
  {
    m_outputFile.open(m_filename.c_str());
    if (!m_outputFile.is_open ())
    {
      NS_FATAL_ERROR ("Could not open tracefile");
    }
    m_outputFile<< "time\trnti\tsinr\tnumsym\ttbsize\tmcs\tcorrupt\n";
  }

  double sinrAvg = Sum (sinr) / (sinr.GetSpectrumModel ()->GetNumBands ());
  m_outputFile << Simulator::Now().GetSeconds() << "\t" << rnti << "\t" 
  << 10 * std::log10 (sinrAvg) << "\t" << (uint32_t)numSym << "\t" 
  << tbSize << "\t" << (uint32_t)mcs << "\t" << (uint32_t)corrupt << std::endl;
}

void
MmWaveVehicularTracesHelper::McsSinrCallbackPerDevice(const SpectrumValue& sinr, uint16_t rnti, uint8_t numSym, uint32_t tbSize, uint8_t mcs, bool corrupt, uint16_t localRnti)
{
  double sinrAvg = Sum (sinr) / (sinr.GetSpectrumModel ()->GetNumBands ());

  m_outputFile << Simulator::Now().GetSeconds() << "\t" << rnti << "\t" << 10 * std::log10 (sinrAvg) << "\t" << (uint32_t)numSym << "\t" << tbSize << "\t" << (uint32_t)mcs<< "\t" << localRnti << std::endl;
}

// ue buffer size

void
MmWaveVehicularTracesHelper::SetUeRlcBufferSizeFilename (std::string outputFilename)
{
  NS_LOG_FUNCTION (this << outputFilename<< m_tracesPath);
  m_ueRlcbufferSizeOutputFilename = outputFilename;
  if (m_ueRlcbufferSizeOutputFilename.find("/", 0) != std::string::npos){
      m_ueRlcbufferSizeOutputFilename = outputFilename;
  }else{
    m_ueRlcbufferSizeOutputFilename = m_tracesPath+outputFilename;
  }
}

void
MmWaveVehicularTracesHelper::SetUeRlcTxFilename (std::string outputFilename)
{
  NS_LOG_FUNCTION (this << outputFilename<< m_tracesPath);
  m_ueRlcTxOutputFilename = outputFilename;
  if (m_ueRlcTxOutputFilename.find("/", 0) != std::string::npos){
      m_ueRlcTxOutputFilename = outputFilename;
  }else{
    m_ueRlcTxOutputFilename = m_tracesPath+outputFilename;
  }
}

void
MmWaveVehicularTracesHelper::SetUeRlcRxFilename (std::string outputFilename)
{
  NS_LOG_FUNCTION (this << outputFilename<< m_tracesPath);
  m_ueRlcRxOutputFilename = outputFilename;
  if (m_ueRlcRxOutputFilename.find("/", 0) != std::string::npos){
      m_ueRlcRxOutputFilename = outputFilename;
  }else{
    m_ueRlcRxOutputFilename = m_tracesPath+outputFilename;
  }
}

void
MmWaveVehicularTracesHelper::SetPhyDataFilename (std::string outputFilename)
{
  NS_LOG_FUNCTION (this << outputFilename<< m_tracesPath);
  m_filename = outputFilename;
  if (m_filename.find("/", 0) != std::string::npos){
      m_filename = outputFilename;
  }else{
    m_filename = m_tracesPath+outputFilename;
  }
}

void
MmWaveVehicularTracesHelper::UeRlcBufferSize (uint16_t rnti, uint64_t imsi, uint32_t bufferSize, uint32_t maxBufferSize)
{
  NS_LOG_FUNCTION (this << " cell " << rnti << " buff size "  << bufferSize << " max size " << maxBufferSize);

  if (!m_ueRlcBufferSizeFile.is_open ())
    {
      m_ueRlcBufferSizeFile.open (m_ueRlcbufferSizeOutputFilename.c_str ()); // , std::ios_base::app
      m_ueRlcBufferSizeFile << "rnti"  << " " << "Imsi"  << " " <<  "RlcBufferSize" << " " << "RlcMaxBufferSize"<< " " << "Timestamp" << std::endl;
    }
  m_ueRlcBufferSizeFile << rnti << " " << imsi << " " << bufferSize << " " << maxBufferSize << " " << Simulator::Now () << std::endl;
}

void
MmWaveVehicularTracesHelper::UeRlcBufferSizeShort (Ptr<MmWaveVehicularTracesHelper> helper, uint16_t rnti, uint32_t bufferSize, uint32_t maxBufferSize)
{
  // std::cout << " rnti " << rnti << " buff size "  << bufferSize << " max size " << maxBufferSize << std::endl;
  
  if (!helper->m_ueRlcBufferSizeFile.is_open ())
    {
      // std::cout << " File not opened so we open it" << std::endl;
      helper->m_ueRlcBufferSizeFile.open (helper->m_ueRlcbufferSizeOutputFilename.c_str ()); // , std::ios_base::app
      helper->m_ueRlcBufferSizeFile << "rnti"  <<  "RlcBufferSize" << " " << "RlcMaxBufferSize"<< " " << "Timestamp" << std::endl;
    }
  // std::cout << "File opened " << std::endl;
  helper->m_ueRlcBufferSizeFile << rnti << " " << bufferSize << " " << maxBufferSize << " " << Simulator::Now ().GetSeconds() << std::endl;
}

void
MmWaveVehicularTracesHelper::UeRlcRx (Ptr<MmWaveVehicularTracesHelper> helper, uint16_t rnti, uint16_t rlcRnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
{
  // std::cout << " rnti " << rnti << " buff size "  << bufferSize << " max size " << maxBufferSize << std::endl;
  
  if (!helper->m_ueRlcRxFile.is_open ())
    {
      // std::cout << " File not opened so we open it" << std::endl;
      helper->m_ueRlcRxFile.open (helper->m_ueRlcRxOutputFilename.c_str ()); // , std::ios_base::app
      helper->m_ueRlcRxFile << "rnti"  <<  " " << "lcid"<< " "<< "packetSize"<< " "<< "delay"<< " " << "Timestamp" << std::endl;
    }
  // std::cout << "File opened " << std::endl;
  helper->m_ueRlcRxFile << rnti << " "<< static_cast<uint32_t>(lcid) << " " << packetSize << " " << delay << " " << Simulator::Now ().GetSeconds() << std::endl;
}

void
MmWaveVehicularTracesHelper::UeRlcTx (Ptr<MmWaveVehicularTracesHelper> helper, uint16_t rnti, uint16_t rlcRnti, uint8_t lcid, uint32_t packetSize)
{
  if (!helper->m_ueRlcTxFile.is_open ())
    {
      // std::cout << " File not opened so we open it" << std::endl;
      helper->m_ueRlcTxFile.open (helper->m_ueRlcTxOutputFilename.c_str ()); // , std::ios_base::app
      helper->m_ueRlcTxFile << "rnti"  <<  " " << "lcid"<< " "<< "packetSize"<< " " << "Timestamp" << std::endl;
    }
  // std::cout << "File opened " << std::endl;
  helper->m_ueRlcTxFile << rnti << " "<< static_cast<uint32_t>(lcid) << " " << packetSize << " "  << Simulator::Now ().GetSeconds() << std::endl;
}


// end modification

}

}
