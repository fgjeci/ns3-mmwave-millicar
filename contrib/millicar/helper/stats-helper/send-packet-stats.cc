/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 */
#include "send-packet-stats.h"

#include <ns3/rng-seed-manager.h>
#include <ns3/abort.h>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mmwave-module.h>
#include <ns3/sqlite-output.h>
#include <ns3/mmwave-packet-relay-tag.h>

// #include "ns3/mpi-interface.h"

namespace ns3 {
namespace mmwave {
SendPacketStats::SendPacketStats ()
{
}

void
SendPacketStats::SetDb (SQLiteOutput *db, const std::string & tableName)
{
  m_db = db;
  m_tableName = tableName;

  bool ret;

  // drop table 
  // ret = m_db->SpinExec("DROP TABLE IF EXISTS " + tableName + ";");
  // NS_ASSERT (ret);"entity VARCHAR(30) NOT NULL,"

  ret = m_db->SpinExec ("CREATE TABLE IF NOT EXISTS " + tableName + "(" +
					              "SourceRnti INTEGER NOT NULL,"
                        "DestRnti INTEGER NOT NULL,"
                        "InterRnti INTEGER NOT NULL,"
                        "localRnti INTEGER NOT NULL,"
                        // "sourceAddr VARCHAR(50) NOT NULL,"
                        // "destAddr VARCHAR(50) NOT NULL,"
                        // "x DOUBLE NOT NULL,"
                        // "y DOUBLE NOT NULL,"
                        "x INTEGER NOT NULL,"
                        "y INTEGER NOT NULL,"
                        "packetId INTEGER NOT NULL,"
                        "txtime DOUBLE NOT NULL,"
                        "seqNum INTEGER NOT NULL,"
                        "Seed INTEGER NOT NULL,"
                        "Run INTEGER NOT NULL,"
                        "payloadSize INTEGER NOT NULL,"
                        "TimeStamp DOUBLE NOT NULL);");
		  //                        "Run INTEGER NOT NULL);");
  NS_ASSERT (ret);

  SendPacketStats::DeleteWhere (m_db, RngSeedManager::GetSeed (),
                                RngSeedManager::GetRun(), 
                                // static_cast<uint32_t> (MpiInterface::GetSystemId ()),
                                tableName);
}

void
SendPacketStats::SavePacketSend (uint16_t sourceRnti, uint16_t intermediateRnti, 
                        uint16_t destinationRnti, uint16_t localRnti,
                        std::string sourceAddress, std::string destAddress
                        , Vector position
                        ,uint64_t packetId, uint32_t seqNumber, double txTime,
                        uint16_t payloadSize
                        )
{
//	NS_UNUSED (power);
//	NS_LOG_UNCOND("Saving sinr " << cellId << " " << rnti<< " " << power<< " " << avgSinr << " " << bwpId);
	SendPacketCache c;
	c.timeInstance = Simulator::Now();
	c.sourceRnti = sourceRnti;
	c.intermediateRnti = intermediateRnti;
    c.destRnti = destinationRnti;
    c.localRnti = localRnti;
    c.destAddr = destAddress;
    c.sourceAddr = sourceAddress;
    c.packetId = packetId;
    c.seqNumber = seqNumber;
    c.txTime = txTime;
    c.position = position;
    c.payloadSize = payloadSize;

	m_sendPacketCache.emplace_back (c);

  // Let's wait until ~1MB of entries before storing it in the database
  if (m_sendPacketCache.size () * sizeof (SendPacketCache) > 1000)
    {
      WriteCache ();
    }
}

void
SendPacketStats::SavePacketRelay (uint16_t sourceRnti, uint16_t intermediateRnti, 
                        uint16_t destinationRnti, uint16_t localRnti, 
                        std::string sourceAddress, std::string destAddress
                        , Vector position
                        ,uint64_t packetId, uint32_t seqNumber, double txTime,
                        uint16_t payloadSize
                        )
{
//	NS_UNUSED (power);
//	NS_LOG_UNCOND("Saving sinr " << cellId << " " << rnti<< " " << power<< " " << avgSinr << " " << bwpId);
	SendPacketCache c;
	c.timeInstance = Simulator::Now();
	c.sourceRnti = sourceRnti;
	c.intermediateRnti = intermediateRnti;
  c.destRnti = destinationRnti;
  c.localRnti = localRnti;
  c.destAddr = destAddress;
  c.sourceAddr = sourceAddress;
  c.packetId = packetId;
  c.seqNumber = seqNumber;
  c.txTime = txTime;
  c.position = position;
  c.payloadSize = payloadSize;

	m_sendPacketCache.emplace_back (c);

  // Let's wait until ~1MB of entries before storing it in the database
  if (m_sendPacketCache.size () * sizeof (SendPacketCache) > 1000)
    {
      WriteCache ();
    }
}

void
SendPacketStats::EmptyCache()
{
  WriteCache ();
}

void
SendPacketStats::DeleteWhere (SQLiteOutput *p, uint32_t seed,
                              uint32_t run, const std::string &table)
{
  bool ret;
  sqlite3_stmt *stmt;
  ret = p->SpinPrepare (&stmt, "DELETE FROM \"" + table + "\" WHERE SEED = ? AND RUN = ?;");
  NS_ABORT_IF (ret == false);
  ret = p->Bind (stmt, 1, seed);
  NS_ABORT_IF (ret == false);
  ret = p->Bind (stmt, 2, run);

  ret = p->SpinExec (stmt);
  NS_ABORT_IF (ret == false);
}

void SendPacketStats::WriteCache ()
{
  bool ret = m_db->SpinExec ("BEGIN TRANSACTION;");
  for (const auto & v : m_sendPacketCache)
    {
//	  NS_LOG_UNCOND("Writing cache " << v.cellId << " " << v.rnti<< " " << v.power<< " " << v.avgSinr << " " << v.bwpId);
      sqlite3_stmt *stmt;
      ret = m_db->SpinPrepare (&stmt, "INSERT INTO " + m_tableName + " VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);");
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 1, v.sourceRnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 2, v.destRnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 3, static_cast<uint32_t>(v.intermediateRnti));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 4, static_cast<uint32_t>(v.localRnti));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 5, static_cast<uint32_t>(v.position.x));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 6, static_cast<uint32_t>(v.position.y));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 7, static_cast<uint32_t>(v.packetId));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 8, v.txTime);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 9, v.seqNumber);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 10, RngSeedManager::GetSeed ());
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 11, static_cast<uint32_t> (RngSeedManager::GetRun ()));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 12, v.payloadSize);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 13, v.timeInstance.GetSeconds());
      
      // NS_ASSERT (ret);
      // ret = m_db->Bind (stmt, 13, v.sourceAddr); // static_cast<double> (
      // NS_ASSERT (ret);
      // ret = m_db->Bind (stmt, 14, v.destAddr);
      NS_ASSERT (ret);
      // ret = m_db->Bind (stmt, 7, static_cast<uint32_t> (MpiInterface::GetSystemId ()));
      // NS_ASSERT (ret);
      // insert the timestamp
      ret = m_db->SpinExec (stmt);
      NS_ASSERT (ret);
    }
  m_sendPacketCache.clear ();
  ret = m_db->SpinExec ("END TRANSACTION;");
  NS_ASSERT (ret);
}
}
} // namespace ns3
