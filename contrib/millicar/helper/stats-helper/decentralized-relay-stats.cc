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
#include "decentralized-relay-stats.h"

#include <ns3/rng-seed-manager.h>
#include <ns3/abort.h>
#include <ns3/core-module.h>
#include <ns3/mmwave-module.h>
#include <ns3/sqlite-output.h>

// #include "ns3/mpi-interface.h"

namespace ns3 {
namespace mmwave {
DecentralizedRelayStats::DecentralizedRelayStats ()
{
}

void
DecentralizedRelayStats::SetDb (SQLiteOutput *db, const std::string & tableName)
{
  m_db = db;
  m_tableName = tableName;

  bool ret;

  ret = m_db->SpinExec ("CREATE TABLE IF NOT EXISTS " + tableName + "(" +
                        "Frame INTEGER NOT NULL,"
                        "SubFrame INTEGER NOT NULL,"
                        "Slot INTEGER NOT NULL,"
                        "Rnti INTEGER NOT NULL,"
                        "DestRnti INTEGER NOT NULL,"
                        "IntermediateRnti INTEGER NOT NULL,"
                        "directLinkSnr DOUBLE NOT NULL,"
                        "bestLinkSnr DOUBLE NOT NULL,"
					              "Seed INTEGER NOT NULL,"
						            "Run INTEGER NOT NULL,"
					              "TimeStamp DOUBLE NOT NULL);");
		  //                        "Run INTEGER NOT NULL);");
  NS_ASSERT (ret);

  DecentralizedRelayStats::DeleteWhere (m_db, RngSeedManager::GetSeed (),
                                RngSeedManager::GetRun(), 
                                // static_cast<uint32_t> (MpiInterface::GetSystemId ()),
                                tableName);
}

void
DecentralizedRelayStats::SaveDecentralizedRelayReport (uint16_t frame, uint8_t subframe, uint8_t slot, 
                                    uint16_t rnti, uint16_t destRnti, uint16_t intermediateRnti, 
                                    double directLinkSnr, double bestLinkSnr)
{
//	NS_UNUSED (power);
//	NS_LOG_UNCOND("Saving sinr " << cellId << " " << rnti<< " " << power<< " " << avgSinr << " " << bwpId);
	DecentralizedRelayCache c;
  c.frame=frame;
  c.subframe=subframe;
  c.slot=slot;
	c.rnti = rnti;
  c.destRnti = destRnti;
  c.intermediateRnti = intermediateRnti;
	c.directLinkSnr = directLinkSnr;
	c.bestLinkSnr = bestLinkSnr;
	c.timeInstance = Simulator::Now();

	m_decentralizedRelayCache.emplace_back (c);

  // Let's wait until ~1MB of entries before storing it in the database
  if (m_decentralizedRelayCache.size () * sizeof (DecentralizedRelayCache) > 100000)
    {
      WriteCache ();
    }
}

void
DecentralizedRelayStats::EmptyCache()
{
  WriteCache ();
}

void
DecentralizedRelayStats::DeleteWhere (SQLiteOutput *p, uint32_t seed,
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

void DecentralizedRelayStats::WriteCache ()
{
  bool ret = m_db->SpinExec ("BEGIN TRANSACTION;");
  for (const auto & v : m_decentralizedRelayCache)
    {
//	  NS_LOG_UNCOND("Writing cache " << v.cellId << " " << v.rnti<< " " << v.power<< " " << v.avgSinr << " " << v.bwpId);
      sqlite3_stmt *stmt;
      ret = m_db->SpinPrepare (&stmt, "INSERT INTO " + m_tableName + " VALUES (?,?,?,?,?,?,?,?,?,?,?);");
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 1, static_cast<uint32_t> (v.frame));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 2, static_cast<uint32_t> (v.subframe));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 3, static_cast<uint32_t> (v.slot));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 4, v.rnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 5, v.destRnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 6, v.intermediateRnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 7, v.directLinkSnr);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 8, v.bestLinkSnr); // static_cast<double> (
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 9, RngSeedManager::GetSeed ());
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 10, static_cast<uint32_t> (RngSeedManager::GetRun ()));
      NS_ASSERT (ret);
      // ret = m_db->Bind (stmt, 9, static_cast<uint32_t> (MpiInterface::GetSystemId ()));
      // NS_ASSERT (ret);
      // insert the timestamp
      ret = m_db->Bind (stmt, 11, v.timeInstance.GetSeconds());
      NS_ASSERT (ret);

      ret = m_db->SpinExec (stmt);
      NS_ASSERT (ret);
    }
  m_decentralizedRelayCache.clear ();
  ret = m_db->SpinExec ("END TRANSACTION;");
  NS_ASSERT (ret);
}
}
} // namespace ns3
