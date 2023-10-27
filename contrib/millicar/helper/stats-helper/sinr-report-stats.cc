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
#include "sinr-report-stats.h"

#include <ns3/rng-seed-manager.h>
#include <ns3/abort.h>
#include <ns3/core-module.h>
#include <ns3/mmwave-module.h>
#include <ns3/sqlite-output.h>

// #include "ns3/mpi-interface.h"

namespace ns3 {
namespace mmwave {
SinrReportStats::SinrReportStats ()
{
}

void
SinrReportStats::SetDb (SQLiteOutput *db, const std::string & tableName)
{
  m_db = db;
  m_tableName = tableName;

  bool ret;

  // drop table 
  // ret = m_db->SpinExec("DROP TABLE IF EXISTS " + tableName + ";");
  // NS_ASSERT (ret);

  ret = m_db->SpinExec ("CREATE TABLE IF NOT EXISTS " + tableName + "(" +
                        "SourceRnti INTEGER NOT NULL,"
                        "Rnti INTEGER NOT NULL,"
                        "NumSym INTEGER NOT NULL,"
                        "snr DOUBLE NOT NULL,"
                        "sinr DOUBLE NOT NULL,"
                        "tbSize INTEGER NOT NULL,"
                        "Seed INTEGER NOT NULL,"
                        "Run INTEGER NOT NULL,"
                        "TimeStamp DOUBLE NOT NULL);");
		  //                        "Run INTEGER NOT NULL);");
  NS_ASSERT (ret);

  SinrReportStats::DeleteWhere (m_db, RngSeedManager::GetSeed (),
                                RngSeedManager::GetRun(), 
                                // static_cast<uint32_t> (MpiInterface::GetSystemId ()),
                                tableName);
}

void
SinrReportStats::SaveSinr (uint16_t sourceRnti, uint16_t rnti, uint8_t numSym, uint32_t tbSize, double snr, double sinr)
{
//	NS_UNUSED (power);
//	NS_LOG_UNCOND("Saving sinr " << cellId << " " << rnti<< " " << power<< " " << avgSinr << " " << bwpId);
	SinrResultCache c;
	c.timeInstance = Simulator::Now();
	c.sourceRnti = sourceRnti;
	c.rnti = rnti;
	c.numSym = numSym;
  if (std::isnan(std::abs(sinr)) ){
    c.sinr =0;
  }else{
    c.sinr = sinr;
  }
  if (std::isnan(std::abs(snr)) ){
    c.snr =0;
  }else{
    c.snr = snr;
  }
  // c.snr = snr;
	c.tbSize = tbSize;

	m_sinrCache.emplace_back (c);

//  m_sinrCache.emplace_back (SinrResultCache (cellId, bwpId, rnti, avgSinr));

  // Let's wait until ~1MB of entries before storing it in the database
  if (m_sinrCache.size () * sizeof (SinrResultCache) > 100000)
    {
      WriteCache ();
    }
}

void
SinrReportStats::EmptyCache()
{
  WriteCache ();
}

void
SinrReportStats::DeleteWhere (SQLiteOutput *p, uint32_t seed,
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

void SinrReportStats::WriteCache ()
{
  bool ret = m_db->SpinExec ("BEGIN TRANSACTION;");
  for (const auto & v : m_sinrCache)
    {
//	  NS_LOG_UNCOND("Writing cache " << v.cellId << " " << v.rnti<< " " << v.power<< " " << v.avgSinr << " " << v.bwpId);
      sqlite3_stmt *stmt;
      ret = m_db->SpinPrepare (&stmt, "INSERT INTO " + m_tableName + " VALUES (?,?,?,?,?,?,?,?,?);");
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 1, v.sourceRnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 2, v.rnti);
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 3, static_cast<uint32_t>(v.numSym));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 4, v.snr); // static_cast<double> (
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 5, v.sinr); // static_cast<double> (
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 6, static_cast<uint32_t>(v.tbSize));
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 7, RngSeedManager::GetSeed ());
      NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 8, static_cast<uint32_t> (RngSeedManager::GetRun ()));
      NS_ASSERT (ret);
      // ret = m_db->Bind (stmt, 9, static_cast<uint32_t> (MpiInterface::GetSystemId ()));
      // NS_ASSERT (ret);
      // insert the timestamp
      ret = m_db->Bind (stmt, 9, v.timeInstance.GetSeconds());
      NS_ASSERT (ret);

      ret = m_db->SpinExec (stmt);
      NS_ASSERT (ret);
    }
  m_sinrCache.clear ();
  ret = m_db->SpinExec ("END TRANSACTION;");
  NS_ASSERT (ret);
}
}
} // namespace ns3
