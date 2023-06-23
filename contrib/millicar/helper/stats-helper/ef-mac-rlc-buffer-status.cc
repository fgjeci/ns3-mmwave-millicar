/*
 * ef-mac-rlc-buffer-status.cc
 *
 *  Created on: Sep 7, 2022
 *      Author: ugjeci
 */



#include "ef-mac-rlc-buffer-status.h"

#include <ns3/rng-seed-manager.h>
#include <ns3/abort.h>
#include <ns3/core-module.h>
#include <ns3/mmwave-module.h>
#include <ns3/sqlite-output.h>

// #include "ns3/mpi-interface.h"

namespace ns3 {
namespace mmwave {
MacBsrStats::MacBsrStats ()
{
}

void
MacBsrStats::SetDb (SQLiteOutput *db, const std::string & tableName)
{
  m_db = db;
  m_tableName = tableName;

  bool ret;

  // drop table 
  // ret = m_db->SpinExec("DROP TABLE IF EXISTS " + tableName + ";");
  // NS_ASSERT (ret);

  ret = m_db->SpinExec ("CREATE TABLE IF NOT EXISTS " + tableName +
                        "(Frame INTEGER NOT NULL,"
                        "SubFrame INTEGER NOT NULL,"
                        "Slot INTEGER NOT NULL,"
                        "cellId INTEGER NOT NULL,"
                        "rnti INTEGER NOT NULL,"
                        "currenttxqueuesize INTEGER NOT NULL,"
                        "headoflinedelaynewtx INTEGER NOT NULL,"
                        "sizependingstatusmsg INTEGER NOT NULL,"
                        "currentrtxqueuesize INTEGER NOT NULL,"
                        "headoflinedelayrtx INTEGER NOT NULL,"
                        "Seed INTEGER NOT NULL,"
		  	  	  	  	"Run INTEGER NOT NULL,"
		  	  	  	  	"TimeStamp DOUBLE NOT NULL);");
//                        "Run INTEGER NOT NULL);");

  NS_ASSERT (ret);

  MacBsrStats::DeleteWhere (m_db, RngSeedManager::GetSeed (),
                                RngSeedManager::GetRun(), 
                                // static_cast<uint32_t> (MpiInterface::GetSystemId ()),
                                tableName);
}

void
MacBsrStats::SaveMacBsrStats (mmwave::SfnSf sfnSf, uint16_t cellId, uint16_t rnti, 
  uint32_t currenttxqueuesize, uint16_t headoflinedelaynewtx, 
  uint16_t sizependingstatusmsg, uint32_t currentrtxqueuesize, 
  uint16_t headoflinedelayrtx, uint8_t lcid)
{
	MacBsrCache c;
  c.sfnSf = sfnSf;
  c.timeInstance = Simulator::Now();
  c.cellId = cellId;
  c.rnti = rnti;
  c.currenttxqueuesize = currenttxqueuesize;
  c.headoflinedelaynewtx = headoflinedelaynewtx;
  c.sizependingstatusmsg = sizependingstatusmsg;
  c.currentrtxqueuesize = currentrtxqueuesize;
  c.headoflinedelayrtx = headoflinedelayrtx;
  c.lcid = lcid;

  m_macBsrCache.emplace_back (c);

  // Let's wait until ~1MB of entries before storing it in the database
  if (m_macBsrCache.size () * sizeof (MacBsrCache) > 100000) // 1000000
    {
      WriteCache ();
    }
}

void
MacBsrStats::EmptyCache()
{
  WriteCache ();
}

void
MacBsrStats::DeleteWhere (SQLiteOutput *p, uint32_t seed,
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

void MacBsrStats::WriteCache ()
{
  bool ret = m_db->SpinExec ("BEGIN TRANSACTION;");
  for (const auto & v : m_macBsrCache)
	{
	  sqlite3_stmt *stmt;
	  ret = m_db->SpinPrepare (&stmt, "INSERT INTO " + m_tableName + " VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);");
    NS_ASSERT (ret);
    ret = m_db->Bind (stmt, 1, static_cast<uint32_t> (v.sfnSf.m_frameNum ));
    NS_ASSERT (ret);
    ret = m_db->Bind (stmt, 2, static_cast<uint32_t> (v.sfnSf.m_sfNum ));
    NS_ASSERT (ret);
    ret = m_db->Bind (stmt, 3, static_cast<uint32_t> (v.sfnSf.m_slotNum));
	  NS_ASSERT (ret);
	  ret = m_db->Bind (stmt, 4, static_cast<uint32_t> (v.cellId));
	  NS_ASSERT (ret);
	  ret = m_db->Bind (stmt, 5, static_cast<uint32_t> (v.rnti));
	  NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 6, static_cast<uint32_t> (v.currenttxqueuesize));
	  NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 7, static_cast<uint32_t> (v.headoflinedelaynewtx));
	  NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 8, static_cast<uint32_t> (v.sizependingstatusmsg));
	  NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 9, static_cast<uint32_t> (v.currentrtxqueuesize));
	  NS_ASSERT (ret);
      ret = m_db->Bind (stmt, 10, static_cast<uint32_t> (v.headoflinedelayrtx));
	  NS_ASSERT (ret);
	  ret = m_db->Bind (stmt, 11, RngSeedManager::GetSeed ());
	  NS_ASSERT (ret);
	  ret = m_db->Bind (stmt, 12, static_cast<uint32_t> (RngSeedManager::GetRun ()));
	  NS_ASSERT (ret);
    // ret = m_db->Bind (stmt, 12, static_cast<uint32_t> (MpiInterface::GetSystemId ()));
    //   NS_ASSERT (ret);
	  // insert the timestamp
	  ret = m_db->Bind (stmt, 13, v.timeInstance);
	  NS_ASSERT (ret);

	  ret = m_db->SpinExec (stmt);
	  NS_ASSERT (ret);
	}
  m_macBsrCache.clear ();
  ret = m_db->SpinExec ("END TRANSACTION;");
  NS_ASSERT (ret);
}
}
} // namespace ns3

