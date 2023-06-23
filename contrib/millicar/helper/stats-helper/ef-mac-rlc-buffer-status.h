/*
 * ef-mac-rlc-buffer-status.h
 *
 *  Created on: Sep 7, 2022
 *      Author: ugjeci
 */

#ifndef CONTRIB_ELEPHANT_FLOW_HELPER_EF_MAC_RLC_BUFFER_STATUS_H_
#define CONTRIB_ELEPHANT_FLOW_HELPER_EF_MAC_RLC_BUFFER_STATUS_H_

#include <inttypes.h>
#include <vector>

#include <ns3/sqlite-output.h>
#include <ns3/mmwave-phy-mac-common.h>

#include <ns3/core-module.h>

namespace ns3 {
namespace mmwave {
    /**
 * \brief Class to collect and store the MAC buffer status report coming from Rlc
 *
 *
 * \see SetDb
 * \see SaveMacBaffurStatusReportStats
 * \see EmptyCache
 */
class MacBsrStats{
    public:
  /**
   * \brief Constructor
   */
	MacBsrStats ();

    /**
   * \brief Install the output dabase.
   * \param db database pointer
   * \param tableName name of the table where the values will be stored
   *
   *  The db pointer must be valid through all the lifespan of the class. The
   * method creates, if not exists, a table for storing the values. The table
   * will contain the following columns:
   *
   * - "(cellId INTEGER NOT NULL,"
   * - "rnti INTEGER NOT NULL,"
   * - "currenttxqueuesize INTEGER NOT NULL,"
   * - "headoflinedelaynewtx INTEGER NOT NULL,"
   * - "sizependingstatusmsg INTEGER NOT NULL,"
   * - "currentrtxqueuesize INTEGER NOT NULL,"
   * - "headoflinedelayrtx INTEGER NOT NULL,"
   * - "lcid INTEGER NOT NULL);"
   *
   * Please note that this method, if the db already contains a table with
   * the same name, also clean existing values that has the same
   * Seed/Run pair.
   */
  void SetDb (SQLiteOutput *db, const std::string& tableName = "macBsr");

  /**
   * \brief Save the slot statistics
   * \param [in] bwdid band
   * \param [in] tb Tb size
   */
  void SaveMacBsrStats (mmwave::SfnSf sfnSf, uint16_t cellId, uint16_t rnti, 
  uint32_t currenttxqueuesize, uint16_t headoflinedelaynewtx, 
  uint16_t sizependingstatusmsg, uint32_t currentrtxqueuesize, 
  uint16_t headoflinedelayrtx, uint8_t lcid);

  /**
   * \brief Force the cache write to disk, emptying the cache itself.
   */
  void EmptyCache ();

private:
  static void
  DeleteWhere (SQLiteOutput* p, uint32_t seed, uint32_t run, const std::string &table);

  void WriteCache ();

  struct MacBsrCache
  {
    Time timeInstance;
    SfnSf sfnSf;
    uint16_t cellId; 
    uint16_t rnti;
    uint32_t currenttxqueuesize; 
    uint16_t headoflinedelaynewtx; 
    uint16_t sizependingstatusmsg; 
    uint32_t currentrtxqueuesize; 
    uint16_t headoflinedelayrtx; 
    uint8_t lcid;
  };

  SQLiteOutput *m_db;                         //!< DB pointer
  std::vector<MacBsrCache> m_macBsrCache;         //!< Result cache
  std::string m_tableName;  


};
}
}



#endif /* CONTRIB_ELEPHANT_FLOW_HELPER_EF_MAC_RLC_BUFFER_STATUS_H_ */
