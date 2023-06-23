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
#ifndef SEND_PACKET_STATS_H
#define SEND_PACKET_STATS_H

#include <inttypes.h>
#include <vector>
#include <ns3/network-module.h>

#include <ns3/sqlite-output.h>

#include <ns3/core-module.h>

namespace ns3 {
namespace mmwave {
/**
 * \brief Class to collect and store the SINR values obtained from a simulation
 *
 * The class is meant to store in a database the SINR values from UE or GNB during
 * a simulation. The class contains a cache, that after some time is written
 * to the disk.
 *
 * \see SetDb
 * \see SaveSinr
 * \see EmptyCache
 */
class SendPacketStats
{
public:
  /**
   * \brief Constructor
   */
  SendPacketStats ();

  /**
   * \brief Install the output dabase.
   * \param db database pointer
   * \param tableName name of the table where the values will be stored
   *
   *  The db pointer must be valid through all the lifespan of the class. The
   * method creates, if not exists, a table for storing the values. The table
   * will contain the following columns:
   *
   *
   * Please note that this method, if the db already contains a table with
   * the same name, also clean existing values that has the same
   * Seed/Run pair.
   */
  void SetDb (SQLiteOutput *db, const std::string& tableName = "sendPacketStats");

  /**
   * \brief Store the SINR values
   * \param cellId Cell ID
   * \param rnti RNTI
   * \param power power (unused)
   * \param avgSinr Average SINR
   * \param bwpId BWP ID
   *
   * The method saves the result in a cache, and if it is necessary, writes the
   * cache to disk before emptying it.
   */
    void SavePacketSend (uint16_t sourceRnti, uint16_t intermediateRnti, 
                        uint16_t destinationRnti, uint16_t localRnti, 
                        std::string sourceAddress, std::string destAddress
                        , Vector position
                        ,uint64_t packetId, uint32_t seqNumber, double txTime
                        );
    
    void SavePacketRelay (uint16_t sourceRnti, uint16_t intermediateRnti, 
                        uint16_t destinationRnti, uint16_t localRnti, 
                        std::string sourceAddress, std::string destAddress
                        , Vector position
                        ,uint64_t packetId, uint32_t seqNumber, double txTime
                        );

  /**
   * \brief Force the cache write to disk, emptying the cache itself.
   */
  void EmptyCache ();

private:
  static void
  DeleteWhere (SQLiteOutput* p, uint32_t seed, uint32_t run, const std::string &table);

  void WriteCache ();

  struct SendPacketCache
  {

    Time timeInstance;
    uint16_t sourceRnti{UINT16_MAX};
    uint16_t intermediateRnti{UINT16_MAX};
    uint16_t destRnti{UINT16_MAX};
    uint16_t localRnti{UINT16_MAX};
    std::string sourceAddr;
    std::string destAddr;
    uint64_t packetId;
    uint32_t seqNumber; 
    double txTime;
    Vector position;

  };

  SQLiteOutput *m_db;                         //!< DB pointer
  std::vector<SendPacketCache> m_sendPacketCache;   //!< Result cache
  std::string m_tableName;                    //!< Table name
};
}
} // namespace ns3

#endif // SEND_PACKET_STATS_H
