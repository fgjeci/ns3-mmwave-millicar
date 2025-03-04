/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York
 * University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef NODE_V2V_CHANNEL_CONDITION_MODEL_H
#define NODE_V2V_CHANNEL_CONDITION_MODEL_H

#include "ns3/channel-condition-model.h"

namespace ns3
{

class MobilityModel;

/**
 * \ingroup buildings
 *
 * \brief Determines the channel condition based on the buildings deployed in the
 * scenario
 *
 * Code adapted from MmWave3gppBuildingsPropagationLossModel
 */
class NodeV2vChannelConditionModel : public ChannelConditionModel
{
  public:
    /**
     * Get the type ID.
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Constructor for the NodeV2vChannelConditionModel class
     */
    NodeV2vChannelConditionModel();

    /**
     * Destructor for the NodeV2vChannelConditionModel class
     */
    ~NodeV2vChannelConditionModel() override;

    /**
     * Computes the condition of the channel between a and b.
     *
     * \param a mobility model
     * \param b mobility model
     * \return the condition of the channel between a and b
     */
    Ptr<ChannelCondition> GetChannelCondition(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const override;

    /**
     * If this model uses objects of type RandomVariableStream,
     * set the stream numbers to the integers starting with the offset
     * 'stream'. Return the number of streams (possibly zero) that
     * have been assigned.
     *
     * \param stream
     * \return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream) override;

  private:
    /**
     * \brief Checks if the line of sight between position l1 and position l2 is
     *        blocked by a building.
     *
     * \param l1 position
     * \param l2 position
     * \return true if the line of sight is blocked, false otherwise
     */
    bool IsLineOfSightBlocked(const Vector& l1, const Vector& l2) const;

    /**
     * @brief a device is considered to be a square with size m_deviceSize
     * 
     */
    double m_deviceSize;
};

} // namespace ns3

#endif /* NODE_V2V_CHANNEL_CONDITION_MODEL_H */
