/*
 * Copyright (c) 2020 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 * Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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

#ifndef THREE_GPP_V2V_BLOCKAGE_CHANNEL_CONDITION_MODEL
#define THREE_GPP_V2V_BLOCKAGE_CHANNEL_CONDITION_MODEL

#include "ns3/buildings-channel-condition-model.h"

#include "ns3/channel-condition-model.h"
// #include "ns3/three-gpp-v2v-channel-condition-model.h"
#include "channel-condition-model-deterministic.h"
#include "node-v2v-channel-condition-model.h"

#include <functional>

namespace ns3
{

class MobilityModel;
// class NodeV2vChannelConditionModel;

/**
 * \ingroup buildings
 *
 * \brief Computes the channel condition for the V2V Urban scenario
 *
 * Computes the channel condition following the specifications for the
 * V2V Urban scenario reported in Table 6.2-1 of 3GPP TR 37.885.
 *
 * 3GPP TR 37.885 defines 3 different channel states for vehicular environments:
 * LOS, NLOS and NLOSv, the latter representing the case in which the LOS path is
 * blocked by other vehicles in the scenario. The document defines a probabilistic
 * model to determine if the channel state is LOS or NLOSv, while the NLOS state
 * is determined in a deterministic way based on the buildings deployed in the
 * scenario. For this reason, this class makes use of an instance of
 * BuildingsChannelConditionModel to determine if the LOS is obstructed by
 * buildings or not.
 */
class ThreeGppV2vUrbanBlockageChannelConditionModel : public DeterministicVehicleChannelConditionModel
{
  public:
    /**
     * Get the type ID.
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Constructor for the ThreeGppV2vUrbanBlockageChannelConditionModel class
     */
    ThreeGppV2vUrbanBlockageChannelConditionModel();

    /**
     * Destructor for the ThreeGppV2vUrbanBlockageChannelConditionModel class
     */
    ~ThreeGppV2vUrbanBlockageChannelConditionModel() override;

    /**
     * Set the channel model object
     * \param channel a pointer to an object implementing the MatrixBasedChannelModel interface
     */
    void SetV2vChannelConditionModel(Ptr<NodeV2vChannelConditionModel> channel);

    /**
     * Get the channel model object
     * \return a pointer to the object implementing the MatrixBasedChannelModel interface
     */
    Ptr<NodeV2vChannelConditionModel> GetV2vChannelConditionModel() const;

  private:
    /**
     * Compute the LOS probability as specified in Table Table 6.2-1 of 3GPP TR 37.885
     * for the V2V Urban scenario.
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the LOS probability
     */
    double ComputePlos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const override;

    /**
     * Compute the NLOS probability. It determines the presence of obstructions
     * between the tx and the rx based on the buildings deployed in the scenario.
     * It returns 1 if the LOS path is obstructed, 0 otherwise.
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the NLOS probability
     */
    double ComputePnlos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const override;

    Ptr<BuildingsChannelConditionModel>
        m_buildingsCcm; //!< used to determine the obstructions due to buildings

      
    Ptr<NodeV2vChannelConditionModel> m_nodeV2vChannelConditionMode;
};

} // namespace ns3

#endif /* THREE_GPP_V2V_BLOCKAGE_CHANNEL_CONDITION_MODEL */
