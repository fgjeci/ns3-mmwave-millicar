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

#include "three-gpp-v2v-blockage-channel-condition-model.h"
#include "node-v2v-channel-condition-model.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include <ns3/building-list.h>
#include <ns3/pointer.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThreeGppV2vUrbanBlockageChannelConditionModel");

NS_OBJECT_ENSURE_REGISTERED(ThreeGppV2vUrbanBlockageChannelConditionModel);

TypeId
ThreeGppV2vUrbanBlockageChannelConditionModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThreeGppV2vUrbanBlockageChannelConditionModel")
                            .SetParent<DeterministicVehicleChannelConditionModel>()
                            // .SetGroupName("Buildings")
                            .AddConstructor<ThreeGppV2vUrbanBlockageChannelConditionModel>()
                            .AddAttribute(
                                "V2vChannelConditionModel",
                                "Pointer to the channel condition model for v2v.",
                                PointerValue(),
                                MakePointerAccessor(&ThreeGppV2vUrbanBlockageChannelConditionModel::SetV2vChannelConditionModel,
                                                    &ThreeGppV2vUrbanBlockageChannelConditionModel::GetV2vChannelConditionModel),
                                // MakePointerAccessor(&ThreeGppV2vUrbanBlockageChannelConditionModel::m_nodeV2vChannelConditionMode),
                                MakePointerChecker<NodeV2vChannelConditionModel>())
                            ;
    return tid;
}

void
ThreeGppV2vUrbanBlockageChannelConditionModel::SetV2vChannelConditionModel(Ptr<NodeV2vChannelConditionModel> channel)
{
    m_nodeV2vChannelConditionMode = channel;
}

Ptr<NodeV2vChannelConditionModel>
ThreeGppV2vUrbanBlockageChannelConditionModel::GetV2vChannelConditionModel() const
{
    return m_nodeV2vChannelConditionMode;
}

ThreeGppV2vUrbanBlockageChannelConditionModel::ThreeGppV2vUrbanBlockageChannelConditionModel()
    : DeterministicVehicleChannelConditionModel()
{
    m_buildingsCcm = CreateObject<BuildingsChannelConditionModel>();
    m_nodeV2vChannelConditionMode = CreateObject<NodeV2vChannelConditionModel> ();
}

ThreeGppV2vUrbanBlockageChannelConditionModel::~ThreeGppV2vUrbanBlockageChannelConditionModel()
{
}

double
ThreeGppV2vUrbanBlockageChannelConditionModel::ComputePlos(Ptr<const MobilityModel> a,
                                                   Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this);

    // determine if there is a building in between the tx and rx
    Ptr<ChannelCondition> cond = m_buildingsCcm->GetChannelCondition(a, b);
    NS_ASSERT_MSG(cond->IsO2o(), "The nodes should be outdoor");

    double pLos = 0.0;
    if (cond->IsLos())
    {
        // compute the 2D distance between a and b
        double distance2D = Calculate2dDistance(a->GetPosition(), b->GetPosition());

        // compute the LOS probability (see 3GPP TR 37.885, Table 6.2-1)
        pLos = std::min(1.0, 1.05 * exp(-0.0114 * distance2D));
    }

    return pLos;
}

double
ThreeGppV2vUrbanBlockageChannelConditionModel::ComputePnlos(Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this);
    double pNlos = 0.0;

    // check if we have nlos from vehicle
    Ptr<ChannelCondition> condV2V = m_nodeV2vChannelConditionMode->GetChannelCondition(a,b);
    if(condV2V->IsNlosv()){
        pNlos = 2.0;
    }

    // determine the NLOS due to buildings
    Ptr<ChannelCondition> cond = m_buildingsCcm->GetChannelCondition(a, b);
    NS_ASSERT_MSG(cond->IsO2o(), "The nodes should be outdoor");
    if (cond->IsNlos())
    {
        pNlos = 1.0;
    }

    return pNlos;
}

} // end namespace ns3
