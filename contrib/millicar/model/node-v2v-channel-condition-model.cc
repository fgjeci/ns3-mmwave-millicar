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

#include "node-v2v-channel-condition-model.h"

#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include <ns3/box.h>
#include "ns3/double.h"
#include <ns3/vector.h>


namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NodeV2vChannelConditionModel");

NS_OBJECT_ENSURE_REGISTERED(NodeV2vChannelConditionModel);

TypeId
NodeV2vChannelConditionModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NodeV2vChannelConditionModel")
                            .SetParent<ChannelConditionModel>()
                            .SetGroupName("Buildings")
                            .AddConstructor<NodeV2vChannelConditionModel>()
                            .AddAttribute(
                                "deviceSize",
                                "We set the size of a node as a 2 by 2 square",
                                DoubleValue(2.0),
                                MakeDoubleAccessor(&NodeV2vChannelConditionModel::m_deviceSize),
                                MakeDoubleChecker<double>(0.0, 10.0));
                            ;
    return tid;
}

NodeV2vChannelConditionModel::NodeV2vChannelConditionModel()
    : ChannelConditionModel()
{
}

NodeV2vChannelConditionModel::~NodeV2vChannelConditionModel()
{
}

Ptr<ChannelCondition>
NodeV2vChannelConditionModel::GetChannelCondition(Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this);

    Ptr<ChannelCondition> cond = CreateObject<ChannelCondition>();

    // The channel condition should be LOS if the line of sight is not blocked,
    // otherwise NLOS if there is a vehicle in the intermediate
    bool blocked = IsLineOfSightBlocked(a->GetPosition(), b->GetPosition());
    NS_LOG_DEBUG("Pos A: " << a->GetPosition() << " b " << b->GetPosition()
                << " block " << blocked);
    // NS_LOG_DEBUG("a and b are obscured by a vehicle, blocked " << blocked);
    if (!blocked)
    {
        // NS_LOG_DEBUG("Set LOS");
        cond->SetLosCondition(ChannelCondition::LosConditionValue::LOS);
    }
    else
    {
        // NS_LOG_DEBUG("Set NLOSv");
        cond->SetLosCondition(ChannelCondition::LosConditionValue::NLOSv);
    }

    return cond;
}

bool
NodeV2vChannelConditionModel::IsLineOfSightBlocked(const ns3::Vector& l1,
                                                     const ns3::Vector& l2) const
{
    for (NodeList::Iterator nodeIt = NodeList::Begin(); nodeIt != NodeList::End(); ++nodeIt){
        // we consider each node as a box with center its position and 
        // get position of node 
        Ptr<MobilityModel> mm = DynamicCast<Node>(*nodeIt)->GetObject<MobilityModel>();
        // we should check that the position of the current node is different from 
        // the position of li and l2
        if ((mm->GetPosition()!=l1) & (mm->GetPosition()!=l2)){
            double _xmin = mm->GetPosition().x - m_deviceSize/2; 
            double _xmax = mm->GetPosition().x + m_deviceSize/2;
            double _ymin = mm->GetPosition().y- m_deviceSize/2;
            double _ymax = mm->GetPosition().y + m_deviceSize/2;
            Box nodeBoxShape(_xmin, _xmax, _ymin, _ymax, 0, 1.6); 
            if (nodeBoxShape.IsIntersect(l1, l2)){
                NS_LOG_DEBUG("Devices at pos " << "(" << l1.x << ", " << l1.y << ")"
                            << " and " << "(" << l2.x << ", " << l2.y << ")"
                            << " have obstacle device at pos "
                            << "(" << mm->GetPosition().x << ", " << mm->GetPosition().y << ")");
                return true;
            }
        }

        
    }

    // The line of sight should not be blocked if the line-segment between
    // l1 and l2 did not intersect any building.
    return false;
}

int64_t
NodeV2vChannelConditionModel::AssignStreams(int64_t /* stream */)
{
    return 0;
}

} // end namespace ns3
