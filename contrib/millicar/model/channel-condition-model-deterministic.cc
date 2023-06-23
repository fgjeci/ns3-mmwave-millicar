#include "channel-condition-model-deterministic.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/channel-condition-model.h"

// ------------------------------------------------------------------------- //


namespace ns3
{

NS_LOG_COMPONENT_DEFINE("DeterministicVehicleChannelConditionModel");

NS_OBJECT_ENSURE_REGISTERED(DeterministicVehicleChannelConditionModel);

TypeId
DeterministicVehicleChannelConditionModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::DeterministicVehicleChannelConditionModel")
            .SetParent<ChannelConditionModel>()
            .SetGroupName("Propagation")
            .AddAttribute(
                "UpdatePeriod",
                "Specifies the time period after which the channel "
                "condition is recomputed. If set to 0, the channel condition is never updated.",
                TimeValue(MilliSeconds(0)),
                MakeTimeAccessor(&DeterministicVehicleChannelConditionModel::m_updatePeriod),
                MakeTimeChecker())
            .AddAttribute("O2iThreshold",
                          "Specifies what will be the ratio of O2I channel "
                          "conditions. Default value is 0 that corresponds to 0 O2I losses.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&DeterministicVehicleChannelConditionModel::m_o2iThreshold),
                          MakeDoubleChecker<double>(0, 1))
            .AddAttribute("O2iLowLossThreshold",
                          "Specifies what will be the ratio of O2I "
                          "low - high penetration losses. Default value is 1.0 meaning that"
                          "all losses will be low",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&DeterministicVehicleChannelConditionModel::m_o2iLowLossThreshold),
                          MakeDoubleChecker<double>(0, 1))
            .AddAttribute("LinkO2iConditionToAntennaHeight",
                          "Specifies whether the O2I condition will "
                          "be determined based on the UE height, i.e. if the UE height is 1.5 then "
                          "it is O2O, "
                          "otherwise it is O2I.",
                          BooleanValue(false),
                          MakeBooleanAccessor(
                              &DeterministicVehicleChannelConditionModel::m_linkO2iConditionToAntennaHeight),
                          MakeBooleanChecker());
    return tid;
}

DeterministicVehicleChannelConditionModel::DeterministicVehicleChannelConditionModel()
    : ChannelConditionModel()
{
    m_uniformVar = CreateObject<UniformRandomVariable>();
    m_uniformVar->SetAttribute("Min", DoubleValue(0));
    m_uniformVar->SetAttribute("Max", DoubleValue(1));

    m_uniformVarO2i = CreateObject<UniformRandomVariable>();
    m_uniformO2iLowHighLossVar = CreateObject<UniformRandomVariable>();
}

DeterministicVehicleChannelConditionModel::~DeterministicVehicleChannelConditionModel()
{
}

void
DeterministicVehicleChannelConditionModel::DoDispose()
{
    m_channelConditionMap.clear();
    m_updatePeriod = Seconds(0.0);
}

Ptr<ChannelCondition>
DeterministicVehicleChannelConditionModel::GetChannelCondition(Ptr<const MobilityModel> a,
                                                   Ptr<const MobilityModel> b) const
{
    Ptr<ChannelCondition> cond;

    // get the key for this channel
    uint32_t key = GetKey(a, b);

    bool notFound = false; // indicates if the channel condition is not present in the map
    bool update = false;   // indicates if the channel condition has to be updated

    // look for the channel condition in m_channelConditionMap
    auto mapItem = m_channelConditionMap.find(key);
    if (mapItem != m_channelConditionMap.end())
    {
        // NS_LOG_DEBUG("found the channel condition in the map");
        cond = mapItem->second.m_condition;

        // check if it has to be updated
        if (!m_updatePeriod.IsZero() &&
            Simulator::Now() - mapItem->second.m_generatedTime > m_updatePeriod)
        {
            NS_LOG_DEBUG("it has to be updated");
            update = true;
        }
    }
    else
    {
        NS_LOG_DEBUG("channel condition not found");
        notFound = true;
    }

    // if the channel condition was not found or if it has to be updated
    // generate a new channel condition
    if (notFound || update)
    {
        cond = ComputeChannelCondition(a, b);
        // store the channel condition in m_channelConditionMap, used as cache.
        // For this reason you see a const_cast.
        Item mapItem;
        mapItem.m_condition = cond;
        mapItem.m_generatedTime = Simulator::Now();
        const_cast<DeterministicVehicleChannelConditionModel*>(this)->m_channelConditionMap[key] = mapItem;
    }

    return cond;
}

ChannelCondition::O2iConditionValue
DeterministicVehicleChannelConditionModel::ComputeO2i(Ptr<const MobilityModel> a,
                                          Ptr<const MobilityModel> b) const
{
    double o2iProb = m_uniformVarO2i->GetValue(0, 1);

    if (m_linkO2iConditionToAntennaHeight)
    {
        if (std::min(a->GetPosition().z, b->GetPosition().z) == 1.5)
        {
            return ChannelCondition::O2iConditionValue::O2O;
        }
        else
        {
            return ChannelCondition::O2iConditionValue::O2I;
        }
    }
    else
    {
        if (o2iProb < m_o2iThreshold)
        {
            NS_LOG_INFO("Return O2i condition ....");
            return ChannelCondition::O2iConditionValue::O2I;
        }
        else
        {
            NS_LOG_INFO("Return O2o condition ....");
            return ChannelCondition::O2iConditionValue::O2O;
        }
    }
}

Ptr<ChannelCondition>
DeterministicVehicleChannelConditionModel::ComputeChannelCondition(Ptr<const MobilityModel> a,
                                                       Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this << a << b);
    Ptr<ChannelCondition> cond = CreateObject<ChannelCondition>();

    // compute the LOS probability
    // double pLos = ComputePlos(a, b);
    double pNlos = ComputePnlos(a, b);

    NS_LOG_DEBUG("pnlos " << pNlos);

    // get the channel condition
    if (pNlos == 0)
    {
        // LOS
        cond->SetLosCondition(ChannelCondition::LosConditionValue::LOS);
    }
    else if (pNlos == 1)
    {
        // NLOS caused by buildings
        cond->SetLosCondition(ChannelCondition::LosConditionValue::NLOS);
    }
    else
    {
        // NLOSv (added to support vehicular scenarios)
        cond->SetLosCondition(ChannelCondition::LosConditionValue::NLOSv);
    }

    cond->SetO2iCondition(ComputeO2i(a, b));

    if (cond->GetO2iCondition() == ChannelCondition::O2iConditionValue::O2I)
    {
        // Since we have O2I penetration losses, we should choose based on the
        // threshold if it will be low or high penetration losses
        // (see TR38.901 Table 7.4.3)
        double o2iLowHighLossProb = m_uniformO2iLowHighLossVar->GetValue(0, 1);
        ChannelCondition::O2iLowHighConditionValue lowHighLossCondition;

        if (o2iLowHighLossProb < m_o2iLowLossThreshold)
        {
            lowHighLossCondition = ChannelCondition::O2iLowHighConditionValue::LOW;
        }
        else
        {
            lowHighLossCondition = ChannelCondition::O2iLowHighConditionValue::HIGH;
        }
        cond->SetO2iLowHighCondition(lowHighLossCondition);
    }

    return cond;
}

double
DeterministicVehicleChannelConditionModel::ComputePnlos(Ptr<const MobilityModel> a,
                                            Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this << a << b);
    // by default returns 1 - PLOS
    return (1 - ComputePlos(a, b));
}

int64_t
DeterministicVehicleChannelConditionModel::AssignStreams(int64_t stream)
{
    m_uniformVar->SetStream(stream);
    m_uniformVarO2i->SetStream(stream + 1);
    m_uniformO2iLowHighLossVar->SetStream(stream + 2);

    return 3;
}

double
DeterministicVehicleChannelConditionModel::Calculate2dDistance(const Vector& a, const Vector& b)
{
    double x = a.x - b.x;
    double y = a.y - b.y;
    double distance2D = sqrt(x * x + y * y);

    return distance2D;
}

uint32_t
DeterministicVehicleChannelConditionModel::GetKey(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b)
{
    // use the nodes ids to obtain a unique key for the channel between a and b
    // sort the nodes ids so that the key is reciprocal
    uint32_t x1 = std::min(a->GetObject<Node>()->GetId(), b->GetObject<Node>()->GetId());
    uint32_t x2 = std::max(a->GetObject<Node>()->GetId(), b->GetObject<Node>()->GetId());

    // use the cantor function to obtain the key
    uint32_t key = (((x1 + x2) * (x1 + x2 + 1)) / 2) + x2;

    return key;
}
// ------------------------------------------------------------------------- //

}