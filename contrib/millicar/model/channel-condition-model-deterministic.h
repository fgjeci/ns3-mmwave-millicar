#ifndef CHANNEL_CONDITION_MODEL_DETERMINISTIC_H
#define CHANNEL_CONDITION_MODEL_DETERMINISTIC_H


#include "ns3/channel-condition-model.h"


namespace ns3
{

  // class ChannelConditionModel;

/**
 * \ingroup propagation
 *
 * \brief Base class for the 3GPP channel condition models
 *
 */
class DeterministicVehicleChannelConditionModel : public ChannelConditionModel
{
  public:
    /**
     * Get the type ID.
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Constructor for the ThreeGppRmaChannelConditionModel class
     */
    DeterministicVehicleChannelConditionModel();

    /**
     * Destructor for the ThreeGppRmaChannelConditionModel class
     */
    ~DeterministicVehicleChannelConditionModel() override;

    /**
     * \brief Retrieve the condition of the channel between a and b.
     *
     * If the channel condition does not exists, the method computes it by calling
     * ComputeChannelCondition and stores it in a local cache, that will be updated
     * following the "UpdatePeriod" parameter.
     *
     * \param a mobility model
     * \param b mobility model
     * \return the condition of the channel between a and b
     */
    Ptr<ChannelCondition> GetChannelCondition(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const override;

    /**
     * If this  model uses objects of type RandomVariableStream,
     * set the stream numbers to the integers starting with the offset
     * 'stream'. Return the number of streams (possibly zero) that
     * have been assigned.
     *
     * \param stream the offset used to set the stream numbers
     * \return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream) override;

  protected:
    void DoDispose() override;

    /**
     * Determine the density of vehicles in a V2V scenario.
     */
    enum VehicleDensity
    {
        LOW,
        MEDIUM,
        HIGH,
        INVALID
    };

    /**
     * \brief Computes the 2D distance between two 3D vectors
     * \param a the first 3D vector
     * \param b the second 3D vector
     * \return the 2D distance between a and b
     */
    static double Calculate2dDistance(const Vector& a, const Vector& b);

    Ptr<UniformRandomVariable> m_uniformVar; //!< uniform random variable

  private:
    /**
     * This method computes the channel condition based on a probabilistic model
     * that is specific for the scenario of interest
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the channel condition
     */
    Ptr<ChannelCondition> ComputeChannelCondition(Ptr<const MobilityModel> a,
                                                  Ptr<const MobilityModel> b) const;

    /**
     * Compute the LOS probability.
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the LOS probability
     */
    virtual double ComputePlos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const = 0;

    /**
     * Determines whether the channel condition is O2I or O2O
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the O2I channelcondition
     */
    virtual ChannelCondition::O2iConditionValue ComputeO2i(Ptr<const MobilityModel> a,
                                                           Ptr<const MobilityModel> b) const;

    /**
     * Compute the NLOS probability. By default returns 1 - PLOS
     *
     * \param a tx mobility model
     * \param b rx mobility model
     * \return the LOS probability
     */
    virtual double ComputePnlos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;

    /**
     * \brief Returns a unique and reciprocal key for the channel between a and b.
     * \param a tx mobility model
     * \param b rx mobility model
     * \return channel key
     */
    static uint32_t GetKey(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b);

    /**
     * Struct to store the channel condition in the m_channelConditionMap
     */
    struct Item
    {
        Ptr<ChannelCondition> m_condition; //!< the channel condition
        Time m_generatedTime;              //!< the time when the condition was generated
    };

    std::unordered_map<uint32_t, Item>
        m_channelConditionMap; //!< map to store the channel conditions
    Time m_updatePeriod;       //!< the update period for the channel condition

    double m_o2iThreshold{
        0}; //!< the threshold for determining what is the ratio of channels with O2I
    double m_o2iLowLossThreshold{0}; //!< the threshold for determining what is the ratio of low -
                                     //!< high O2I building penetration losses
    double m_linkO2iConditionToAntennaHeight{
        false}; //!< the indicator that determines whether the O2I/O2O condition is determined based
                //!< on the UE height
    Ptr<UniformRandomVariable> m_uniformVarO2i; //!< uniform random variable that is used for the
                                                //!< generation of the O2i conditions
    Ptr<UniformRandomVariable>
        m_uniformO2iLowHighLossVar; //!< a uniform random variable for the calculation of the
                                    //!< low/high losses, see TR38.901 Table 7.4.3-2
};


}
#endif /* CHANNEL_CONDITION_MODEL_H */