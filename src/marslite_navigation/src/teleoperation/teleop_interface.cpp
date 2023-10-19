#include "marslite_navigation/teleop_interface.h"

namespace marslite_navigation {

bool TeleopInterface::parseParameters(void)
{
    ros::NodeHandle pNh("~");

    forceStopDistance_ = pNh.param<float>("force_stop_distance", 0.5);
    constantLinearVelocityLimit_ = pNh.param<float>("constant_linear_velocity_limit", 0.7);
    constantAngularVelocityLimit_ = pNh.param<float>("constant_angular_velocity_limit", 0.3);
    constantLinearVelocityStep_ = pNh.param<float>("constant_linear_velocity_step", 0.01);
    constantAngularVelocityStep_ = pNh.param<float>("constant_angular_velocity_step", 0.05);
    predictTimeInterval_ = pNh.param<float>("predict_time_interval", 2);
    sectorHalfRange_ = pNh.param<int>("sector_half_range", 90);

    useAPF_ = pNh.param<bool>("use_apf", true);
    messageEnabled_ = pNh.param<bool>("message_enabled", true);
    autoSlowDownEnabled_ = pNh.param<bool>("auto_slow_down_enabled", true);

    return true;
}

} // namespace marslite_navigation