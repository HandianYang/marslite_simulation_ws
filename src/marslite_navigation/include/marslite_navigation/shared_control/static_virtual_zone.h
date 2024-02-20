#ifndef STATIC_VIRTUAL_ZONE_H_
#define STATIC_VIRTUAL_ZONE_H_

#include "marslite_navigation/shared_control/virtual_zone.h"

#include <dynamic_reconfigure/server.h>
#include "marslite_navigation/VirtualZoneConfig.h"

namespace marslite_navigation {

namespace shared_control {

class StaticVirtualZone : public VirtualZone {
public:
    explicit StaticVirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    
protected:
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) override;

private:
    // dynamic reconfigure
    void reconfigureCB(marslite_navigation::VirtualZoneConfig& config, uint32_t level);
    dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>* server_;
    dynamic_reconfigure::Server<marslite_navigation::VirtualZoneConfig>::CallbackType f_;

};

} // namespace shared_control

} // namespace marslite_navigation

#endif  // #ifndef STATIC_VIRTUAL_ZONE_H_