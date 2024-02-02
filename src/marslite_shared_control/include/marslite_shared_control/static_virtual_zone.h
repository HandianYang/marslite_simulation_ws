#ifndef STATIC_VIRTUAL_ZONE_H_
#define STATIC_VIRTUAL_ZONE_H_

#include "marslite_shared_control/virtual_zone.h"

#include <dynamic_reconfigure/server.h>
#include "marslite_shared_control/VirtualZoneConfig.h"

namespace marslite_shared_control {

class StaticVirtualZone : public VirtualZone {
public:
    explicit StaticVirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    
protected:
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) override;

private:
    // dynamic reconfigure
    void reconfigureCB(marslite_shared_control::VirtualZoneConfig &config, uint32_t level);
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>* server_;
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>::CallbackType f_;

};

} // namespace marslite_shared_control

#endif  // #ifndef STATIC_VIRTUAL_ZONE_H_