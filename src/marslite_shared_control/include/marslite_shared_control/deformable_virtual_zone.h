#ifndef DEFORMABLE_VIRTUAL_ZONE_H_
#define DEFORMABLE_VIRTUAL_ZONE_H_

#include "marslite_shared_control/virtual_zone.h"

#include <dynamic_reconfigure/server.h>
#include "marslite_shared_control/VirtualZoneConfig.h"

namespace marslite_shared_control {

class DeformableVirtualZone : public VirtualZone {
public:
    explicit DeformableVirtualZone(const ros::NodeHandle& nh = ros::NodeHandle());
    
protected:
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) override;

private:
    // dynamic reconfigure
    void reconfigureCB(marslite_shared_control::VirtualZoneConfig &config, uint32_t level);
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>* server_;
    dynamic_reconfigure::Server<marslite_shared_control::VirtualZoneConfig>::CallbackType f_;
};

} // namespace marslite_shared_control

#endif  // #ifndef DEFORMABLE_VIRTUAL_ZONE_H_