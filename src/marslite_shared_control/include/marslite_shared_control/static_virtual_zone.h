#ifndef STATIC_VIRTUAL_ZONE_H_
#define STATIC_VIRTUAL_ZONE_H_

#include "marslite_shared_control/virtual_zone.h"

namespace marslite_shared_control {

class StaticVirtualZone : public VirtualZone {
public:
    explicit StaticVirtualZone(void);
    
protected:
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr = nullptr) override;
};

} // namespace marslite_shared_control

#endif  // #ifndef STATIC_VIRTUAL_ZONE_H_