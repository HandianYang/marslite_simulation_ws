#ifndef DEFORMABLE_VIRTUAL_ZONE_H_
#define DEFORMABLE_VIRTUAL_ZONE_H_

#include "marslite_shared_control/virtual_zone.h"

namespace marslite_shared_control {

class DeformableVirtualZone : public VirtualZone {
public:
    explicit DeformableVirtualZone(void);
    
protected:
    void calculateFields(const sensor_msgs::LaserScanConstPtr& laserPtr) override;
};

} // namespace marslite_shared_control

#endif  // #ifndef DEFORMABLE_VIRTUAL_ZONE_H_