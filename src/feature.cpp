#include <feature.h>

namespace lidar_selection {

uint64_t Feature::feat_counter_ = 0; 
void Feature::resetStatics()
{
  feat_counter_ = 0;
}

} // namespace svo
