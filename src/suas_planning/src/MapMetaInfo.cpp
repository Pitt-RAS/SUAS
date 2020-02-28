#include "PlanningPoints.hpp"

namespace suas_planning {

MapMetaInfo::MapMetaInfo(std::vector<int8_t> &map, unsigned int width, unsigned int height)
    : width_(width), height_(height), map_for_(map) {}

}  // namespace suas_planning
