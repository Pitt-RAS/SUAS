#include "PlanningPoints.hpp"

namespace suas_planning {

Obstacle::Obstacle() :
    center_x_(0),
    center_y_(0) {}

Obstacle::Obstacle(double x, double y) :
    center_x_((int) round(x)),
    center_y_((int) round(y)) {}

inline int Obstacle::GetLinearIndex(int row, int col, int columns) {
    return (row * columns) + col;
}

bool Obstacle::CheckGridBounds(std::vector<int8_t>& map, int x, int y, MapMetaInfo map_meta) {
    int width = map_meta.width_;
    int height = map_meta.height_;
    if (x < 0 || y < 0) {
        return false;
    } else if (x >= width || y >= height) {
        return false;
    }
    return true;
}

int Obstacle::ExpandSize(double vehicle_radius) {
    return 0;
}

int Obstacle::GetMinX() {
    return center_x_;
}

int Obstacle::GetMaxX() {
    return center_x_;
}

int Obstacle::GetMinY() {
    return center_y_;
}

int Obstacle::GetMaxY() {
    return center_y_;
}


}