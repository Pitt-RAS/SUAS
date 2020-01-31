#include "PlanningPoints.hpp"

namespace suas_planning {

CircularObstacle::CircularObstacle() {
    center_x_ = 0;
    center_y_= 0;
    radius_ = 0;
}

CircularObstacle::CircularObstacle(double x, double y, double radius) {
    center_x_ = (int) round(x);
    center_y_ = (int) round(y);
    radius_ = (int) round(radius);
}


// expand radius of obstacle so we can treat vehicle a point object
// preferentially rounding up to make sure we dont stray in.
int CircularObstacle::ExpandSize(double vehicle_radius) {
    radius_ += (int) ceil(vehicle_radius);
    return radius_;
}

// Implementation of the midpoint circle algorithm; Taken/Adapted from Wikipedia/Rosetta Code
// Assuming row major ordering for map and that the map is sufficiently large to rasterize
// entire obstacles, i.e. no partial shapes
void CircularObstacle::PlotObstacle(std::vector<int8_t>& map, MapMetaInfo map_meta) {
    int f = 1 - radius_;
    int df_x = 0;
    int df_y = -2 * radius_;
    int x = 0;
    int y = radius_;

    while (x < y) {
        if (f >= 0) {
            y--;
            df_y += 2;
            f += df_y;
        }
        x++;
        df_x += 2;
        f += df_x + 1;

        PlotPointProbability(map, center_x_ + x, center_y_ + y, map_meta);
        PlotPointProbability(map, center_x_ - x, center_y_ + y, map_meta);
        PlotPointProbability(map, center_x_ + x, center_y_ - y, map_meta);
        PlotPointProbability(map, center_x_ - x, center_y_ - y, map_meta);
        PlotPointProbability(map, center_x_ + y, center_y_ + x, map_meta);
        PlotPointProbability(map, center_x_ - y, center_y_ + x, map_meta);
        PlotPointProbability(map, center_x_ + y, center_y_ - x, map_meta);
        PlotPointProbability(map, center_x_ - y, center_y_ - x, map_meta);
    }

}

// fills point with probablitiy value
// Note due to preference for x axis to be horizontal, the indexing into the
// 2d array is [row][col]; [y][x]
// Returns -1 if unable to plot plot
int CircularObstacle::PlotPointProbability(std::vector<int8_t>& map, int x, int y, MapMetaInfo map_meta, int8_t prob) {
    if (!Obstacle::CheckGridBounds(map, x, y, map_meta)) {
        return -1;
    }
    int index = Obstacle::GetLinearIndex(y, x, map_meta.width_);
    map[index] = prob;
    return index;
}

int CircularObstacle::GetMinX() {
    return center_x_ - radius_;
}

int CircularObstacle::GetMaxX() {
    return center_x_ + radius_;
}

int CircularObstacle::GetMinY() {
    return center_y_ - radius_;
}

int CircularObstacle::GetMaxY() {
    return center_y_ + radius_;
}

}
