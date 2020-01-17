#ifndef SUAS_POINTS_H
#define SUAS_POINTS_H

#include <cinttypes>

namespace suas_planning {
    class Obstacle {
    public:
        Obstacle(double x, double y);
        int32_t center_x_;
        int32_t center_y_;
        void PlotObstacle(int8_t& map);
    };

    class CircularObstacle: public Obstacle {
    public:
        CircularObstacle(double x, double y, double radius);
        int32_t radius_;
    };
        
} // namespace suas_planning

#endif
