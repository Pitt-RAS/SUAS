#include "PlanningPoints.hpp"

namespace suas_planning {
    Obstacle::Obstacle() {
        center_x_ = 0;
        center_y_ = 0;
    }

    Obstacle::Obstacle(double x, double y) {
        center_x_ = (int) round(x);
        center_y_ = (int) round(y);
    }
    static inline int GetLinearIndex(int row, int col, int columns) {
        return (row * columns) + col;
    }
}