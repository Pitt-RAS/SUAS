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

    // Implementation of the midpoint circle algorithm; Taken/Adapted from Wikipedia/Rosetta Code
    // Assuming row major ordering for map and that the map is sufficiently large to rasterize
    // entire obstacles, i.e. no partial shapes
    void CircularObstacle::PlotObstacle(std::vector<int8_t>& map, int width, int height) {
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

            PlotPointProbability(map, center_x_ + x, center_y_ + y, width);
            PlotPointProbability(map, center_x_ - x, center_y_ + y, width);
            PlotPointProbability(map, center_x_ + x, center_y_ - y, width);
            PlotPointProbability(map, center_x_ - x, center_y_ - y, width);
            PlotPointProbability(map, center_x_ + y, center_y_ + x, width);
            PlotPointProbability(map, center_x_ - y, center_y_ + x, width);
            PlotPointProbability(map, center_x_ + y, center_y_ - x, width);
            PlotPointProbability(map, center_x_ - y, center_y_ - x, width);
        }

    }
    // fills point with probablitiy value
    // Note due to preference for x axis to be horizontal, the indexing into the 
    // 2d array is [row][col]; [y][x] 
    int PlotPointProbablity(std::vector<int8_t>& map, int x, int y, int map_width, int8_t prob) {
        int index = Obstacle::GetLinearIndex(y, x, map_width);
        map[index] = prob;
        return index;
    }

}
