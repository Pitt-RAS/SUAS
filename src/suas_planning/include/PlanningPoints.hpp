#ifndef SUAS_POINTS_H
#define SUAS_POINTS_H

#include <cinttypes>
#include <cmath>
#include <vector>

namespace suas_planning {

class MapMetaInfo {
  public:
    MapMetaInfo();  // temporary until I figure out a way to make this private and not have a class call this
    MapMetaInfo(std::vector<int8_t> &map, unsigned int width, unsigned int height);
    unsigned int width_;
    unsigned int height_;
    std::vector<int8_t> &map_for_;
};

class Obstacle {
  public:
    Obstacle();
    Obstacle(double x, double y);
    static bool CheckGridBounds(std::vector<int8_t> &map, int x, int y, MapMetaInfo map_meta);
    static bool IsFree(std::vector<int8_t> &map, int x, int y, MapMetaInfo map_meta);
    static inline int GetLinearIndex(int row, int col, int columns) { return (row * columns) + col; };
    virtual void PlotObstacle(std::vector<int8_t> &map, MapMetaInfo map_meta);
    virtual int ExpandSize(double vehicle_radius);
    virtual int GetMinX();
    virtual int GetMaxX();
    virtual int GetMinY();
    virtual int GetMaxY();
    int center_x_;
    int center_y_;
    virtual ~Obstacle();
};

class CircularObstacle : public Obstacle {
  public:
    CircularObstacle();
    CircularObstacle(double x, double y, double radius);
    void PlotObstacle(std::vector<int8_t> &map, MapMetaInfo map_meta);
    int ExpandSize(double vehicle_radius);
    int GetMinX();
    int GetMaxX();
    int GetMinY();
    int GetMaxY();
    int radius_;

  private:
    int PlotPointProbability(std::vector<int8_t> &map, int x, int y, MapMetaInfo map_meta, int8_t prob = 100);
};

class Waypoint {
  public:
    Waypoint(double x, double y);
    int x_;
    int y_;

  private:
    Waypoint();
};

bool operator==(const Waypoint &lhs, const Waypoint &rhs);
bool operator!=(const Waypoint &lhs, const Waypoint &rhs);

}  // namespace suas_planning

#endif
