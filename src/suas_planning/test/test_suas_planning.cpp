#include "../include/PlanningPoints.hpp"
#include <gtest/gtest.h>
#include <vector>
#include <cstdint>
#include <memory>
#include <iostream>

#define MAP_DIM 100

class CircularObstacleTest : public testing::Test
{
protected:
    void SetUp() override
    {
        x = 8;
        y = 8;
        radius = 2;
        map.resize(MAP_DIM * MAP_DIM);
        test_obst = suas_planning::CircularObstacle((double)x, (double)y, (double)radius);
        mmi = std::make_unique<suas_planning::MapMetaInfo>(map, MAP_DIM, MAP_DIM);
    }

    void TearDown() override
    {

    }

    std::vector<int8_t> map;
    std::unique_ptr<suas_planning::MapMetaInfo> mmi;
    suas_planning::CircularObstacle test_obst;
    int x;
    int y;
    int radius;
    const int8_t reference[64] = {
        0,0,0,1,1,0,0,0,
        0,0,1,0,0,1,0,0,
        0,1,0,0,0,0,1,0,
        1,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,1,
        0,1,0,0,0,0,1,0,
        0,0,1,0,0,1,0,0,
        0,0,0,1,1,0,0,0,
    };
};

TEST_F(CircularObstacleTest, TestCheckBounds)
{
    int x_min = x - radius;
    int x_max = x + radius;
    int y_min = y - radius;
    int y_max = y + radius;

    EXPECT_EQ(x_min, test_obst.GetMinX());
    EXPECT_EQ(x_max, test_obst.GetMaxX());
    EXPECT_EQ(y_min, test_obst.GetMinY());
    EXPECT_EQ(y_max, test_obst.GetMaxY());
}

TEST_F(CircularObstacleTest, TestRasterize)
{
    test_obst.PlotObstacle(map, *mmi);
    std::cout << "dump map" << std::endl;
    for (int i = 0; i < MAP_DIM; i++)
    {
        for (int j = 0; j < MAP_DIM; j++)
        {
            std::cout << (int)map[i * MAP_DIM + j] << " ";
        }
        std::cout << std::endl;
    }
    int n = 0;
    int hit = 0;
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if ((int)map[i * 8 + j] == 1 && (int)(100 * reference[n]) == 1)
            {
                hit++;
            }
            n++;
            std::cout << (int)map[i * 8 + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "hit: " << hit << "/" << "16" << std::endl;
    EXPECT_EQ(hit, 16);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
