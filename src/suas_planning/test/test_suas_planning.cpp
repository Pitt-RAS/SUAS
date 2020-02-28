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
        x = 32;
        y = 32;
        radius = 12;
        map.resize(MAP_DIM * MAP_DIM);
        test_obst = suas_planning::CircularObstacle((double)x, (double)y, (double)radius);
        mmi = std::make_unique<suas_planning::MapMetaInfo>(map, MAP_DIM, MAP_DIM);
    }

    void TearDown() override
    {

    }

    // members
    std::vector<int8_t> map;
    std::unique_ptr<suas_planning::MapMetaInfo> mmi;
    suas_planning::CircularObstacle test_obst;
    int x;
    int y;
    int radius;
    int reference[25*25] = {
        0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
        0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
        0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
        0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
        0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
        0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,
        0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0
    };
};

TEST_F(CircularObstacleTest, TestGetLinearIndex)
{
    int expected_index = y * MAP_DIM + x;
    EXPECT_EQ(expected_index, suas_planning::Obstacle::GetLinearIndex(y, x, MAP_DIM));
    EXPECT_EQ(MAP_DIM, suas_planning::Obstacle::GetLinearIndex(1, 0, MAP_DIM));
    EXPECT_EQ(0, suas_planning::Obstacle::GetLinearIndex(0, 0, MAP_DIM));
}

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

TEST_F(CircularObstacleTest, TestPosition)
{
    
}

TEST_F(CircularObstacleTest, TestRasterize)
{
    /* no equality checking, but circles can be visually confirmed */

    test_obst.PlotObstacle(map, *mmi);
    int start_x = x - radius;
    int start_y = y - radius;
    int index = suas_planning::Obstacle::GetLinearIndex(y, x, MAP_DIM);

    int ri = 0;
    for (int i = 0; i < radius * 2 + 1; i++)
    {
        for (int j = 0; j < radius * 2 + 1; j++)
        {
            int v = (int)map[suas_planning::Obstacle::GetLinearIndex(start_y + i, start_x + j, MAP_DIM)] / 100;
            EXPECT_EQ(v, reference[ri]);
            //std::cout << v << " ";
            ri++;
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
