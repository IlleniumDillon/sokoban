#include <iostream>
#include <matplotlibcpp.h>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "robot.hpp"
#include "world.hpp"

using namespace Eigen;
namespace plt = matplotlibcpp;

enum : uint8_t
{
    FREE = 0,
    OCCUPIED = 1,
    INTERACTION = 2,
    UNKNOWN = 3
};

constexpr int mapWidth = 21;
constexpr int mapHeight = 16;

uint8_t map_[mapHeight][mapWidth] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1},
    {1,0,0,1,1,1,1,1,1,1,0,0,1,0,0,2,0,0,0,0,1},
    {1,0,0,0,0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,1},
    {1,0,0,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,1,1,0,0,0,0,0,0,0,1,0,0,1,0,0,1},
    {1,0,0,0,0,0,1,1,0,0,1,0,0,1,1,1,1,1,0,0,1},
    {1,0,0,1,0,0,1,0,0,0,0,1,1,0,0,0,1,0,0,0,1},
    {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
};

int main()
{
    std::vector<Box> boxes;
    for (int i = 0; i < mapHeight; i++) 
    {
        for (int j = 0; j < mapWidth; j++) 
        {
            if (map_[i][j] == OCCUPIED) 
            {
                boxes.push_back(Box(Vector2i(j, i), "box" + std::to_string(boxes.size()), true));
            }
            else if (map_[i][j] == INTERACTION) 
            {
                boxes.push_back(Box(Vector2i(j, i), "box" + std::to_string(boxes.size()), false));
            }
        }
    }

    Task task(Vector2i(10, 7), "box36");
    Robot robot(Vector2i(10, 7), "robot0");
    robot.getTask(task);

    World world(mapWidth, mapHeight);

    for (auto& box : boxes) 
    {
        world.addBox(box);
    }

    world.addRobot(robot);

    while(true)
    {
        world.update();
        world.draw();

        char input = cv::waitKey(0);
        if(input == 27)
            break;
        else if(input == ' ')
            continue;
        else if(input == 'w')
            world.robots[0].move(Vector2i(0, -1));
        else if(input == 's')
            world.robots[0].move(Vector2i(0, 1));
        else if(input == 'a')
            world.robots[0].move(Vector2i(-1, 0));
        else if(input == 'd')
            world.robots[0].move(Vector2i(1, 0));
        else if(input == 'p')
            world.robots[0].setAction(Robot::PUSH);
        else if(input == 'l')
            world.robots[0].setAction(Robot::PULL);
        else if(input == 'n')
            world.robots[0].setAction(Robot::NOACTION);
        else
            continue;
    }

    return 0;
}