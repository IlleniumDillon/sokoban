#pragma once 

#include <iostream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "AStar.hpp"
#include "task.hpp"
#include "world.hpp"

using namespace Eigen;
using namespace cv;
using namespace std;

class Sokoban
{
public:
    Sokoban() {}
    void setWorld(World* world_)
    {
        world = world_;
    }
    void update()
    {
    }
    std::vector<Task> fixTaskList(std::vector<Task> taskList)
    {
        return taskList;
    }
    std::vector<Vector2i> findPath(Mat map, Vector2i box_start, Vector2i box_goal, Vector2i robot_start)
    {
        //todo: implement A* algorithm
        //todo: generate a path from box_start to box_goal
        //todo: generate a path of robot
        //todo: fix the path of robot
        AStar astar;
        return std::vector<Vector2i>{Vector2i(10,7), Vector2i(10,8), Vector2i(10,9)};
    }
    World* world;
};