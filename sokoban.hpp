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
        findPath(world->robots[0].position, Vector2i(1, 1), world->boxes[36].position, world->robots[0].position);       
    }
    void update()
    {
        if(index < path.size())
        {
            world->robots[0].setAction(path[index].action);
            world->robots[0].move(path[index].move);
            index++;
        }
        else
        {
            world->robots[0].setAction(Robot::Action::NOACTION);
            world->robots[0].move(Vector2i(0, 0));
        }
    }
    std::vector<Task> fixTaskList(std::vector<Task> taskList)
    {
        return taskList;
    }
    void generateMap(Mat& map, vector<string> ignore)
    {
        map = Mat::zeros(world->height, world->width, CV_8UC1);
        for (auto& box : world->boxes)
        {
            if(find(ignore.begin(), ignore.end(), box.name) == ignore.end())
            {
                map.at<uchar>(box.position.y(), box.position.x()) = 1;
            }
            else
            {
                map.at<uchar>(box.position.y(), box.position.x()) = 0;
            }
        }
        for (auto& robot : world->robots)
        {
            if(find(ignore.begin(), ignore.end(), robot.name) == ignore.end())
            {
                map.at<uchar>(robot.position.y(), robot.position.x()) = 1;
            }
            else
            {
                map.at<uchar>(robot.position.y(), robot.position.x()) = 0;
            }
        }
    }
    void findPath(Vector2i robot_start, Vector2i robot_end, Vector2i box_start, Vector2i box_end)
    {
        std::vector<string> ignore{"robot0","box36"};
        Mat map;
        generateMap(map, ignore);
        int cost = boxAStar.graphSearch(box_start,box_end, robot_start, map);
        std::cout << "Cost: " << cost << std::endl;
        path = boxAStar.getPathList();
    }

    World* world;
    RobotAStar robotAStar;
    BoxAStar boxAStar;
    vector<robotPathPoint> path;
    int index = 0;
};