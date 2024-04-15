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

class WorldNode
{
public:
    vector<Vector2i> position;
    Vector2i robotPosition;

    WorldNode* parent;
    int g;
    int h;
    int f;
    int id;
    multimap<int, WorldNode*>::iterator it;
    
    vector<robotPathPoint> path;
};

typedef WorldNode* WorldNodePtr;
class Sokoban
{
public:
    Sokoban() {}
    void setWorld(World* world_)
    {
        world = world_;
        findPath(world->robots[0].position, Vector2i(1, 1), world->boxes[36].position, Vector2i(1, 1));       
    }
    void update()
    {
        if(index < world->robots[0].path.size())
        {
            world->robots[0].setAction(world->robots[0].path[index].action);
            world->robots[0].move(world->robots[0].path[index].move);
            index++;
        }
        else
        {
            world->robots[0].setAction(Robot::Action::NOACTION);
            world->robots[0].move(Vector2i(0, 0));
        }
    }
    void fixTaskList()
    {
        for (auto& task : world->taskList)
        {
            task.robotName = "robot0";
        }
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
        generateMap(map, ignore);
        int cost = boxAStar.graphSearch(box_start,box_end, robot_start, map);
        std::cout << "Cost: " << cost << std::endl;
        world->robots[0].path = boxAStar.getPathList();
    }
    int graphSearch(vector<Task>& startAndGoal, Robot& robot)
    {
        pathList.clear();
        vector<Vector2i> boxStarts;
        vector<Vector2i> boxGoals;
        for (auto& task : startAndGoal)
        {
            auto box = find_if(world->boxes.begin(), world->boxes.end(), [&](Box& box) { return box.name == task.boxName; });
            boxStarts.push_back(box->position);
            boxGoals.push_back(task.position);
        }
        std::multimap<int, WorldNode> openList;

        WorldNode startNode;
        startNode.position = boxStarts;
        startNode.robotPosition = robot.position;
        startNode.g = 0;
        startNode.h = 0;
        startNode.f = 0;
        startNode.parent = nullptr;
        startNode.id = 1;
        startNode.path = pathList; 
    }

    World* world;
    RobotAStar robotAStar;
    BoxAStar boxAStar;
    Mat map;
    vector<robotPathPoint> pathList;
    int index = 0;
};