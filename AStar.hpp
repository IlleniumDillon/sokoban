#include <list>
#include <algorithm>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "robot.hpp"

using namespace Eigen;
using namespace cv;

class robotPathPoint
{
public:
    Vector2i move;
    Robot::Action action;
};

class robotNode
{
public:
    robotNode(Vector2i p) 
    {
        position = p;
        parent = nullptr;
        g = std::numeric_limits<int>::max();
        h = 0;
        f = std::numeric_limits<int>::max();
        id = 0;
    }
    Vector2i position;
    robotNode* parent;
    int g;
    int h;
    int f;
    int id;
    multimap<int, robotNode*>::iterator it;
};
typedef robotNode* robotNodePtr;
class boxNode
{   
public:
    Vector2i position;
    boxNode* parent;
    int g;
    int h;
    int f;
    Robot::Action action;
    Vector2i currRobotPos;
};

class RobotAStar
{
public:
    RobotAStar() {}
    ~RobotAStar()
    {
        if(gridMap != nullptr)
        {
            for(int i = 0; i < map.rows; i++)
            {
                delete[] gridMap[i];
            }
            delete[] gridMap;
        }
    }
    void setGridMap(Mat& map_)
    {
        map = map_.clone();
        if(gridMap != nullptr)
        {
            for(int i = 0; i < map.rows; i++)
            {
                delete[] gridMap[i];
            }
            delete[] gridMap;
        }
        gridMap = new robotNodePtr*[map.rows];
        for(int i = 0; i < map.rows; i++)
        {
            gridMap[i] = new robotNodePtr[map.cols];
            for(int j = 0; j < map.cols; j++)
            {
                gridMap[i][j] = new robotNode(Vector2i(j, i));
            }
        }
    }
    void getNeighbors(robotNodePtr current, vector<robotNodePtr>& neighbors, vector<int>& edgeCost)
    {
        neighbors.clear();
        edgeCost.clear();
        Vector2i directions[4] = {Vector2i(0, 1), Vector2i(0, -1), Vector2i(1, 0), Vector2i(-1, 0)};
        for(int i = 0; i < 4; i++)
        {
            Vector2i next = current->position + directions[i];
            if(next.x() < 0 || next.x() >= map.cols || next.y() < 0 || next.y() >= map.rows)
                continue;
            if(map.at<uint8_t>(next.y(), next.x()) == 1)
                continue;
            neighbors.push_back(gridMap[next.y()][next.x()]);
            edgeCost.push_back(1);
        }
    }
    int graphSearch(Vector2i start, Vector2i goal, Mat& map)
    {
        //map: 0 free, 1 occupied
        setGridMap(map);

        std::multimap<int, robotNode*> openList;
        robotNodePtr startPtr = new robotNode(start);
        robotNodePtr goalPtr = new robotNode(goal);
        startPtr->g = 0;
        startPtr->h = (start - goal).norm();
        startPtr->f = startPtr->g + startPtr->h;
        startPtr->id = 1;
        openList.insert(std::pair<int, robotNode*>(startPtr->f, startPtr));

        int tentative_gScore;
        int numIter = 0;
        robotNodePtr currentPtr = nullptr;
        robotNodePtr neighborPtr = nullptr;

        vector<robotNodePtr> neighbors;
        vector<int> edgeCost;

        while(!openList.empty())
        {
            numIter++;
            currentPtr = openList.begin()->second;
            if(currentPtr->position == goal)
            {
                while(currentPtr->parent != nullptr)
                {
                    robotPathPoint pathPoint;
                    pathPoint.move = currentPtr->position - currentPtr->parent->position;
                    path.push_back(pathPoint);
                    currentPtr = currentPtr->parent;
                }
                //reverse path
                std::reverse(path.begin(), path.end());
                return currentPtr->g;
            }
            openList.erase(openList.begin());
            currentPtr->id = -1;

            getNeighbors(currentPtr, neighbors, edgeCost);

            for(int i = 0; i < neighbors.size(); i++)
            {
                neighborPtr = neighbors[i];
                if(neighborPtr->id == -1)
                    continue;
                tentative_gScore = currentPtr->g + edgeCost[i];
                if(neighborPtr->id == 0)
                {
                    neighborPtr->id = 1;
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->h = (neighborPtr->position - goal).norm();
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->it = openList.insert(std::pair<int, robotNode*>(neighborPtr->f, neighborPtr));
                }
                else if(tentative_gScore < neighborPtr->g)
                {
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    openList.erase(neighborPtr->it);
                    neighborPtr->it = openList.insert(std::pair<int, robotNode*>(neighborPtr->f, neighborPtr));
                }
            }
        }
        return -1;
    }

    vector<robotPathPoint> getPath()
    {
        return path;
    }

    Mat map;
    robotNodePtr** gridMap = nullptr;
    vector<robotPathPoint> path;
};