#include <list>
#include <algorithm>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "robot.hpp"

using namespace Eigen;
using namespace cv;

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
    boxNode(Vector2i p, Vector2i robotPos, Robot::Action action_ = Robot::NOACTION)
    {
        position = p;
        parent = nullptr;
        g = std::numeric_limits<int>::max();
        h = 0;
        f = std::numeric_limits<int>::max();
        currRobotPos = robotPos;
        action = action_;
    }
    Vector2i position;
    boxNode* parent;
    int g;
    int h;
    int f;
    Robot::Action action;
    Vector2i currRobotPos;
    int id = 0;
    multimap<int, boxNode*>::iterator it;
    vector<robotPathPoint> path;    
};

typedef boxNode* boxNodePtr;

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
        path.clear();
        pathPos.clear();
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
                int cost = currentPtr->g;
                while(currentPtr->parent != nullptr)
                {
                    robotPathPoint pathPoint;
                    pathPoint.move = currentPtr->position - currentPtr->parent->position;
                    path.push_back(pathPoint);
                    pathPos.push_back(currentPtr->position);
                    currentPtr = currentPtr->parent;
                }
                //reverse path
                std::reverse(path.begin(), path.end());
                pathPos.push_back(start);
                std::reverse(pathPos.begin(), pathPos.end());
                return cost;
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
    vector<Vector2i> pathPos;
};

class BoxAStar
{
public:
    BoxAStar() {}
    ~BoxAStar()
    {
        if(gridMap != nullptr)
        {
            for(int i = 0; i < map.rows; i++)
            {
                for(int j = 0; j < map.cols; j++)
                {
                    delete[] gridMap[i][j];
                }
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
                for(int j = 0; j < map.cols; j++)
                {
                    delete[] gridMap[i][j];
                }
                delete[] gridMap[i];
            }
            delete[] gridMap;
        }
        gridMap = new boxNodePtr**[map.rows];
        for(int i = 0; i < map.rows; i++)
        {
            gridMap[i] = new boxNodePtr*[map.cols];
            for(int j = 0; j < map.cols; j++)
            {
                gridMap[i][j] = new boxNodePtr[4];
                Vector2i directions[4] = {Vector2i(0, 1), Vector2i(0, -1), Vector2i(1, 0), Vector2i(-1, 0)};
                for(int k = 0; k < 4; k++)
                {
                    Vector2i next = Vector2i(j, i) + directions[k];
                    gridMap[i][j][k] = new boxNode(Vector2i(j, i), Vector2i(next.x(), next.y()));
                }
            }
        }
    }
    void getNeighbors(boxNodePtr current, vector<boxNodePtr>& neighbors, vector<int>& edgeCost, vector<vector<robotPathPoint>>& minipath)
    {
        minipath.clear();
        neighbors.clear();
        edgeCost.clear();

        boxNodePtr* elseNeighbors = gridMap[current->position.y()][current->position.x()];
        for(int i = 0; i < 4; i++)
        {
            if(elseNeighbors[i]->currRobotPos == current->currRobotPos)
            {
                Vector2i move = current->position - current->currRobotPos;
                Vector2i new_pos = current->position + move;
                
                if (current->position.x() < 0 || current->position.x() >= map.cols || current->position.y() < 0 || current->position.y() >= map.rows)
                {
                    continue;
                }
                if (new_pos.x() < 0 || new_pos.x() >= map.cols || new_pos.y() < 0 || new_pos.y() >= map.rows)
                {
                    continue;
                }
                if (map.at<uchar>(new_pos.y(), new_pos.x()) == 1)
                {
                    continue;
                }
                if (map.at<uchar>(current->currRobotPos.y(), current->currRobotPos.x()) == 1)
                {
                    continue;
                }
                gridMap[new_pos.y()][new_pos.x()][i]->action = Robot::Action::PUSH;
                robotPathPoint point;
                point.move = move;
                point.action = Robot::Action::PUSH;
                std::vector<robotPathPoint> temppath;
                temppath.push_back(point);
                minipath.push_back(temppath);
                neighbors.push_back(gridMap[new_pos.y()][new_pos.x()][i]);
                edgeCost.push_back(1);
            }
            else
            {
                if (elseNeighbors[i]->position.x() < 0 || elseNeighbors[i]->position.x() >= map.cols || elseNeighbors[i]->position.y() < 0 || elseNeighbors[i]->position.y() >= map.rows)
                {
                    continue;
                }
                if (elseNeighbors[i]->currRobotPos.x() < 0 || elseNeighbors[i]->currRobotPos.x() >= map.cols || elseNeighbors[i]->currRobotPos.y() < 0 || elseNeighbors[i]->currRobotPos.y() >= map.rows)
                {
                    continue;
                }
                if (map.at<uchar>(elseNeighbors[i]->position.y(), elseNeighbors[i]->position.x()) == 1)
                {
                    continue;
                }
                if (map.at<uchar>(elseNeighbors[i]->currRobotPos.y(), elseNeighbors[i]->currRobotPos.x()) == 1)
                {
                    continue;
                }
                if (abs(elseNeighbors[i]->currRobotPos.x() - current->currRobotPos.x()) == 2 || abs(elseNeighbors[i]->currRobotPos.y() - current->currRobotPos.y()) == 2)
                {
                    continue;
                }
                //robot astar
                Mat tempMap = map.clone();
                tempMap.at<uchar>(elseNeighbors[i]->position.y(),elseNeighbors[i]->position.x()) = 1;
                int cost = robotAstar.graphSearch(current->currRobotPos,elseNeighbors[i]->currRobotPos,tempMap);
                if(cost < 0) continue;
                elseNeighbors[i]->action = Robot::Action::NOACTION;
                auto temppath = robotAstar.getPath();  
                for (auto& pathPoint : temppath)
                {
                    pathPoint.action = Robot::Action::NOACTION;
                }
                minipath.push_back(temppath);
                neighbors.push_back(elseNeighbors[i]);
                edgeCost.push_back(cost);
            }
        }
    }
    int getHeuristic(boxNodePtr current, Vector2i goal)
    {
        RobotAStar tempSolver;
        int cost = tempSolver.graphSearch(current->position, goal, map);
        return cost;
    }
    int graphSearch(Vector2i start, Vector2i goal, Vector2i robotStart, Mat& map)
    {
        pathList.clear();
        //map:0 free, 1 occupied
        setGridMap(map);

        // RobotAStar tempAStar;
        // int cost = tempAStar.graphSearch(start, goal, map);
        // if(cost < 0) return -1;
        // auto boxNativePath = tempAStar.getPath();
        // boxNativePathPos = tempAStar.pathPos;
        // auto firstMove = boxNativePath[0].move;
        // auto firstRobotPos = start - firstMove;

        std::multimap<int, boxNode*> openList;

        boxNodePtr* startStates = gridMap[start.y()][start.x()];
        for (int i = 0; i < 4 ; i++)
        {
            boxNodePtr startPtr = startStates[i];
            Mat tempMap = map.clone();
            tempMap.at<uchar>(startPtr->position.y(),startPtr->position.x()) = 1;
            int cost = robotAstar.graphSearch(robotStart,startPtr->currRobotPos,tempMap);
            if(cost < 0) continue;
            startStates[i]->g = cost;
            startStates[i]->h = getHeuristic(startStates[i], goal);
            startStates[i]->f = startStates[i]->g + startStates[i]->h;
            startStates[i]->id = 1;
            startStates[i]->parent = nullptr;
            startStates[i]->path = robotAstar.getPath();
            for (auto& pathPoint : startStates[i]->path)
            {
                pathPoint.action = Robot::Action::NOACTION;
            }
            openList.insert(std::pair<int, boxNode*>(startPtr->f, startPtr));
        }
        
        int tentative_gScore;
        int numIter = 0;
        boxNodePtr currentPtr = nullptr;
        boxNodePtr neighborPtr = nullptr;

        vector<boxNodePtr> neighbors;
        vector<int> edgeCost;
        vector<vector<robotPathPoint>> minipath;

        while(!openList.empty())
        {
            numIter++;
            currentPtr = openList.begin()->second;
            if(currentPtr->position == goal)
            {
                int cost = currentPtr->g;
                vector<boxNodePtr> path;
                while (currentPtr != nullptr)
                {
                    path.push_back(currentPtr);
                    currentPtr = currentPtr->parent;
                }
                reverse(path.begin(), path.end());
                // for (auto& node : path)
                // {
                //     cout << node->position.x() << " " << node->position.y() << endl;
                // }
                for (auto& node : path)
                {
                    for (auto& pathPoint : node->path)
                    {
                        pathList.push_back(pathPoint);
                        //cout << pathPoint.move.x() << " " << pathPoint.move.y() << " " << pathPoint.action << endl;
                    }
                }
                // cout << "iter: " << numIter << endl;
                return cost;
            }
            openList.erase(openList.begin());
            currentPtr->id = -1;

            getNeighbors(currentPtr, neighbors, edgeCost, minipath);
            // cout << "neighbors: " << neighbors.size() << endl;

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
                    //neighborPtr->h = 0;
                    neighborPtr->h = getHeuristic(neighborPtr, goal);
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->path = minipath[i];
                    neighborPtr->it = openList.insert(std::pair<int, boxNode*>(neighborPtr->f, neighborPtr));
                }
                else if(tentative_gScore < neighborPtr->g)
                {
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->path = minipath[i];
                    openList.erase(neighborPtr->it);
                    neighborPtr->it = openList.insert(std::pair<int, boxNode*>(neighborPtr->f, neighborPtr));
                }
            }
        }
        return -1;
    }

    vector<robotPathPoint> getPathList()
    {
        return pathList;
    }

    Mat map;
    boxNodePtr*** gridMap = nullptr;
    vector<robotPathPoint> pathList;
    RobotAStar robotAstar;
    vector<Vector2i> boxNativePathPos;
};