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

class WorldNode;
typedef std::shared_ptr<WorldNode> WorldNodePtr; 
class WorldNode
{
public:
    vector<Vector2i> position;
    Vector2i robotPosition;

    WorldNodePtr parent;
    int g;
    int h;
    int f;
    int id;
    multimap<int, WorldNodePtr>::iterator it;
    
    vector<robotPathPoint> path;

    bool operator== (const WorldNode& node)
    {
        return position == node.position && robotPosition == node.robotPosition;
    }
    bool operator!= (const WorldNode& node)
    {
        return position != node.position || robotPosition != node.robotPosition;
    }
    bool operator<= (const WorldNode& node)
    {
        return position == node.position;
    }
};

class Sokoban
{
public:
    Sokoban() {}
    void setWorld(World* world_)
    {
        world = world_;
        //findPath(world->robots[0].position, Vector2i(1, 1), world->boxes[36].position, Vector2i(1, 1));
        std::vector<string> ignore{"robot0","box36", "box89"};
        //std::vector<string> ignore{"robot0","box36"};
        generateMap(map, ignore);
        int cost = graphSearch(world->taskList, world->robots[0]);  
        cout << "Cost: " << cost << endl;
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
    void getNeighbors(WorldNodePtr current, vector<WorldNodePtr>& neighbors, vector<int>& edgeCost, vector<vector<robotPathPoint>>& minipath, vector<WorldNodePtr>& closeList ,multimap<int, WorldNodePtr>& openList)
    {
        neighbors.clear();
        edgeCost.clear();
        minipath.clear();
        //fix map
        Mat tempMap = map.clone();
        for (int i = 0; i < current->position.size(); i++)
        {
            tempMap.at<uchar>(current->position[i].y(), current->position[i].x()) = 1;
        }
        //exclude current position
        int moveBoxId = -1;
        for (int i = 0; i < current->position.size(); i++)
        {
            if (current->position[i].x() < 0 || current->position[i].x() >= map.cols || current->position[i].y() < 0 || current->position[i].y() >= map.rows)
            {
                continue;
            }
            for (int j = 0; j < robotRelativePosition.size(); j++)
            {
                Vector2i robotPosition = current->position[i] + robotRelativePosition[j];
                if (robotPosition == current->robotPosition)
                {
                    moveBoxId = i;
                    break;
                }
            }
        }
        for (int i = 0; i < current->position.size(); i++)
        {
            int punish = 0;
            if (i == moveBoxId)
            {
                punish = 1;
            }
            else
            {
                punish = 10;
            }
            if (current->position[i].x() < 0 || current->position[i].x() >= map.cols || current->position[i].y() < 0 || current->position[i].y() >= map.rows)
            {
                continue;
            }
            for (int j = 0; j < robotRelativePosition.size(); j++)
            {
                Vector2i robotPosition = current->position[i] + robotRelativePosition[j];
                //not move box
                if (robotPosition == current->robotPosition)
                {
                    continue;
                }
                if (tempMap.at<uchar>(robotPosition.y(), robotPosition.x()) == 1)
                {
                    continue;
                }
                // //check if position is in closeList
                auto it_c = find_if(closeList.begin(), closeList.end(), [&](WorldNodePtr node) { return node->robotPosition == robotPosition && node->position == current->position; });
                if(it_c != closeList.end())
                {
                    minipath.push_back({});
                    edgeCost.push_back(0);
                    neighbors.push_back(*it_c);
                    continue;
                }
                
                //check if position is in openList
                auto it = find_if(openList.begin(), openList.end(), [&](pair<int, WorldNodePtr> node) { return node.second->robotPosition == robotPosition && node.second->position == current->position; });
                if(it != openList.end())
                {
                    RobotAStar tempAStar;
                    int cost = tempAStar.graphSearch(current->robotPosition, robotPosition, tempMap);
                    if (cost == -1)
                    {
                        continue;
                    }
                    auto tempPath = tempAStar.getPath();
                    for (auto& path : tempPath)
                    {
                        path.action = Robot::Action::NOACTION;
                    }
                    minipath.push_back(tempPath);
                    neighbors.push_back(it->second);
                    edgeCost.push_back(cost * punish);
                }
                //new node
                else
                {
                    WorldNodePtr neighbor = make_shared<WorldNode>();
                    neighbor->position = current->position;
                    neighbor->robotPosition = robotPosition;
                    neighbor->parent = nullptr;
                    neighbor->g = numeric_limits<int>::max();
                    neighbor->h = 0;
                    neighbor->f = numeric_limits<int>::max();
                    neighbor->id = 0;
                    neighbor->path.clear();

                    RobotAStar tempAStar;
                    int cost = tempAStar.graphSearch(current->robotPosition, robotPosition, tempMap);
                    if (cost == -1)
                    {
                        continue;
                    }

                    auto tempPath = tempAStar.getPath();
                    for (auto& path : tempPath)
                    {
                        path.action = Robot::Action::NOACTION;
                    }
                    minipath.push_back(tempPath);
                    neighbors.push_back(neighbor);
                    edgeCost.push_back(cost * punish);
                }

            }
        }
        //move box
        if (moveBoxId != -1)
        {
            Vector2i move = current->position[moveBoxId] - current->robotPosition;
            vector<Vector2i> tempPosition = current->position;
            tempPosition[moveBoxId] += move;
            Vector2i newRobotPosition = current->robotPosition + move;
            //check if boxPosition is valid
            if (tempMap.at<uchar>(tempPosition[moveBoxId].y(), tempPosition[moveBoxId].x()) == 1)
            {
                return;
            }
            //check if position is out of map
            if (tempPosition[moveBoxId].x() < 0 || tempPosition[moveBoxId].x() >= map.cols || tempPosition[moveBoxId].y() < 0 || tempPosition[moveBoxId].y() >= map.rows)
            {
                return;
            }
            // //check if position is in closeList
            auto it_c = find_if(closeList.begin(), closeList.end(), [&](WorldNodePtr node) { return node->robotPosition == newRobotPosition && node->position == tempPosition; });
            if(it_c != closeList.end())
            {
                minipath.push_back({});
                edgeCost.push_back(0);
                neighbors.push_back(*it_c);
                return;
            }
            //check if position is in openList
            auto it = find_if(openList.begin(), openList.end(), [&](pair<int, WorldNodePtr> node) { return node.second->robotPosition == current->robotPosition && node.second->position == tempPosition; });
            if(it != openList.end())
            {
                robotPathPoint point;
                point.move = move;
                point.action = Robot::Action::PUSH;
                std::vector<robotPathPoint> temppath;
                temppath.push_back(point);
                minipath.push_back(temppath);
                neighbors.push_back(it->second);
                edgeCost.push_back(1);
            }
            //new node
            else
            {
                WorldNodePtr neighbor = make_shared<WorldNode>();
                neighbor->position = tempPosition;
                neighbor->robotPosition = newRobotPosition;
                neighbor->parent = nullptr;
                neighbor->g = numeric_limits<int>::max();
                neighbor->h = 0;
                neighbor->f = numeric_limits<int>::max();
                neighbor->id = 0;
                neighbor->path.clear();

                robotPathPoint point;
                point.move = move;
                point.action = Robot::Action::PUSH;
                std::vector<robotPathPoint> temppath;
                temppath.push_back(point);
                minipath.push_back(temppath);
                neighbors.push_back(neighbor);
                edgeCost.push_back(1);
            }
        }
    }
    int getHeuristic(WorldNodePtr node, vector<Vector2i>& goal)
    {
        int cost = 0;
        for (int i = 0; i < node->position.size(); i++)
        {
            cost += abs(node->position[i].x() - goal[i].x()) + abs(node->position[i].y() - goal[i].y());
        }
        return cost;
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

        std::multimap<int, WorldNodePtr> openList;
        std::vector<WorldNodePtr> closeList;

        WorldNodePtr goalNode = make_shared<WorldNode>();
        goalNode->position = boxGoals;

        for (int i = 0; i < boxStarts.size(); i++)
        {
            for (int j = 0; j < robotRelativePosition.size(); j++)
            {
                WorldNodePtr startNode = make_shared<WorldNode>();
                startNode->position = boxStarts;
                startNode->robotPosition = boxStarts[i] + robotRelativePosition[j];
                startNode->parent = nullptr;
                Mat tempMap = map.clone();
                for (int k = 0; k < boxStarts.size(); k++)
                {
                    tempMap.at<uchar>(boxStarts[k].y(), boxStarts[k].x()) = 1;
                }
                int cost = robotAStar.graphSearch(robot.position, startNode->robotPosition, tempMap);
                if (cost == -1)
                {
                    continue;
                }
                startNode->g = cost;
                startNode->h = getHeuristic(startNode, boxGoals);
                startNode->f = startNode->g + startNode->h;
                startNode->id = 1;
                startNode->path = robotAStar.getPath();
                for (auto& path : startNode->path)
                {
                    path.action = Robot::Action::NOACTION;
                }
                startNode->it = openList.insert({startNode->f, startNode});
            }
        }

        int tentative_gScore = 0;
        int numIter = 0;
        WorldNodePtr currentPtr = nullptr;
        WorldNodePtr neighborPtr = nullptr;

        vector<WorldNodePtr> neighbors;
        vector<int> edgeCost;
        vector<vector<robotPathPoint>> minipath;

        while (!openList.empty())
        {
            numIter++;
            currentPtr = openList.begin()->second;
            if (*currentPtr <= *goalNode)
            {
                int cost = currentPtr->g;
                vector<WorldNodePtr> path;
                while (currentPtr != nullptr)
                {
                    path.push_back(currentPtr);
                    currentPtr = currentPtr->parent;
                }
                reverse(path.begin(), path.end());
                for (auto& node : path)
                {
                    for (auto& pathPoint : node->path)
                    {
                        pathList.push_back(pathPoint);
                        cout << pathPoint.move.x() << " " << pathPoint.move.y() << " " << pathPoint.action << endl;
                    }
                }
                cout << "iter: " << numIter << endl;
                robot.path = pathList;
                return cost;
            }
            openList.erase(openList.begin());
            currentPtr->id = -1;
            closeList.push_back(currentPtr);

            getNeighbors(currentPtr, neighbors, edgeCost, minipath, closeList, openList);
            //cout << "neighbors: " << neighbors.size() << endl;
            for (int i = 0; i < neighbors.size(); i++)
            {
                neighborPtr = neighbors[i];
                if (neighborPtr->id == -1)
                {
                    continue;
                }
                tentative_gScore = currentPtr->g + edgeCost[i];
                if (neighborPtr->id == 0)
                {
                    neighborPtr->id = 1;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->h = getHeuristic(neighborPtr, boxGoals);
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->path = minipath[i];
                    neighborPtr->it = openList.insert({neighborPtr->f, neighborPtr});
                }
                else if(tentative_gScore < neighborPtr->g)
                {
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->path = minipath[i];
                    openList.erase(neighborPtr->it);
                    neighborPtr->it = openList.insert({neighborPtr->f, neighborPtr});
                }
            }
            
        }
        return -1;
    }

    World* world;
    RobotAStar robotAStar;
    BoxAStar boxAStar;
    //map: 0 free, 1 occupied
    //ignore moving box and robot
    Mat map;
    vector<robotPathPoint> pathList;
    int index = 0;

    vector<Vector2i> robotRelativePosition = {Vector2i(0, 1), Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0)};
};