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
        std::vector<string> ignore;
        for (auto& box : world->taskList)
        {
            ignore.push_back(box.boxName);
        }
        for (auto& robot : world->robots)
        {
            ignore.push_back(robot.name);
        }
        // std::vector<string> ignore{"robot0","box36"};
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
        BoxAStar boxAStar;
        int cost = boxAStar.graphSearch(box_start,box_end, robot_start, map);
        std::cout << "Cost: " << cost << std::endl;
        world->robots[0].path = boxAStar.getPathList();
    }
    void getNeighbors(WorldNodePtr current, vector<WorldNodePtr>& neighbors, vector<int>& edgeCost, vector<vector<robotPathPoint>>& minipath, vector<Vector2i>& robotStateList, vector<WorldNodePtr>& closeList ,multimap<int, WorldNodePtr>& openList)
    {
        neighbors.clear();
        edgeCost.clear();
        minipath.clear();
        robotStateList.clear();

        for (int i = 0; i < boxGoals.size(); i++)
        {
            if(current->position[i] == boxGoals[i])
            {
                continue;
            }

            BoxAStar boxAStar;
            Mat tempMap = map.clone();
            for (int j = 0; j < current->position.size(); j++)
            {
                tempMap.at<uchar>(current->position[j].y(), current->position[j].x()) = 1;
            }
            tempMap.at<uchar>(current->position[i].y(), current->position[i].x()) = 0;

            vector<Vector2i> goalPositions = current->position;
            goalPositions[i] = boxGoals[i];

            //check if the goal position is in closeList
            auto it_c = find_if(closeList.begin(), closeList.end(), [&](WorldNodePtr node) { return node->position == goalPositions; });
            if(it_c != closeList.end())
            {
                continue;
            }

            //check if the goal position is in openList
            auto it = find_if(openList.begin(), openList.end(), [&](pair<int, WorldNodePtr> node) { return node.second->position == goalPositions; });
            if(it != openList.end())
            {
                int cost = boxAStar.graphSearch(current->position[i], boxGoals[i], current->robotPosition, tempMap);
                if(cost == -1)
                {
                    continue;
                }
                auto tempPath = boxAStar.getPathList();
                auto lastMove = tempPath.back().move;
                auto lastRobotPosition = boxGoals[i] - lastMove;

                neighbors.push_back(it->second);
                edgeCost.push_back(cost);
                minipath.push_back(tempPath);
                robotStateList.push_back(lastRobotPosition);
                continue;
            }

            //new node
            int cost = boxAStar.graphSearch(current->position[i], boxGoals[i], current->robotPosition, tempMap);
            if(cost == -1)
            {
                continue;
            }
            auto tempPath = boxAStar.getPathList();
            auto lastMove = tempPath.back().move;
            auto lastRobotPosition = boxGoals[i] - lastMove;

            WorldNodePtr newNode = make_shared<WorldNode>();
            newNode->position = goalPositions;
            newNode->robotPosition = lastRobotPosition;
            newNode->g = std::numeric_limits<int>::max();
            newNode->h = 0;
            newNode->f = std::numeric_limits<int>::max();
            newNode->id = 0;
            newNode->parent = nullptr;
            newNode->path.clear();

            neighbors.push_back(newNode);
            edgeCost.push_back(cost);
            minipath.push_back(tempPath);
            robotStateList.push_back(lastRobotPosition);
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
        boxStarts.clear();
        boxGoals.clear();

        for (auto& task : startAndGoal)
        {
            auto box = find_if(world->boxes.begin(), world->boxes.end(), [&](Box& box) { return box.name == task.boxName; });
            boxStarts.push_back(box->position);
            boxGoals.push_back(task.position);
        }

        multimap<int, WorldNodePtr> openList;

        WorldNodePtr startNode = make_shared<WorldNode>();
        startNode->position = boxStarts;
        startNode->robotPosition = robot.position;
        startNode->g = 0;
        startNode->h = getHeuristic(startNode, boxGoals);
        startNode->f = startNode->g + startNode->h;
        startNode->id = 1;
        startNode->parent = nullptr;
        startNode->it = openList.insert(make_pair(startNode->f, startNode));

        vector<WorldNodePtr> closeList;

        int tentative_gScore;
        int numIter = 0;
        WorldNodePtr currentPtr = nullptr;
        WorldNodePtr neighborPtr = nullptr;

        vector<WorldNodePtr> neighbors;
        vector<int> edgeCost;
        vector<vector<robotPathPoint>> minipath;
        vector<Vector2i> robotStateList;

        while (!openList.empty())
        {
            numIter++;
            currentPtr = openList.begin()->second;
            if (currentPtr->position == boxGoals)
            {
                int cost = currentPtr->g;
                vector<WorldNodePtr> worldPath;
                while (currentPtr != nullptr)
                {
                    worldPath.push_back(currentPtr);
                    currentPtr = currentPtr->parent;
                }
                reverse(worldPath.begin(), worldPath.end());
                for (int i = 1; i < worldPath.size(); i++)
                {
                    for (int j = 0; j < worldPath[i]->path.size(); j++)
                    {
                        pathList.push_back(worldPath[i]->path[j]);
                    }
                }
                robot.path = pathList;
                return cost;
            }
            openList.erase(openList.begin());
            closeList.push_back(currentPtr);
            currentPtr->id = -1;

            getNeighbors(currentPtr, neighbors, edgeCost, minipath, robotStateList, closeList, openList);

            for(int i = 0; i < neighbors.size(); i++)
            {
                neighborPtr = neighbors[i];
                if (neighborPtr->id == -1)
                {
                    continue;
                }
                tentative_gScore = currentPtr->g + edgeCost[i];
                if(neighborPtr->id == 0)
                {
                    neighborPtr->id = 1;
                    neighborPtr->g = tentative_gScore;
                    //neighborPtr->h = 0;
                    neighborPtr->h = getHeuristic(neighborPtr, boxGoals);
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->path = minipath[i];
                    neighborPtr->robotPosition = robotStateList[i];
                    neighborPtr->it = openList.insert(std::pair<int, WorldNodePtr>(neighborPtr->f, neighborPtr));
                }
                else if(tentative_gScore < neighborPtr->g)
                {
                    neighborPtr->g = tentative_gScore;
                    neighborPtr->f = neighborPtr->g + neighborPtr->h;
                    neighborPtr->parent = currentPtr;
                    neighborPtr->path = minipath[i];
                    neighborPtr->robotPosition = robotStateList[i];
                    openList.erase(neighborPtr->it);
                    neighborPtr->it = openList.insert(std::pair<int, WorldNodePtr>(neighborPtr->f, neighborPtr));
                }
            }
        }
        
        return -1;
    }

    World* world;
    //RobotAStar robotAStar;
    //BoxAStar boxAStar;
    //map: 0 free, 1 occupied
    //ignore moving box and robot
    Mat map;
    vector<robotPathPoint> pathList;
    int index = 0;
    vector<Vector2i> boxStarts;
    vector<Vector2i> boxGoals;
    vector<Vector2i> robotRelativePosition = {Vector2i(0, 1), Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0)};
};