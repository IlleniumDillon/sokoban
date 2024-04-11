#pragma once

#include <iostream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "sokoban.hpp"
#include "task.hpp"

using namespace Eigen;
using namespace cv;
using namespace std;

class Robot
{
public:
    typedef enum
    {
        IDLE,
        MOVING,
        ACTIVATE,
    } State;
    typedef enum
    {
        PUSH_UP,
        PUSH_DOWN,
        PUSH_LEFT,
        PUSH_RIGHT,
        PULL_UP,
        PULL_DOWN,
        PULL_LEFT,
        PULL_RIGHT,
        NOACTION
    } Action;
    Robot(Vector2i _position, string _name) : position(_position), name(_name) {}
    void getTask(Task task_)
    {
        taskList.push_back(task_);
        taskList = sokoban.fixTaskList(taskList);
    }
    void update(Mat map_move, Mat map_push, std::vector<Box> boxes)
    {
        switch (state)
        {
        case IDLE:
            if (taskList.size() > 0)
            {
                Task task = taskList[0];
                goal = task.position;
                Vector2i box_position = (*std::find_if(boxes.begin(), boxes.end(), [=](Box box) { return box.name == task.name; })).position;
                path = sokoban.findPath(map_move, box_position, goal, position);
                if (path.size() > 0)
                {
                    state = MOVING;
                }
                else
                {
                    taskList.erase(taskList.begin());
                }
            }
            break;
        case MOVING:
            if (path.size() > 0)
            {
                position = path[0];
                path.erase(path.begin());
            }
            else
            {
                state = IDLE;
                taskList.erase(taskList.begin());
            }
            break;
        }
    }
    Vector2i position;
    std::string name;
    Sokoban sokoban;
    std::vector<Task> taskList;
    Vector2i goal;
    std::vector<Vector2i> path;
    State state = IDLE;
    Action action = NOACTION;
};