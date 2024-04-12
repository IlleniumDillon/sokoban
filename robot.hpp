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
        PUSH,
        PULL,
        NOACTION
    } Action;
    Robot(Vector2i _position, string _name) : position(_position), name(_name) {}
    void getTask(Task task_)
    {
        taskList.push_back(task_);
        taskList = sokoban.fixTaskList(taskList);
    }
    void move(Vector2i direction)
    {
        position += direction;
        lastMove = direction;
    }
    void setAction(Action action_)
    {
        action = action_;
    }
    
    Vector2i position;
    Vector2i lastMove;
    std::string name;
    Sokoban sokoban;
    std::vector<Task> taskList;
    Vector2i goal;
    std::vector<Vector2i> path;
    Action action = NOACTION;
};