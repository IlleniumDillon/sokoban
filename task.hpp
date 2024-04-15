#pragma once 

#include <iostream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cv;
using namespace std;

class Task
{
public:
    Task(){};
    Task(Vector2i _position, string box_name, string robot_name) 
    {
        position = _position;
        boxName = box_name;
        robotName = robot_name;
    }
    std::string boxName;
    std::string robotName;
    Vector2i position;
};