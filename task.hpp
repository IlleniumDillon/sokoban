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
    Task(Vector2i _position, string _name) 
    {
        position = _position;
        name = _name;
    }
    std::string name;
    Vector2i position;
};