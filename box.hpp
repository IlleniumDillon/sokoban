#pragma once

#include <iostream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cv;
using namespace std;

class Box
{
public:
    Box(Vector2i _position, string _name, bool _static) : position(_position), name(_name), static_(_static) {}
    Vector2i position;
    std::string name;
    bool static_;
};