#pragma once

#include <iostream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "box.hpp"
#include "robot.hpp"

using namespace Eigen;
using namespace cv;
using namespace std;

class World
{
public:
    World(int _width, int _height) : width(_width), height(_height)
    {
        map_blank = Mat(height, width, CV_8UC3, Scalar(255, 255, 255));
        map_move = Mat(height, width, CV_8UC1, Scalar(0));
        map_push = Mat(height, width, CV_8UC1, Scalar(0));
    }

    void addBox(Box box)
    {
        boxes.push_back(box);
    }

    void addRobot(Robot robot)
    {
        robots.push_back(robot);
    }

    void draw()
    {
        Mat map_show = map_blank.clone();

        for(auto& box : boxes)
        {
            if(box.static_)
                map_show.at<Vec3b>(box.position.y(), box.position.x()) = Vec3b(0, 0, 0);
            else
                map_show.at<Vec3b>(box.position.y(), box.position.x()) = Vec3b(0, 255, 0);
        }

        for(auto& robot : robots)
        {
            map_show.at<Vec3b>(robot.position.y(), robot.position.x()) = Vec3b(0, 0, 255);
        }

        resize(map_show, map_show, Size(800, 600), 0, 0, INTER_NEAREST);

        for (auto& robot : robots)
        {
            int text_x = robot.position.x() * 800 / width;
            int text_y = robot.position.y() * 600 / height + 10;
            putText(map_show, robot.name, Point(text_x, text_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);

            for (auto &task : robot.taskList)
            {
                int task_x = task.position.x() * 800 / width;
                int task_y = task.position.y() * 600 / height + 10;
                putText(map_show, task.name, Point(task_x, task_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

                int task_lu_x = task.position.x() * 800 / width;
                int task_lu_y = task.position.y() * 600 / height;
                int task_rd_x = (task.position.x() + 1) * 800 / width;
                int task_rd_y = (task.position.y() + 1) * 600 / height;
                rectangle(map_show, Point(task_lu_x, task_lu_y), Point(task_rd_x, task_rd_y), Scalar(0, 255, 255), 5);
            }

            for(auto& path : robot.path)
            {
                int path_x = (path.x() + 0.5) * 800 / width;
                int path_y = (path.y() + 0.5) * 600 / height;
                circle(map_show, Point(path_x, path_y), 5, Scalar(0, 0, 255), -1);
            }
        }

        for (auto& box : boxes)
        {
            if(box.static_)
                continue;
            int text_x = box.position.x() * 800 / width;
            int text_y = box.position.y() * 600 / height + 10;
            putText(map_show, box.name, Point(text_x, text_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
        }

        imshow("Map", map_show);
    }

    void update()
    {
        for(auto& box : boxes)
        {
            map_move.at<uchar>(box.position.y(), box.position.x()) = 1;
            map_push.at<uchar>(box.position.y(), box.position.x()) = 1;
        }
        for(auto& robot : robots)
        {
            map_move.at<uchar>(robot.position.y(), robot.position.x()) = 1;
            map_push.at<uchar>(robot.position.y(), robot.position.x()) = 1;
        }
        for(auto& robot : robots)
        {
            map_move.at<uchar>(robot.position.y(), robot.position.x()) = 0;
            map_push.at<uchar>(robot.position.y(), robot.position.x()) = 0;
            map_push.at<uchar>(robot.taskList.front().position.x(), robot.taskList.front().position.y()) = 0;
            robot.update(map_move, map_push, boxes);
        }

        //todo: box position update
    }

private:
    int width, height;
    Mat map_blank;
    Mat map_move;
    Mat map_push;
    vector<Box> boxes;
    vector<Robot> robots;
};