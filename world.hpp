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
    void addTask(Task task)
    {
        taskList.push_back(task);
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

        resize(map_show, map_show, Size(show_width, show_height), 0, 0, INTER_NEAREST);

        for (auto& robot : robots)
        {
            int text_x = robot.position.x() * show_width / width;
            int text_y = robot.position.y() * show_height / height + 10;
            putText(map_show, robot.name, Point(text_x, text_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);

            int mode_x = robot.position.x() * show_width / width;
            int mode_y = robot.position.y() * show_height / height + 20;
            string mode_str = robot.action == Robot::Action::PUSH ? "push" : robot.action == Robot::Action::PULL ? "pull" : "noaction";
            putText(map_show, mode_str, Point(mode_x, mode_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);

            int steps_x = robot.position.x() * show_width / width;
            int steps_y = robot.position.y() * show_height / height + 30;
            putText(map_show, to_string(robot.curSteps), Point(steps_x, steps_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);

            for (auto &task : taskList)
            {
                int task_x = task.position.x() * show_width / width;
                int task_y = task.position.y() * show_height / height + 10;
                putText(map_show, task.boxName, Point(task_x, task_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

                int task_lu_x = task.position.x() * show_width / width;
                int task_lu_y = task.position.y() * show_height / height;
                int task_rd_x = (task.position.x() + 1) * show_width / width;
                int task_rd_y = (task.position.y() + 1) * show_height / height;
                rectangle(map_show, Point(task_lu_x, task_lu_y), Point(task_rd_x, task_rd_y), Scalar(0, 255, 255), 5);
            }
        }

        for (auto& box : boxes)
        {
            if(box.static_)
                continue;
            int text_x = box.position.x() * show_width / width;
            int text_y = box.position.y() * show_height / height + 10;
            putText(map_show, box.name, Point(text_x, text_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
        }

        imshow("Map", map_show);
    }

    void update()
    {
        //box and robot position update
        for(auto& robot : robots)
        {
            auto newRobotPose = robot.position + robot.lastMove;
            if(newRobotPose.x() < 0 || newRobotPose.x() >= width || newRobotPose.y() < 0 || newRobotPose.y() >= height)
            {
                cout << "Robot " << robot.name << " move out of map!" << endl;
                break;
            }
            auto box = find_if(boxes.begin(), boxes.end(), [&](Box box){return box.position == newRobotPose;});
            if(box != boxes.end())
            {
                if(box->static_)
                {
                    cout << "Robot " << robot.name << " hit a static box!" << endl;
                    break;
                }
                else
                {
                    if(robot.action == Robot::Action::NOACTION)
                    {
                        cout << "Robot " << robot.name << " hit a box without action!" << endl;
                        break;
                    }
                }
            }
            
            if(robot.action == Robot::Action::PUSH)
            {
                if(robot.lastMove == Vector2i(0, 0))
                {
                    cout << "Robot " << robot.name << " start push" << endl;
                    break;
                }
                else if(box != boxes.end())
                {
                    auto boxPos = box->position + robot.lastMove;
                    if(boxPos.x() < 0 || boxPos.x() >= width || boxPos.y() < 0 || boxPos.y() >= height)
                    {
                        cout << "Robot " << robot.name << " push a box out of map!" << endl;
                        break;
                    }
                    else if(find_if(boxes.begin(), boxes.end(), [&](Box box){return box.position == boxPos;}) != boxes.end())
                    {
                        cout << "Robot " << robot.name << " push a box to another box!" << endl;
                        break;
                    }
                    else
                    {
                        box->position += robot.lastMove;
                        robot.position += robot.lastMove;
                    }
                }
                else
                {
                    cout << "Robot " << robot.name << " push without box!" << endl;
                    break;
                }
            }
            else if(robot.action == Robot::Action::PULL)
            {
                auto pullPos = robot.position - robot.lastMove;
                auto pullBox = find_if(boxes.begin(), boxes.end(), [&](Box box){return box.position == pullPos;});
                if(robot.lastMove == Vector2i(0, 0))
                {
                    cout << "Robot " << robot.name << " start pull" << endl;
                    break;
                }
                else if(pullBox != boxes.end())
                {
                    if(pullBox->static_)
                    {
                        cout << "Robot " << robot.name << " pull a static box!" << endl;
                        break;
                    }
                    else
                    {
                        pullBox->position += robot.lastMove;
                        robot.position += robot.lastMove;
                    }
                }
                else
                {
                    cout << "Robot " << robot.name << " pull without box!" << endl;
                    break;
                }
            }
            else
            {
                robot.position += robot.lastMove;
            }
        }

    }


    int width, height;
    int show_width = 800, show_height = 600;
    Mat map_blank;
    Mat map_move;
    Mat map_push;
    vector<Box> boxes;
    vector<Robot> robots;
    std::vector<Task> taskList;
};