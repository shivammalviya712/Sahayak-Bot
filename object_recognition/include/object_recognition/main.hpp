/*
Author: eYRC_SB_363
*/

#ifndef _MAIN_H
#define _MAIN_H

// Standard libraries
#include <iostream>
#include <vector>
#include <string>

// ROS Libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>


// My header files
#include <object_recognition/camera.hpp>
#include <object_recognition/recognition.hpp>

int main(int argc, char **argv);

void detect_callback(std_msgs::Bool msg);

#endif