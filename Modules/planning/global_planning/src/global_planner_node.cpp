#include <ros/ros.h>

#include "global_planner.h"

using namespace Global_Planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  ros::NodeHandle nh("~");

  Global_Planner global_planner;//创建一个Global_Planner对象
  global_planner.init(nh);

  ros::spin();//等待回调函数触发

  return 0;//执行不到
}

