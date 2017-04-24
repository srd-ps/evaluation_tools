#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

float gt_x;
float gt_y;
float gt_x_old;
float gt_y_old;
float standstill_x;
float standstill_y;
uint standstill_counter(0);
bool init(false);
bool robot_standstill(false);
uint min_standstill_count(25);

void compare(const nav_msgs::Odometry::ConstPtr& msg)
{
  gt_x_old = gt_x;
  gt_y_old = gt_y;
  float div_x = msg->pose.pose.position.x;
  float div_y = msg->pose.pose.position.y;
  ROS_INFO_STREAM("Div x: " << div_x << " Div y: " << div_y);
  gt_x = div_x;
  gt_y = div_y;

  if (fabs(gt_x_old - gt_x) < 0.1 && fabs(gt_y_old - gt_y) < 0.1)
  {
    if (!init)
    {
     standstill_counter = 0;
     standstill_x = gt_x;
     standstill_y = gt_y;
     init = true;
    }
    else
    {
      if ((fabs(standstill_x - gt_x) < 0.1 && fabs(standstill_y - gt_y) < 0.1))
        standstill_counter++;
      else
        init = false;
    }
  }
  else
    init = false;

  if (min_standstill_count < standstill_counter)
    robot_standstill = true;
  ROS_INFO_STREAM("Standstill counter:" <<standstill_counter);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "loc_evaluation");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub = n.subscribe("base_pose_ground_truth", 1, compare);

  ros::Duration t(1);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string map("/map");
  std::string name;

  std::string path;

  int count_max;

  if (!pn.getParam("tf_prefix", name))
  {
    name = "";
    std::cout << "it failed" << std::endl;
  }

  if (!pn.getParam("path", path))
    path = "measure.txt";

  if (!pn.getParam("count_max", count_max))
    count_max = 30;

  std::ofstream log;
  log.open(path.c_str(), std::ofstream::out | std::ofstream::app);

  int count = 0;
  while (ros::ok())
  {

    t.sleep();

    ros::spinOnce();

    try
    {
      listener.lookupTransform(map, name+"/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    float trans_x = transform.getOrigin().getX();
    float trans_y = transform.getOrigin().getY();
    ROS_INFO_STREAM("trans_x: " << trans_x << " trans_y: " << trans_y);

    float r = std::sqrt((trans_x - gt_x)*(trans_x - gt_x)+(trans_y - gt_y)*(trans_y - gt_y));
    float err_y = fabs(trans_y-gt_y);
    ROS_INFO_STREAM("R: " << r );
    ROS_INFO_STREAM("Err_y:"<<err_y);

    //log << err_y << std::endl;
    log << r << std::endl;
    ++count;
    ROS_INFO_STREAM(count);

    if (count_max == count)
    {
      ROS_INFO_STREAM("Ended Recording due to max measurement count");
      ROS_INFO_STREAM(path);
      log.close();
      break;
    }
    else if (robot_standstill)
    {
      ROS_INFO_STREAM("Ended Recording due to robot stand still");
      ROS_INFO_STREAM(path);
      log.close();
      break;
    }
  }

  return 0;
}
