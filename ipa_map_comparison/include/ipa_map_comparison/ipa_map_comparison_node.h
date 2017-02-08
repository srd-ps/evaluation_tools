#ifndef MAP_COMPARISON_NODE_H
#define MAP_COMPARISON_NODE_H

//ros includes
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>


class ipa_map_comparison_node
{
public:
  ipa_map_comparison_node();
  nav_msgs::OccupancyGrid map_2d_msg_;
  nav_msgs::OccupancyGrid ref_2d_msg_;
  void publish();
private:
  nav_msgs::OccupancyGrid ground_truth_map_;
  nav_msgs::OccupancyGrid map_;
  void compareMaps();
  ros::Publisher pub;
  ros::Publisher pub2;


};

#endif // MAP_COMPARISON_NODE_H
