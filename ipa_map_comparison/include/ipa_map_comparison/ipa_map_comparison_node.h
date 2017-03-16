#ifndef MAP_COMPARISON_NODE_H
#define MAP_COMPARISON_NODE_H

//ros includes
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <ipa_map_comparison/StartMapEval.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>


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
  bool startMapEval(ipa_map_comparison::StartMapEval::Request& req, ipa_map_comparison::StartMapEval::Response& res);
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::ServiceServer start_map_eval_service_;
  std::string eval_file_name_;
  int number_of_neighbours_;
  float neighbourhood_score_;
  bool map_eval_started_;


};

#endif // MAP_COMPARISON_NODE_H
