#include "ipa_map_comparison/ipa_map_comparison_node.h"

ipa_map_comparison_node::ipa_map_comparison_node()
{
  ros::NodeHandle nh;
  ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv;

  while (!map_client.call(srv))
  {
    ros::Duration call_delay(0.5);
    call_delay.sleep();
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout ground truth map. Trying again!");
  }
  ground_truth_map_ = srv.response.map;
  map_client = nh.serviceClient<nav_msgs::GetMap>("/map");
  while (!map_client.call(srv))
  {
    ros::Duration call_delay(0.5);
    call_delay.sleep();
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node", "[ipa_map_comparison_node]: Unable to load layout map. Trying again!");
  }
  map_ = srv.response.map;

  //check wheather the two maps have the same size
//  if (map_.info.height * map_.info.resolution != ground_truth_map_.info.height * ground_truth_map_.info.resolution
//      || map_.info.width * map_.info.resolution != ground_truth_map_.info.width * ground_truth_map_.info.resolution)
//  {
//    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node","[ipa_map_comparison_node]: Sizes of maps differ: Ground Truth Width:"
//                           <<ground_truth_map_.info.width * ground_truth_map_.info.resolution<<"Map Width: "
//                           <<map_.info.width * map_.info.resolution<<"Ground Truth Height: "
//                           <<ground_truth_map_.info.width * ground_truth_map_.info.resolution<<"Map Height: "
//                           <<map_.info.height * map_.info.resolution);
//    return;
//  }
  double factor = map_.info.resolution / ground_truth_map_.info.resolution;
  if (factor == std::floor(factor) && factor >= 1.0)
    compareMaps();
  else
  {
    ROS_ERROR_STREAM_NAMED("ipa_map_comparison_node","[ipa_map_comparison_node:]"
                                                     "Factor of both maps is not an integer value: "<<factor);
    return;
  }
}
void ipa_map_comparison_node::compareMaps()
{
  int factor = map_.info.resolution / ground_truth_map_.info.resolution;
  int width_offset = 0, height_offset = 0, width_left_offset = 0, width_right_offset = 0, height_top_offset = 0,
      height_bottom_offset = 0, occ_count = 0, free_count = 0;
  //check the size of the maps, if they fit into each other without overlapping if overlapping correct size
  //necessary for the sliding window comparison later on
  if (factor * map_.info.width > ground_truth_map_.info.width)
  {
    width_offset = factor * map_.info.width - ground_truth_map_.info.width;
    if (width_offset / 2.0 == std::floor((width_offset / 2.0 )))
        width_left_offset = width_right_offset = width_offset/2;
    else
    {
      width_left_offset = std::ceil(width_offset / 2.0);
      width_right_offset = std::floor(width_offset / 2.0);
    }

  }
  if (factor * map_.info.width > ground_truth_map_.info.width)
  {
    height_offset = factor * map_.info.height - ground_truth_map_.info.height;
    if (height_offset / 2.0 == std::floor((width_offset / 2.0 )))
        height_top_offset = height_bottom_offset = height_offset/2;
    else
    {
      height_top_offset = std::ceil(height_offset / 2.0);
      height_bottom_offset = std::floor(height_offset / 2.0);
    }
  }
  double free_score = 0, occ_score = 0;
  std::vector<std::vector<int> > ground_truth_2d_map, map_2d;
  for (int i = 0; i < ground_truth_map_.info.height + height_offset; i++)
  {
    std::vector<int> temp;
    if (i < height_top_offset || i >= ground_truth_map_.info.height)
      temp = std::vector<int>(ground_truth_map_.info.width + width_offset, -2);
    else
    {
      for (int j = 0; j < ground_truth_map_.info.width + width_offset; j++)
      {
        if (j < width_left_offset || j >= ground_truth_map_.info.width)
          temp.push_back(-2);
        else
        {
          temp.push_back(ground_truth_map_.data[i*ground_truth_map_.info.width + j]);
          if (ground_truth_map_.data[i*ground_truth_map_.info.width + j] == 100)
            occ_count++;
          else if (ground_truth_map_.data[i*ground_truth_map_.info.width + j] == 0)
            free_count++;
        }
      }
    }
    ground_truth_2d_map.push_back(temp);
  }

  for (int i = 0; i < map_.info.height; i++)
  {
    std::vector<int> temp;
    for (int j = 0; j < map_.info.width; j++)
    {
      temp.push_back(map_.data[i*ground_truth_map_.info.width + j]);
    }
    map_2d.push_back(temp);
  }

  for (int i = 0; i < map_2d.size(); i++)
    for (int j = 0; j < map_2d[j].size(); j++)
    {
      int map_occ_value = map_2d[i][j];
      for (int inner_i = 0; inner_i < factor; inner_i++)
        for (int inner_j = 0; inner_j < factor; inner_j++)
        {
          int ground_truth_value = ground_truth_2d_map[i* factor + inner_i][j* factor + inner_j];
          if (map_occ_value == ground_truth_value)
          {
            if (map_occ_value == 100)
              occ_score ++;
            else if(map_occ_value == 0)
              free_score ++;
          }

        }

    }
  free_score /= free_count;
  occ_score /= occ_count;
  ROS_ERROR_STREAM("occ_score: "<<occ_score<< " free_score: "<<free_score);

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ipa_map_comparison_node");
  ros::NodeHandle n;

  // wait till first clock message is received
  while (ros::Time::now().sec == 0)
  {
  }
  ipa_map_comparison_node comp_node = ipa_map_comparison_node();
  ros::spinOnce();
  return 0;
}
