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
  ros::NodeHandle nh;
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
  if (factor * map_.info.height > ground_truth_map_.info.height)
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
  ROS_ERROR_STREAM("w_offset: "<<width_offset<<" h_offset: "<<height_offset );
  double free_score = 0, occ_score = 0, false_free = 0, false_occ = 0;
  std::vector<std::vector<int> > ground_truth_2d_map, map_2d;
  ground_truth_2d_map.resize(ground_truth_map_.info.height + height_offset);
  for (int j = 0; j < ground_truth_map_.info.height + height_offset; j++)
    ground_truth_2d_map[j].resize(ground_truth_map_.info.width + width_offset);
  for (int i = 0; i < ground_truth_map_.info.height + height_offset; i++)
  {
    for (int j = 0; j < ground_truth_map_.info.width + width_offset; j++)
    {
      if /*(i < height_offset)*/(i >= ground_truth_map_.info.height)
      {
        ground_truth_2d_map[i][j] = -1;
        //temp = std::vector<int>(ground_truth_map_.info.width + width_offset, -1);
      }
      else if (j >= ground_truth_map_.info.width)
        ground_truth_2d_map[i][j] = -1;
      else
      {
        ground_truth_2d_map[i][j] = (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j]);
        if (ground_truth_2d_map[i][j] < -1 || ground_truth_2d_map[i][j] > 100)
          ground_truth_2d_map[i][j]=-1;
          //ROS_ERROR_STREAM("ground_truth_2d_map[i][j]: "<<static_cast<int>((ground_truth_map_.data[(i)*ground_truth_map_.info.width + j]))<<" i "<<i<<" j "<<j);
        if (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] >= 50)
          occ_count++;
        else if (ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] >= 0
                 && ground_truth_map_.data[(i)*ground_truth_map_.info.width + j] < 50)
          free_count++;
      }
    }
  }
  for (int i = 0; i < ground_truth_map_.info.height + height_offset; i++)
  {
    for (int j = 0; j < ground_truth_map_.info.width + width_offset; j++)
    {
//      if (ground_truth_2d_map[i][j] < -1 || ground_truth_2d_map[i][j] > 100)
//        ROS_ERROR_STREAM("ground_truth_2d_map[i][j]: "<<ground_truth_2d_map[i][j]<<" i "<<i<<" j "<<j);
    }
  }
  ROS_ERROR_STREAM(map_.info.height);
  map_2d.resize(ground_truth_map_.info.height + height_offset);
  for (int j = 0; j < ground_truth_map_.info.height + height_offset; j++)
    map_2d[j].resize(ground_truth_map_.info.width + width_offset);
  ROS_ERROR_STREAM("height: "<<map_2d.size()<< "width:"<<map_2d[0].size());
  for (int i = 0; i < (map_.info.height); i++)
  {
    for (int j = 0; j < (map_.info.width); j++)
    {
      for (int inner_i = 0; inner_i < factor; inner_i++)
        for (int inner_j = 0; inner_j < factor; inner_j++)
      {
            map_2d[inner_i + i*factor][inner_j + j*factor]=map_.data[i*map_.info.width + j];
            //ROS_ERROR_STREAM("x: "<<i + inner_i<< " y: "<<j+ inner_j);
      }
    }
  }
  ROS_ERROR_STREAM("2d map  finished");

  map_2d_msg_.info.height = map_2d.size();
  map_2d_msg_.info.width = map_2d[0].size();
  map_2d_msg_.info.resolution = ground_truth_map_.info.resolution;
  map_2d_msg_.info.origin = ground_truth_map_.info.origin;
  std::vector<int8_t> map_data((ground_truth_map_.info.width + width_offset) * (ground_truth_map_.info.height + height_offset), -1);
  for (uint y = 0; y < (ground_truth_map_.info.height + height_offset); y++)
    for (uint x = 0; x < (ground_truth_map_.info.width + width_offset); x++)
    {
        map_data.at(y * (ground_truth_map_.info.width + width_offset) + x) = map_2d[y][x];
    }

  map_2d_msg_.data = map_data;
  pub = nh.advertise<nav_msgs::OccupancyGrid>("map_measured", 1);
  pub.publish(map_2d_msg_);
  ref_2d_msg_.info.height = ground_truth_map_.info.height + height_offset;
  ref_2d_msg_.info.width = ground_truth_map_.info.width + width_offset;
  ref_2d_msg_.info.resolution = ground_truth_map_.info.resolution;
  ref_2d_msg_.info.origin = ground_truth_map_.info.origin;
  std::vector<int8_t> map_data2((ground_truth_map_.info.width + width_offset) * (ground_truth_map_.info.height + height_offset), -1);
  for (uint y = 0; y < (ground_truth_map_.info.height + height_offset); y++)
    for (uint x = 0; x < (ground_truth_map_.info.width + width_offset); x++)
    {
        map_data2.at(y * (ground_truth_map_.info.width + width_offset) + x) = ground_truth_2d_map[y][x];
    }

  ref_2d_msg_.data = map_data2;
  pub2 = nh.advertise<nav_msgs::OccupancyGrid>("map_ref", 1);
  pub2.publish(ref_2d_msg_);

  //ROS_ERROR_STREAM("height: "<<height_offset<<" width: "<<width_offset);
  for (int i = 0; i < (ground_truth_map_.info.height + height_offset); i++)
    for (int j = 0; j < (ground_truth_map_.info.width + width_offset); j++)
    {
      int map_occ_value = map_2d[i][j];
      int ground_truth_value = ground_truth_2d_map[i][j];
      if (map_occ_value == ground_truth_value)
      {
        if (map_occ_value >= 50)
          occ_score ++;
        else if(map_occ_value >= 0 && map_occ_value < 50)
          free_score ++;
      }
      else
      {
        if (map_occ_value >= 50)
          false_occ++;
        else if (map_occ_value >= 0 && map_occ_value < 50)
          false_free++;
      }

    }
  free_score /= free_count;
  occ_score /= occ_count;
  false_occ /= ground_truth_map_.info.height* ground_truth_map_.info.width;
  false_free /= ground_truth_map_.info.height* ground_truth_map_.info.width;
  ROS_ERROR_STREAM("occ_score: "<<occ_score<< " free_score: "<<free_score<< " false_occ: "<<false_occ<<" false_free: "<<false_free);

}
void ipa_map_comparison_node::publish()
{
  pub.publish(map_2d_msg_);
  pub2.publish(ref_2d_msg_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ipa_map_comparison_node");
  ros::NodeHandle n;

  // wait till first clock message is received
  while (ros::Time::now().sec == 0)
  {
  }
  ros::Duration(3).sleep();
  ipa_map_comparison_node comp_node = ipa_map_comparison_node();
  while (ros::ok())
  {
    comp_node.publish();
    ros::spinOnce();
  }
  return 0;
}
