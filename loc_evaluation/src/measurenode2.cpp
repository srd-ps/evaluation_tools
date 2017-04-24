#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream> 
#include <math.h>
#include <list>
#include <string>
#include <tf/transform_datatypes.h>

using namespace std;

class evaluation_test{
public:       
        
    list<double> errorlist_trans_station1;
    list<double> errorlist_trans_station2;
    list<double> errorlist_rot_station1;
    list<double> errorlist_rot_station2;

    double vecNorm(double x, double y){
            return sqrt(x*x + y*y);
    }

    double rad_to_deg (double rad){
        return rad * (180/M_PI);
    }

    void get_translation_station1(){
        tf::StampedTransform transform;
        tf::TransformListener listener;
        listener.waitForTransform("/base_link", "/base_link_optitrack_station1", ros::Time(0), ros::Duration(3.0));
        
        try{
            listener.lookupTransform("/base_link", "/base_link_optitrack_station1", ros::Time(0), transform);
        }

        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        double trans = this->vecNorm(transform.getOrigin().x(), transform.getOrigin().y());
        double rot = tf::getYaw(transform.getRotation());
        double rot_deg = rad_to_deg(rot);

        if(trans < 0.25){
            errorlist_rot_station1.push_back(rot_deg);
            errorlist_trans_station1.push_back(trans);
            cout << "Current rotational error in degree on Station 1: " << rot_deg << endl;
            cout << "Current translational error on Station 1: " << trans << endl;

        }

    }    
    void get_translation_station2(){
        tf::StampedTransform transform;
        tf::TransformListener listener;

        listener.waitForTransform("/base_link", "/base_link_optitrack_station2", ros::Time(0), ros::Duration(3.0));
        try{
            listener.lookupTransform("/base_link", "/base_link_optitrack_station2", ros::Time(0), transform);
        }

        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        double trans = this->vecNorm(transform.getOrigin().x(), transform.getOrigin().y());
        double rot = tf::getYaw(transform.getRotation());
        double rot_deg = rad_to_deg(rot);

        if(trans < 0.25 and set_validity_station2()){
            errorlist_rot_station2.push_back(rot_deg);
            errorlist_trans_station2.push_back(trans);
            cout << "Current rotational error in degree on Station 2: " << rot_deg << endl;
            cout << "Current translational error on Station 2: " << trans << endl;
            
        }

        
    }

    bool set_validity_station2(){
        tf::StampedTransform transform;
        tf::StampedTransform init_transform;
        tf::TransformListener listener;

        listener.waitForTransform("/map", "/base_link_optitrack_station2", ros::Time(0), ros::Duration(3.0));
        try{
            listener.lookupTransform("/map", "/base_link_optitrack_station2", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::Duration(0.1).sleep();
        try{
            listener.lookupTransform("/map", "/base_link_optitrack_station2", ros::Time(0), init_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        if(transform.getOrigin().x() != init_transform.getOrigin().x()){
            return true;
        }
        else{
            return false;
        }


    }

};

int main(int argc, char** argv){
  evaluation_test test;
  ros::init(argc, argv, "ipa_navigation_eva_script");
  ros::NodeHandle node;

  ofstream log_data_trans("trans.txt", ios::app);  
  ofstream log_data_rot("rot.txt", ios::app);

  for(int i = 0; i<200; i++){
    test.get_translation_station1();
    test.get_translation_station2();    
  }
  while(test.errorlist_trans_station1.empty() == false){
    log_data_trans << test.errorlist_trans_station1.front() << endl;
    test.errorlist_trans_station1.pop_front();
    log_data_rot << test.errorlist_rot_station1.front() << endl;
    test.errorlist_rot_station1.pop_front();
  }
  
  while(test.errorlist_trans_station2.empty() == false){
    log_data_trans << test.errorlist_trans_station2.front() << endl;
    test.errorlist_trans_station2.pop_front();
    log_data_rot << test.errorlist_rot_station2.front() << endl;
    test.errorlist_rot_station2.pop_front();
  }

  log_data_trans.close();
  log_data_rot.close();

  return 0;
};