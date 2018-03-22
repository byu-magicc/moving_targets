#pragma once

#include <memory>
#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/math/Matrix3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/rendering/rendering.hh>
// #include "gazebo/rendering/Visual.hh"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "path_manager/radius_manager.h"
#include "path_follower/straight_line.h"
#include "path_follower/orbit.h"
#include "path_follower/base_follower.h"
#include "waypoint.h"


#include <Eigen/Dense>

namespace gazebo
{
  class TargetMotion : public ModelPlugin 
  {
  public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

    void loadTrajectory();

    void OnUpdate(const common::UpdateInfo& _info);


    void VelController(double chi_er, double h_er, double yaw, double distance, double dt);

    double getValueFromSdf(std::string name);

    void drawWaypoins(const motion::waypoints_t waypoints);

    // Publish movers state to ROS
    void PublishState();





  private:
    // Pointer to the model
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_pointer_;
    // rendering::VisualPtr visual_;
    event::ConnectionPtr updateConnection_;

    double simTime_d1_ = 0;

    ros::Publisher state_;

    // Velocity
    float v_;

    // waypoint manager
    motion::RadiusManager radius_manager_;

    // Pointer to follower class that gets 
    // heading and altitude commands
    std::shared_ptr<motion::BaseFollower> follower_;

    // Agent's waypoints
    motion::waypoints_t waypoints_;

    // Parameters
    motion::FollowerParams params_;

    // Used to draw waypoints
    // rendering::DynamicLines *dynamic_lines_;

    // ROS stuff
    ros::NodeHandle nh_;

    common::PID headingPID_;
    common::PID altitudePID_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMotion)
}