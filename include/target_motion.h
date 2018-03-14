#include <memory>
#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "unicycle_planner.h"

#include <Eigen/Dense>

namespace gazebo
{
  class TargetMotion : public ModelPlugin
  {
  public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);


    void OnUpdate(const common::UpdateInfo& _info);


    void VelController(double vel, double omega);

    // Publish movers state to ROS
    void PublishState();





  private:
    // Pointer to the model
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection_

    ros::Publisher state_;

    // ROS stuff
    ros::NodeHandle nh_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMotion)
}