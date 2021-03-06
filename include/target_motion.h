#pragma once

#include <memory>
#include <stdio.h>
#include <algorithm>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/rendering/rendering.hh>
// #include "gazebo/rendering/Visual.hh"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include "waypoint.h"

#include "gz_compat.h"

#include "path_manager/base_manager.h"
#include "path_manager/radius_manager.h"
#include "path_manager/half_plane_manager.h"

#include "path_follower/base_follower.h"
#include "path_follower/straight_line.h"
#include "path_follower/orbit.h"
#include "path_follower/random_waypoints.h"

#include "moving_targets/MovingTargets.h"


namespace gazebo
{
  class TargetMotion : public ModelPlugin 
  {
  public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo& _info);

    void getCommandError(float& chi_er, float& h_er, float& yaw, float& distance);

    void getVelCommands(const common::UpdateInfo& _info, double chi_er, double h_er, double yaw, double distance, double dt, GazeboVector& linear_vel, GazeboVector& angular_vel);

    double getValueFromSdf(std::string name);

    void drawWaypoins(const motion::waypoints_t waypoints);

    bool targetService(moving_targets::MovingTargets::Request &req, moving_targets::MovingTargets::Response &res);

    // Publish movers state to ROS
    void PublishState();

  private:
    // Gazebo stuff
    physics::ModelPtr model_;                // Ptr to model
    sdf::ElementPtr sdf_pointer_;            // Ptr to sdf
    event::ConnectionPtr updateConnection_;  // Ptr to update function

    // Path manager
    std::unique_ptr<motion::BaseManager> manager_;

    // Path follower
    std::shared_ptr<motion::BaseFollower> follower_;
    motion::FollowerParams params_;

    // Agent's waypoints
    motion::waypoints_t waypoints_curr_; // Current waypoints
    motion::waypoints_t waypoints_init_; // Initial waypoints

    // target_motion parameters
    bool move_;                  // Indicates if the agent should move or not
    double simTime_d1_ = 0;      // Time step (s)
    float update_rate_;          //
    float v_;                    // Linear velocity (m/s)
    GazeboPose pose_init_;       // Initial pose of agent.
    float acceleration_;

    // Used to draw waypoints
    // rendering::DynamicLines *dynamic_lines_;

    // ROS stuff
    ros::NodeHandle nh_;
    ros::Publisher state_;
    ros::ServiceServer target_srv_;

    // PID controls
    common::PID headingPID_;
    common::PID altitudePID_;

    // speed scale use to slow down
    float delta_t_;

    void loadTrajectory();
    void loadManager();

    void turnOffCollisions();
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMotion)
}
