#include "target_motion.h"

namespace gazebo
{


void TargetMotion::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointers to the model and to the sdf
  model_ = _parent;

  // Turn off gravity
  model_->SetGravityMode(false);


  //
  // Connect to ROS
  //

  nh_ = ros::NodeHandle("targets/" + model_->GetName());

  // Load the trajectory parameters based on the desired type 
  int traj = nh_.param<int>("trajectory_type", 0);
  if (traj == 0)
  { // point

    double x = nh_.param<double>("x", 5);
    double y = nh_.param<double>("y", 5);

    gzmsg << "[TargetMotion] Generated a goToPoint trajectory for " << model_->GetName() << ".\n";
  }
  else if (traj == 1)
  { // waypoints

    // Get origin of waypoints
    double x = nh_.param<double>("x", 0);
    double y = nh_.param<double>("y", 0);

    // Get waypoint lists for x and y
    std::vector<double> waypoints_x, waypoints_y;
    nh_.getParam("waypoints_x", waypoints_x);
    nh_.getParam("waypoints_y", waypoints_y);

    // Construct a waypoint container
    for (int i=0; i<waypoints_x.size(); i++) {
      motion::coord_t waypoint (x + waypoints_x[i], y + waypoints_y[i], 0);
      waypoints_.push_back(waypoint); // translate waypoints to origin

      std::cout << "waypoints: " << waypoints_[i] << std::endl;
    }

    double vx = nh_.param<double>("vx", 1);
    double vy = nh_.param<double>("vy", 1);

    gzmsg << "[TargetMotion] Generated a waypoint trajectory for " << model_->GetName() << ".\n";
  }
  else if (traj == 2)
  { // circle

    double radius = nh_.param<double>("radius", 2);
    double x = nh_.param<double>("x", 5);
    double y = nh_.param<double>("y", 5);

    gzmsg << "[TargetMotion] Generated a circle trajectory for " << model_->GetName() << ".\n";
  }
  else if (traj == 3)
  { // lemniscate (figure-8)

    double a = nh_.param<double>("a", 2);
    double x = nh_.param<double>("x", 5);
    double y = nh_.param<double>("y", 5);

    gzmsg << "[TargetMotion] Generated a lemniscate trajectory for " << model_->GetName() << ".\n";
  }
  else
  { // undefined
    gzerr << "[TargetMotion] Trajectory type " << traj << " is undefined.\n";
  }

  // Create ROS publishers
  state_ = nh_.advertise<nav_msgs::Odometry>("state", 10);


  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetMotion::OnUpdate, this, _1));

}

// ------------------------------------------------------------------------

void TargetMotion::OnUpdate(const common::UpdateInfo& _info)
{
  
  // Get world pose
  math::Pose pose = model_->GetWorldPose();

  // Get position and pack into type motion::coord_t
  math::Vector3 position = pose.pos;
  motion::coord_t pos (position.x, position.y, position.z);

  // Get Euler angles
  math::Vector3 rot = pose.rot.GetAsEuler();

  // Manage waypoints
  motion::coord_t waypoint = radius_manager_.manage_waypoints(pos, waypoints_,1);

  // std::cout << "Current waypoint: " << waypoint;

  // Heading
  float yaw = rot.z;

  // Rotation Matrix
  math::Matrix3 rotz(cos(yaw), -sin(yaw), 0.0,
               sin(yaw),  cos(yaw), 0.0,
               0.0,       0.0,    1.0);

  math::Vector3 vel (1, 0, 0);

  math::Vector3 command = rotz*vel;

  model_->SetLinearVel(command);

  motion::coord_t vect = waypoint - pos;

  std::cout << "vect: " << vect << std::endl;

  double omega = atan2(std::get<1>(vect), std::get<0>(vect));

  std::cout << "omega: " << omega << std::endl;
  std::cout << "yaw: " << yaw << std::endl;

  double mag = motion::get_magnitude(vect);

  double test = 0.1*(omega-yaw);

  // if ( mag < 2)
    model_->SetAngularVel(ignition::math::Vector3d(0,0,test));
  // else
    // model_->SetAngularVel(ignition::math::Vector3d(0,0,0));


    // get velocity and orientation

  PublishState();


}

// ------------------------------------------------------------------------

void TargetMotion::VelController(double vel, double omega)
{
  
  math::Vector3 rot = model_->GetWorldPose().rot.GetAsEuler();


  float yaw = rot.z;

  // Rotation matrix from body to world
  Eigen::Matrix<float, 3,3> rotz;
  rotz << cos(yaw), -sin(yaw), 0.0,
          sin(yaw),  cos(yaw), 0.0,
            0.0,       0.0,    1.0;

  Eigen::Matrix<float,3,1> world_command;
  world_command << vel, 0, 0;
  Eigen::Matrix<float,3,1> interial_frame_command;
  interial_frame_command = rotz*world_command;



  model_->SetLinearVel(ignition::math::Vector3d(interial_frame_command(0),interial_frame_command(1),0));
  model_->SetAngularVel(ignition::math::Vector3d(0,0,omega));


}

void TargetMotion::PublishState() {

    // publish mover's state odometry
    nav_msgs::Odometry odom;
    odom.header.stamp.sec = model_->GetWorld()->GetSimTime().sec;
    odom.header.stamp.nsec = model_->GetWorld()->GetSimTime().nsec;
    odom.header.frame_id = "map";
    odom.child_frame_id = "target_" + model_->GetName();
    odom.pose.pose.position.x     = model_->GetWorldPose().pos.x;
    odom.pose.pose.position.y     = model_->GetWorldPose().pos.y;
    odom.pose.pose.position.z     = model_->GetWorldPose().pos.z;
    odom.pose.pose.orientation.w  = model_->GetWorldPose().rot.w;
    odom.pose.pose.orientation.x  = model_->GetWorldPose().rot.x;
    odom.pose.pose.orientation.y  = model_->GetWorldPose().rot.y;
    odom.pose.pose.orientation.z  = model_->GetWorldPose().rot.z;
    odom.twist.twist.linear.x     = model_->GetRelativeLinearVel().x;
    odom.twist.twist.linear.y     = model_->GetRelativeLinearVel().y;
    odom.twist.twist.linear.z     = model_->GetRelativeLinearVel().z;
    odom.twist.twist.angular.x    = model_->GetRelativeAngularVel().x;
    odom.twist.twist.angular.y    = model_->GetRelativeAngularVel().y;
    odom.twist.twist.angular.z    = model_->GetRelativeAngularVel().z;
    state_.publish(odom);
}

}
