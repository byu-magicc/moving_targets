#include "target_motion.h"

namespace gazebo {

// void TargetMotion::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
// {
//   visual_ = _parent;
// }

// ----------------------------------------------------------------------------

void TargetMotion::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointers to the model and to the sdf
  model_ = _parent;
  sdf_pointer_ = _sdf;

  // Turn off gravity
  model_->SetGravityMode(false);


  
  // Connect to ROS
  nh_ = ros::NodeHandle("targets/" + model_->GetName());

  // Create ROS publishers
  state_ = nh_.advertise<nav_msgs::Odometry>("state", 10);

  // Create the service
  target_srv_ = nh_.advertiseService("moving_target", &TargetMotion::targetService, this);


  //
  // Get SDF Values
  //
  double kpPsi         = getValueFromSdf("kpPsi");
  double kdPsi         = getValueFromSdf("kdPsi");
  double kpZ           = getValueFromSdf("kpZ");
  double kdZ           = getValueFromSdf("kdZ");
  double maxVPsi       = getValueFromSdf("maxVPsi");
  double maxVZ         = getValueFromSdf("maxVZ");
  params_.k_orbit      = getValueFromSdf("k_orbit");
  params_.k_path       = getValueFromSdf("k_path");
  params_.chi_infinity = getValueFromSdf("chi_infinity");
  update_rate_         = getValueFromSdf("update_rate");


  // Init PID controllers
  headingPID_.Init(kpPsi, 0, kdPsi, 0,0, maxVPsi, -maxVPsi);
  altitudePID_.Init(kpZ, 0, kdZ, 0,0, maxVZ, -maxVZ);

  // Capture the initial pose of the mover
  pose_init_ = GZ_COMPAT_GET_WORLD_POSE(model_);

  // Load trajectory
  loadTrajectory();

  // Load path manager
  loadManager();

  // Do we want to collide with other objects?
  bool collide = nh_.param<bool>("collisions", true);
  if (!collide) turnOffCollisions();

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetMotion::OnUpdate, this, _1));
}

// ------------------------------------------------------------------------

void TargetMotion::loadTrajectory() {

  //
  // Get ROS parameters
  // 

  move_ = nh_.param<bool>("move_target", true);
  acceleration_ = nh_.param<float>("acceleration", 1);
  delta_t_ = 2.4/acceleration_;

  // Get relative waypoint lists for x, y and z
  std::vector<double> waypoints_x, waypoints_y, waypoints_z;
  nh_.getParam("waypoints_x", waypoints_x);
  nh_.getParam("waypoints_y", waypoints_y);
  nh_.getParam("waypoints_z", waypoints_z);

  // If there are no waypoints, add one 0 waypoint
  if (waypoints_x.size() == 0 && waypoints_y.size() == 0 && waypoints_z.size() == 0)
    waypoints_x.push_back(0);

  // make all waypoint vectors the same size (zero pad)
  int maxWPs = std::max(waypoints_x.size(), std::max(waypoints_y.size(), waypoints_z.size()));
  std::fill_n(std::back_inserter(waypoints_x), maxWPs-waypoints_x.size(), 0);
  std::fill_n(std::back_inserter(waypoints_y), maxWPs-waypoints_y.size(), 0);
  std::fill_n(std::back_inserter(waypoints_z), maxWPs-waypoints_z.size(), 0);

  // Construct a waypoint container
  gzmsg << "waypoints:" << std::endl;
  for (int i=0; i<waypoints_x.size(); i++)
  {
    // translate body-relative waypoints to the world frame
    GazeboVector pos = GZ_COMPAT_GET_POS(pose_init_);
    motion::coord_t waypoint(GZ_COMPAT_GET_X(pos) + waypoints_x[i],
                             GZ_COMPAT_GET_Y(pos) + waypoints_y[i],
                             GZ_COMPAT_GET_Z(pos) + waypoints_z[i]);
    waypoints_init_.push_back(waypoint);

    gzmsg << "\t" << waypoints_init_[i] << std::endl;
  }

  // trajectory type 
  params_.traj = nh_.param<int>("trajectory_type", 0);

  // Orbit circle parameters
  params_.rho = nh_.param<float>("radius", 2);
  params_.lambda = nh_.param<float>("lambda", 1);

  // velocity
  v_ = nh_.param<float>("v", 1);

  std::vector<std::string> traj_types = {"goToPoint", "waypoints", "circle", "ellipse", "random_waypoints"};


  // Orbit trajectory
  if (params_.traj == 2) {
    follower_ = std::make_shared<motion::Orbit>();
  } else if (params_.traj == 0 || params_.traj == 1 || params_.traj == 3) {
    follower_ = std::make_shared<motion::StraightLine>();
  } else if (params_.traj == 4) {
    follower_ = std::make_shared<motion::RandomWaypoints>(waypoints_init_);

    // Pick a random initial condition
    motion::coord_t pos0 = std::static_pointer_cast<motion::RandomWaypoints>(follower_)->initialize();
    pose_init_ = GazeboPose(std::get<0>(pos0), std::get<1>(pos0), std::get<2>(pos0), 0, 0, 0);
    model_->SetWorldPose(pose_init_);
  } else {
    gzerr << "[TargetMotion] Trajectory type " << params_.traj << " is undefined.\n";
  }
  
  gzmsg << "[TargetMotion] Generated a " << traj_types[params_.traj] << " trajectory for " << model_->GetName() << ".\n";

  gzmsg << "params: " << "\n\tk_orbit:      " << params_.k_orbit
                      << "\n\trho:          " << params_.rho
                      << "\n\tlambda:       " << params_.lambda
                      << "\n\tk_path:       " << params_.k_path
                      << "\n\tchi_infinity: " << params_.chi_infinity << std::endl;


  follower_->set_parameters(params_);

  // Set current waypoints
  waypoints_curr_ = waypoints_init_;
}

// ------------------------------------------------------------------------

void TargetMotion::loadManager() {

  // manager type 
  bool half_plane = nh_.param<bool>("half_plane", false);

  if (half_plane) {
    manager_ = std::unique_ptr<motion::HalfPlaneManager>(new motion::HalfPlaneManager());
  } else {
    manager_ = std::unique_ptr<motion::RadiusManager>(new motion::RadiusManager());
  }

  // goToPoint only goes to point once
  // random waypoints manages its own waypoints
  bool should_cycle = (params_.traj != 0 && params_.traj != 4);
  manager_->set_cycling(should_cycle);
}

// ------------------------------------------------------------------------

void TargetMotion::OnUpdate(const common::UpdateInfo& _info)
{

  // calculate the timestep
  double dt = (simTime_d1_ == 0) ? 0.0001 : _info.simTime.Double() - simTime_d1_;

  if (dt > 1.0/update_rate_ || simTime_d1_ == 0)
  {  
   
    GazeboVector linear_vel(0, 0, 0); 
    GazeboVector angular_vel(0, 0, 0);

    float chi_er, h_er, yaw, distance;

    getCommandError(chi_er, h_er, yaw, distance);

    getVelCommands(_info, chi_er, h_er, yaw, distance, dt, linear_vel, angular_vel);
    

    // Set commands
    model_->SetLinearVel(linear_vel);
    model_->SetAngularVel(angular_vel);


    // capture the current simulation time for next iteration
    simTime_d1_ = _info.simTime.Double();

    PublishState();
  }
}

// ------------------------------------------------------------------------

void TargetMotion::getCommandError(float& chi_er, float& h_er, float& yaw, float& distance) {

  // Get world pose
  GazeboPose pose = GZ_COMPAT_GET_WORLD_POSE(model_);

  // Get position and pack into type motion::coord_t
  GazeboVector position = GZ_COMPAT_GET_POS(pose);
  motion::coord_t pos(GZ_COMPAT_GET_X(position), 
                      GZ_COMPAT_GET_Y(position),
                      GZ_COMPAT_GET_Z(position));

  // Get Euler angles
  GazeboVector rot = GZ_COMPAT_GET_EULER(GZ_COMPAT_GET_ROT(pose));

  // Manage waypoints
  bool wp_reached = manager_->manage_waypoints(pos, waypoints_curr_, distance);

  // Heading
  yaw = GZ_COMPAT_GET_Z(rot);

  // Get heading and altitude commands
  motion::FollowerCommands commands;
  if (params_.traj == 2)
    commands = follower_->orbit_follower(waypoints_curr_[0], pos, yaw);
  else if (params_.traj == 4)
    commands = follower_->randomize(waypoints_curr_, wp_reached, pos, yaw);
  else
    commands = follower_->line_follower(waypoints_curr_[0], waypoints_curr_[1]-waypoints_curr_[0], pos, yaw);

  // calculate altitude and heading errors
  h_er = std::get<2>(pos) - commands.h_c;
  chi_er =  yaw - commands.chi_c;

}

// ------------------------------------------------------------------------

void TargetMotion::getVelCommands(const common::UpdateInfo& _info, double chi_er, double h_er, double yaw, double distance, double dt, GazeboVector& linear_vel, GazeboVector& angular_vel)
{

  //
  // calculate angular and linear velocity commands
  //

  double vz = altitudePID_.Update(h_er, dt);
  double wz = headingPID_.Update(chi_er, dt);

  // If goToPoint trajectory and we have arrived, then
  // trigger the exponential slowing down and stop moving
  if (params_.traj == 0 && distance < 0.3) move_ = false;

  //
  // Rotate commands into the current frame
  //

  // linear velocity command
  GazeboVector vel (v_, 0, vz);

  // Rotation Matrix
  GazeboMatrix rotz(cos(yaw), -sin(yaw), 0.0,
                     sin(yaw),  cos(yaw), 0.0,
                     0.0,       0.0,      1.0);  

  // Rotate linear commands into current frame
  linear_vel = rotz*vel;

  // Get angular vel
  angular_vel = GazeboVector(0, 0, wz);


  float scale;
  if (move_)
  {

    delta_t_ -= dt;

    scale = std::exp(-delta_t_*acceleration_);

    if (scale > 1)
    {
      scale = 1;
      delta_t_ = 0;
    }

  }
  else
  {

    delta_t_ += dt;

    scale = std::exp(-delta_t_*acceleration_);

    if (scale < 0.1)
    {
      delta_t_ = 2.4/acceleration_;  
      scale = 0;
    }

  }


  linear_vel = linear_vel*scale;
  angular_vel = angular_vel*scale;
}

// ------------------------------------------------------------------------

// void TargetMotion::drawWaypoins(const motion::waypoints_t waypoints) {

//   dynamic_lines_ = visual_->CreateDynamicLine();

//   std::cout << "size: " << waypoints.size() << std::endl;

//   for (int i = 0; i < waypoints.size(); i++) {

//     dynamic_lines_->AddPoint(
//       GazeboVector(
//         std::get<0>(waypoints[i]), 
//         std::get<1>(waypoints[i]), 
//         std::get<2>(waypoints[i]))
//       ); 


//     std::cout << "here" << std::endl;
//   }

//   dynamic_lines_->setMaterial("Gazebo/Purple");
//   dynamic_lines_->setVisibilityFlags(GZ_VISIBILITY_GUI);
//   dynamic_lines_->setVisible(true);

//   // dynamic_lines_.Update();

// }


// ------------------------------------------------------------------------


double TargetMotion::getValueFromSdf(std::string name) {

  if (sdf_pointer_->HasElement(name))
    return sdf_pointer_->GetElement(name)->Get<double>();
  else
    gzerr << "[TargetMotion] Please specify the <" << name << "> element.\n";
    
   return 0;

}

// ------------------------------------------------------------------------

bool TargetMotion::targetService(moving_targets::MovingTargets::Request &req, moving_targets::MovingTargets::Response &res) {

  move_ = req.move;

  // Reset the agent to initial pose and reset the waypoints
  if (req.reset == true) {

    waypoints_curr_ = waypoints_init_;
    model_->SetWorldPose(pose_init_);

    // Reset PID controllers
    headingPID_.Reset();
    altitudePID_.Reset();

    delta_t_ = 2.4/acceleration_;
  }

  return true;

}

// ------------------------------------------------------------------------

void TargetMotion::PublishState() {

  // publish mover's state odometry
  nav_msgs::Odometry odom;
  odom.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(model_->GetWorld()).sec;
  odom.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(model_->GetWorld()).nsec;
  odom.header.frame_id = "map";
  odom.child_frame_id = "target_" + model_->GetName();

  GazeboPose pose = GZ_COMPAT_GET_WORLD_POSE(model_);
  GazeboVector pos = GZ_COMPAT_GET_POS(pose);
  GazeboQuaternion quat = GZ_COMPAT_GET_ROT(pose);
  odom.pose.pose.position.x     = GZ_COMPAT_GET_X(pos);
  odom.pose.pose.position.y     = GZ_COMPAT_GET_Y(pos);
  odom.pose.pose.position.z     = GZ_COMPAT_GET_Z(pos);
  odom.pose.pose.orientation.w  = GZ_COMPAT_GET_W(quat);
  odom.pose.pose.orientation.x  = GZ_COMPAT_GET_X(quat);
  odom.pose.pose.orientation.y  = GZ_COMPAT_GET_Y(quat);
  odom.pose.pose.orientation.z  = GZ_COMPAT_GET_Z(quat);

  GazeboVector linvel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(model_);
  GazeboVector angvel = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(model_);

  odom.twist.twist.linear.x     = GZ_COMPAT_GET_X(linvel);
  odom.twist.twist.linear.y     = GZ_COMPAT_GET_Y(linvel);
  odom.twist.twist.linear.z     = GZ_COMPAT_GET_Z(linvel);
  odom.twist.twist.angular.x    = GZ_COMPAT_GET_X(angvel);
  odom.twist.twist.angular.y    = GZ_COMPAT_GET_Y(angvel);
  odom.twist.twist.angular.z    = GZ_COMPAT_GET_Z(angvel);
  state_.publish(odom);
}

// ------------------------------------------------------------------------

void TargetMotion::turnOffCollisions() {
  auto links = model_->GetLinks();

  // for each link in this model, turn off collisions!
  for (auto&& l : links) {
    l->SetCollideMode("none");
  }

}

}
