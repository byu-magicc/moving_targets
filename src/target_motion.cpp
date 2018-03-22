#include "target_motion.h"

namespace gazebo
{

// void TargetMotion::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {

//   visual_ = _parent;

// }

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


  // Init PID controllers
  headingPID_.Init(kpPsi, 0, kdPsi, 0,0, maxVPsi, -maxVPsi);
  altitudePID_.Init(kpZ, 0, kdZ, 0,0, maxVZ, -maxVZ);


  // Load trajectory
  loadTrajectory();

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetMotion::OnUpdate, this, _1));

}

// ------------------------------------------------------------------------

void TargetMotion::loadTrajectory() {

  //
  // Get ROS parameters
  // 

  // Get initial position
  float x = nh_.param<float>("x",0);
  float y = nh_.param<float>("y",0);
  float z = nh_.param<float>("z",0);

  // Get relative waypoint lists for x, y and z
  std::vector<double> waypoints_x, waypoints_y, waypoints_z;
  nh_.getParam("waypoints_x", waypoints_x);
  nh_.getParam("waypoints_y", waypoints_y);
  nh_.getParam("waypoints_z", waypoints_z);

  // Construct a waypoint container
  gzmsg << "waypoints: \n";
  for (int i=0; i<waypoints_x.size(); i++) {
    motion::coord_t waypoint (x + waypoints_x[i], y + waypoints_y[i], z + waypoints_z[i]);
    waypoints_.push_back(waypoint); // translate waypoints to origin

    gzmsg << waypoints_[i] << "\n";
  }

  // trajectory type 
  params_.traj = nh_.param<int>("trajectory_type", 0);

  // Orbit circle parameters
  params_.row = nh_.param<float>("radius", 2);
  params_.lambda = nh_.param<float>("lambda", 1);

  // velocity
  v_ = nh_.param<float>("v",1);

  std::vector<std::string> traj_types = {"goToPoint", "waypoints", "circle", "elipse"};


  // Orbit trajectory
  if (params_.traj == 2) {

    follower_ = std::make_shared<motion::Orbit>();
    gzmsg << "[TargetMotion] Generated a " << traj_types[params_.traj] << " trajectory for " << model_->GetName() << ".\n";

  }
  else if (params_.traj == 0 || params_.traj == 1 || params_.traj == 3) {

    follower_ = std::make_shared<motion::StraightLine>();
    gzmsg << "[TargetMotion] Generated a " << traj_types[params_.traj] << " trajectory for " << model_->GetName() << ".\n";
    

  }
  else {
    gzerr << "[TargetMotion] Trajectory type " << params_.traj << " is undefined.\n";
  }

  gzmsg << "params: " << "\n k_orbit: " << params_.k_orbit
                      << "\n row: " << params_.row
                      << "\n lambda: " << params_.lambda
                      << "\n k_path: " << params_.k_path
                      << "\n chi_infinity: " << params_.chi_infinity <<"\n";


  follower_->set_parameters(params_);


}

// ------------------------------------------------------------------------

void TargetMotion::OnUpdate(const common::UpdateInfo& _info)
{
  
  // calculate the timestep
   double dt = (simTime_d1_ == 0) ? 0.0001 : _info.simTime.Double() - simTime_d1_;
   simTime_d1_ = _info.simTime.Double();


  // Get world pose
  math::Pose pose = model_->GetWorldPose();

  // Get position and pack into type motion::coord_t
  math::Vector3 position = pose.pos;
  motion::coord_t pos (position.x, position.y, position.z);

  // Get Euler angles
  math::Vector3 rot = pose.rot.GetAsEuler();

  // Manage waypoints
  float distance = radius_manager_.manage_waypoints(pos, waypoints_, params_.traj);

  // Heading
  float yaw = rot.z;

  // Get heading and altitude commands
  motion::FollowerCommands commands;
  if (params_.traj == 2) {

    // std::cout << "here" << std::endl;
    commands = follower_->orbit_follower(waypoints_[0],pos, yaw);

  } 
  else {

   commands = follower_->line_follower(waypoints_[0], waypoints_[1]-waypoints_[0],pos, yaw);

  }

  // calculate heading and altitude errors
  float h_er = std::get<2>(pos) - commands.h_c;
  float chi_er =  yaw - commands.chi_c;

  VelController(chi_er, h_er, yaw, distance, dt);


  PublishState();


}

// ------------------------------------------------------------------------

void TargetMotion::VelController(double chi_er, double h_er, double yaw, double distance, double dt)
{
  
 //
 // calculate angular and linear velocity commands
 //

 double vz = altitudePID_.Update(h_er, dt);
 double wz = headingPID_.Update(chi_er, dt);

 // If goToPoint trajector and there
 if (params_.traj == 0 && distance < 0.3) {
  v_ = 0;
  wz = 0;
  vz = 0;
 }


 //
 // Rotate commands into the current frame
 //

 // linear velocity command
 math::Vector3 vel (v_, 0, vz);

 // Rotation Matrix
 math::Matrix3 rotz(cos(yaw), -sin(yaw), 0.0,
              sin(yaw),  cos(yaw), 0.0,
              0.0,       0.0,    1.0);  

 // Rotate commands into current frame
 math::Vector3 command = rotz*vel;

 // Set commands
 model_->SetLinearVel(command);
 model_->SetAngularVel(ignition::math::Vector3d(0,0,wz));


}

// ------------------------------------------------------------------------

// void TargetMotion::drawWaypoins(const motion::waypoints_t waypoints) {

//   dynamic_lines_ = visual_->CreateDynamicLine();

//   std::cout << "size: " << waypoints.size() << std::endl;

//   for (int i = 0; i < waypoints.size(); i++) {

//     dynamic_lines_->AddPoint(
//       math::Vector3(
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
