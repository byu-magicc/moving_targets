#include <memory>
#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

#include "unicycle_planner.h"

namespace gazebo
{
  class TargetMotion : public ModelPlugin
  {
  public:

    double getValueFromSdf(std::string name)
    {
      if (sdf_pointer->HasElement(name))
        return sdf_pointer->GetElement(name)->Get<double>();
      else
        gzerr << "[TargetMotion] Please specify the <" << name << "> element.\n";
        
      return 0;
    }

    // ------------------------------------------------------------------------

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointers to the model and to the sdf
      model = _parent;
      sdf_pointer = _sdf;

      //
      // Find the link to apply force to
      //

      std::string link_name;
      if (sdf_pointer->HasElement("baseLink"))
        link_name = sdf_pointer->GetElement("baseLink")->Get<std::string>();
      else
        gzthrow("[TargetMotion] Please specify a <baseLink> element.");

      link = model->GetLink(link_name);
      if (link == nullptr)
        gzthrow("[TargetMotion] Could not find specified link \"" << link_name << "\".");

      //
      // Get SDF Values
      //

      kpXY      = getValueFromSdf("kpXY");
      kpOmega   = getValueFromSdf("kpOmega");
      maxFXY    = getValueFromSdf("maxFXY");
      maxFOmega = getValueFromSdf("maxFOmega");

      //
      // Initialize the unicycle motion planner
      //

      unicycle = std::unique_ptr<motion::UnicyclePlanner>(new motion::UnicyclePlanner());

      //
      // Connect to ROS
      //

      nh = ros::NodeHandle(model->GetName());

      // Load the trajectory parameters based on the desired type 
      int traj = nh.param<int>("trajectory_type", 0);
      if (traj == 0)
      { // point

        double x = nh.param<double>("x", 5);
        double y = nh.param<double>("y", 5);

        gzmsg << "[TargetMotion] Generated a goToPoint trajectory for " << model->GetName() << ".\n";
        unicycle->goToPoint(x, y);
      }
      else if (traj == 1)
      { // waypoints

        // Get origin of waypoints
        double x = nh.param<double>("x", 0);
        double y = nh.param<double>("y", 0);

        // Get waypoint lists for x and y
        std::vector<double> waypoints_x, waypoints_y;
        nh.getParam("waypoints_x", waypoints_x);
        nh.getParam("waypoints_y", waypoints_y);

        // Construct a waypoint container
        motion::waypoints_t waypoints;
        for (int i=0; i<waypoints_x.size(); i++)
          waypoints.push_back({ x + waypoints_x[i], y + waypoints_y[i]}); // translate waypoints to origin

        double vx = nh.param<double>("vx", 1);
        double vy = nh.param<double>("vy", 1);

        gzmsg << "[TargetMotion] Generated a waypoint trajectory for " << model->GetName() << ".\n";
        unicycle->generateWaypoints(waypoints, {vx, vy});
      }
      else if (traj == 2)
      { // circle

        double radius = nh.param<double>("radius", 2);
        double x = nh.param<double>("x", 5);
        double y = nh.param<double>("y", 5);

        gzmsg << "[TargetMotion] Generated a circle trajectory for " << model->GetName() << ".\n";
        unicycle->generateCircle(radius, {x, y});
      }
      else if (traj == 3)
      { // lemniscate (figure-8)

        double a = nh.param<double>("a", 2);
        double x = nh.param<double>("x", 5);
        double y = nh.param<double>("y", 5);

        gzmsg << "[TargetMotion] Generated a lemniscate trajectory for " << model->GetName() << ".\n";
        unicycle->generateLemniscate(a, {x, y});
      }
      else
      { // undefined
        gzerr << "[TargetMotion] Trajectory type " << traj << " is undefined.\n";
      }      

      //
      // Initialize low-level PID controllers
      //

      forcePID.Init(kpXY, 100000, 0, 100000, -100000, maxFXY, -maxFXY);
      torquePID.Init(kpOmega, 1000, 0, 1000, -1000, maxFOmega, -maxFOmega);

      // Listen to the update event. This event is broadcast every simulation iteration.
      updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetMotion::OnUpdate, this, _1));
    }

    // ------------------------------------------------------------------------

    void OnUpdate(const common::UpdateInfo& _info)
    {
      // calculate the timestep
      double dt = (simTime_d1_ == 0) ? 0.0001 : _info.simTime.Double() - simTime_d1_;
      simTime_d1_ = _info.simTime.Double();

      // Update the unicycle motion planner with the current state of the robot
      // as integrated / propagated from the Gazebo physics engine
      math::Vector3 pos = link->GetWorldPose().pos;
      math::Vector3 rot = link->GetWorldPose().rot.GetAsEuler();
      unicycle->updateState(pos.x, pos.y, rot.z);

      // Use the unicycle motion planner to generate speed and heading rate commands
      double v, w;
      unicycle->getCommands(dt, v, w);

      // Command the Gazebo robot to achieve the desired speed and heading rate
      VelController(v, w, dt);
    }

    // ------------------------------------------------------------------------

    void VelController(double vel, double omega, double dt)
    {
      // Apply forces to the model (using P control) to achieve
      // the commanded linear and angular velocities.

      // Check if robot has fallen down
      if (link->GetWorldPose().pos.z > 0.1 || 
          std::abs(link->GetWorldPose().rot.GetAsEuler().x) > 0.1 ||
          std::abs(link->GetWorldPose().rot.GetAsEuler().y) > 0.1)
      {
        gzerr << "I've fallen and can't get up!\n";
        LifeCall(); // Help Mrs. Fletcher stand up
        return;
      }

      // speed --> force in body x-axis
      double c = 2.0/vel; // inverse velocity clamp
      double linearVel_error = math::clamp(link->GetRelativeLinearVel().x - vel, -c, c);
      double Fx = forcePID.Update(linearVel_error, dt);
      // if (Fx == maxFXY) gzerr << "XY clamped (" << Fx << ") !!\n";
      link->AddRelativeForce(math::Vector3(Fx, 0, 0));

      // Heading rate --> angular velocity
      double angularVel_error = link->GetRelativeAngularVel().z - omega;
      double Fw = torquePID.Update(angularVel_error, dt);
      // if (Fw == maxFOmega) gzerr << "Omega clamped (" << Fw << ") !!\n";
      link->AddRelativeTorque(math::Vector3(0, 0, Fw));

      // gzmsg << "Linear Velocity (" << link->GetRelativeLinearVel().x << ") Error: " << vel - link->GetRelativeLinearVel().x << "\t" << "Force: " << Fx << "\n";
      // gzmsg << "Angular Velocity (" << link->GetRelativeAngularVel().z << ") Error: " << omega - link->GetRelativeAngularVel().z << "\t" << "Torque: " << Fw << "\n";
    }

    // ------------------------------------------------------------------------

    void LifeCall()
    {
      // start with the current pose
      math::Pose resetPose = link->GetWorldPose();
      resetPose.pos.z = 0;

      // zero out the roll, pitch orientations
      math::Vector3 euler = resetPose.rot.GetAsEuler();
      euler.x = 0; euler.y = 0;
      resetPose.rot.SetFromEuler(euler);

      // teleport to a standing position
      link->SetWorldPose(resetPose);

      // Because we just teleported the model, reset the
      // vel, accel, force, torque on the link ...
      link->ResetPhysicsStates();

      // ... and the PIDs
      forcePID.Reset();
      torquePID.Reset();
    }

    // ------------------------------------------------------------------------

  private:
    // Pointer to the model
    sdf::ElementPtr sdf_pointer;
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

    double simTime_d1_ = 0;

    common::PID forcePID;
    common::PID torquePID;

    // parameters
    double kpXY;
    double kpOmega;
    double maxFXY;
    double maxFOmega;

    // ROS stuff
    ros::NodeHandle nh;

    std::unique_ptr<motion::UnicyclePlanner> unicycle;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMotion)
}