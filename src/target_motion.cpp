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
      // Connect to ROS
      //

      nh = ros::NodeHandle(model->GetName());

      // Connect to subscribers
      gzmsg << "[TargetMotion] Subscribing to " << ("/" + model->GetName() + "/command") << "\n";

      //
      // Initialize the unicycle motion planner
      //

      unicycle = std::unique_ptr<motion::UnicyclePlanner>(new motion::UnicyclePlanner());


      motion::waypoints_t waypoints = {{5,5},{10,8},{7,0},{15,-3},{15,4},{-2,4}};
      motion::coord_t vel = {4, 4};
      unicycle->generateWaypoints(waypoints, vel);

      double radius = 2;
      motion::coord_t center = {5, 5};
      // unicycle->generateCircle(radius, center);

      double a = 2;
      center = {0, 0};
      // unicycle->generateLemniscate(a, center);

      unicycle->goToPoint(20, -8.5);

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

      // VelController(0, 3.14159, dt);

      // return;

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

      // speed --> force in body x-axis
      double c = 2.0/vel; // inverse velocity clamp
      double linearVel_error = math::clamp(link->GetRelativeLinearVel().x - vel, -c, c);
      double Fx = forcePID.Update(linearVel_error, dt);
      if (Fx == maxFXY) gzerr << "XY clamped (" << Fx << ") !!\n";
      link->AddRelativeForce(math::Vector3(Fx, 0, 0));

      // Heading rate --> angular velocity
      double angularVel_error = link->GetRelativeAngularVel().z - omega;
      double Fw = torquePID.Update(angularVel_error, dt);
      if (Fw == maxFOmega) gzerr << "Omega clamped (" << Fw << ") !!\n";
      link->AddRelativeTorque(math::Vector3(0, 0, Fw));

      gzmsg << "Linear Velocity (" << link->GetRelativeLinearVel().x << ") Error: " << vel - link->GetRelativeLinearVel().x << "\t" << "Force: " << Fx << "\n";
      gzmsg << "Angular Velocity (" << link->GetRelativeAngularVel().z << ") Error: " << omega - link->GetRelativeAngularVel().z << "\t" << "Torque: " << Fw << "\n";
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