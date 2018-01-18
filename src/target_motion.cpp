#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

namespace gazebo
{
  class TargetMotion : public ModelPlugin
  {
  public:
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
      friction  = getValueFromSdf("friction");

      //
      // Connect to ROS
      //

      nh = ros::NodeHandle(model->GetName());

      // Connect to subscribers
      gzmsg << "[TargetMotion] Subscribing to " << ("/" + model->GetName() + "/command") << "\n";


      // Listen to the update event. This event is broadcast every simulation iteration.
      updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetMotion::OnUpdate, this, _1));
    }


    double getValueFromSdf(std::string name)
    {
      if (sdf_pointer->HasElement(name))
        return sdf_pointer->GetElement(name)->Get<double>();
      else
        gzerr << "[TargetMotion] Please specify the <" << name << "> element.\n";
        
      return 0;
    }


    int sgn(double val)
    {
      return (0 < val) - (val < 0);
    }

    void saturate2(double& Fx, double& Fy, const double Fmax)
    {
      double F = sqrt(Fx*Fx + Fy*Fy);
      if (fabs(F) > Fmax)
      {
        Fx *= Fmax / F;
        Fy *= Fmax / F;
      }
    }   

    void saturate(double& F, const double Fmax)
    {
      if (fabs(F) > Fmax)
        F = Fmax*sgn(F);
    }


    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo& _info)
    {
      // Apply forces to the model (using P control) to achieve the commanded linear and angular velocities.
      math::Vector3 linearVel = link->GetWorldLinearVel();
      math::Vector3 angularVel = link->GetWorldAngularVel();
      double Fx = (2 - linearVel.x)*kpXY;
      double Fy = (0 - linearVel.y)*kpXY;
      double Fw = (1 - angularVel.z)*kpOmega;
      saturate2(Fx, Fy, maxFXY);
      saturate(Fw, maxFOmega);

      link->AddForceAtRelativePosition(math::Vector3(Fx, Fy, 0), math::Vector3(0, 0, 0));
      link->AddTorque(math::Vector3(0, 0, Fw));

      // Artificially add friction
      math::Vector3 vel = link->GetWorldLinearVel();
      math::Vector3 friction_force = -vel*friction;
      link->AddForce(friction_force);
    }


  private:
    // Pointer to the model
    sdf::ElementPtr sdf_pointer;
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

    // parameters
    double kpXY;
    double kpOmega;
    double maxFXY;
    double maxFOmega;
    double friction;

    // ROS stuff
    ros::NodeHandle nh;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMotion)
}