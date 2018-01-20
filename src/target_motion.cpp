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

      // gzmsg << "\n\n";
      // gzmsg << "GetRelativePose: " << link->GetRelativePose() << "\n";
      // gzmsg << "GetInitialRelativePose: " << link->GetInitialRelativePose() << "\n";
      // gzmsg << "GetWorldCoGPose: " << link->GetWorldCoGPose() << "\n";
      // gzmsg << "GetWorldInertialPose: " << link->GetWorldInertialPose() << "\n";
      // gzmsg << "GetWorldPose: " << link->GetWorldPose() << "\n";

      // gzmsg << "\n\n";
      // gzmsg << "GetRelativeLinearVel: " << link->GetRelativeLinearVel() << "\n";
      // gzmsg << "GetWorldCoGLinearVel: " << link->GetWorldCoGLinearVel() << "\n";
      // gzmsg << "GetWorldLinearVel: " << link->GetWorldLinearVel() << "\n";

      // gzmsg << "\n\n";
      // gzmsg << "GetRelativeAngularVel: " << link->GetRelativeAngularVel() << "\n";
      // gzmsg << "GetWorldAngularVel: " << link->GetWorldAngularVel() << "\n";

      // gzmsg << "=======================================================================================\n\n";

      math::Vector3 vel = PosController(math::Vector3(10, 10, 0));


      VelController(vel.x, vel.z);
    }

    math::Vector3 PosController(const math::Vector3& pos)
    {
      math::Vector3 currentPos = link->GetWorldPose().pos;
      math::Vector3 currentRot = link->GetWorldPose().rot.GetAsEuler();

      // Calculate error from current position to goal position
      math::Vector3 e = pos - currentPos;

      // Distance control
      double d = sqrt( e.x*e.x + e.y*e.y );
      double v = d*1;

      // heading control
      double goalHeading = atan2(e.y, e.x);
      double angleDiff = goalHeading - currentRot.z;
      
      // angle wrapping to ensure angleDiff \in [-pi, pi]
      if (angleDiff > M_PI) angleDiff -= 2*M_PI;
      if (angleDiff < -M_PI) angleDiff += 2*M_PI;

      double w = angleDiff*4;

      return math::Vector3(v, 0, w);
    }

    void VelController(double vel, double omega)
    {
      // Apply forces to the model (using P control) to achieve the commanded linear and angular velocities.
      math::Vector3 linearVel = link->GetRelativeLinearVel();
      double Fx = (vel - linearVel.x)*kpXY;
      saturate(Fx, maxFXY);
      link->AddRelativeForce(math::Vector3(Fx, 0, 0));


      math::Vector3 angularVel = link->GetRelativeAngularVel();
      double Fw = (omega - angularVel.z)*kpOmega;
      saturate(Fw, maxFOmega);
      link->AddRelativeTorque(math::Vector3(0, 0, Fw));

      // Artificially add friction
      link->AddForce( -link->GetWorldLinearVel()*friction );
      link->AddTorque( -link->GetWorldAngularVel()*friction );
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