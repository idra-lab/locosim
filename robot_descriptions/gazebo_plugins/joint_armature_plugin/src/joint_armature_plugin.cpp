// Reflected rotor inertia ("armature") on ODE joints, the way PhysX and MuJoCo model it.
//
// A geared joint feels its motor's rotor inertia multiplied by the square of the gear ratio. On a
// quadruped that is the largest inertia the joint sees - on the Go2 0.017-0.038 kg m^2 of it,
// against 0.005 kg m^2 for the whole lower leg about the knee - and Isaac adds it to the joint as
// armature: a term on the diagonal of the joint-space inertia and nowhere else. ODE has no such
// parameter. Putting it into the child link's inertia is not equivalent (the link's inertia also
// turns with every joint upstream, so the knee's armature lands on the thigh and the trunk too),
// and a geared rotor body closes a kinematic loop that set_model_configuration then refuses to
// move, which breaks every go0 posture at spawn.
//
// What is done here instead is to add, for each listed joint, one ODE angular motor between the
// joint's child and parent bodies, along the joint axis. Before every physics step its target
// velocity is set to the joint's current rate and its constraint force mixing to dt / armature.
// ODE then solves the motor row as  J v' = v - CFM f,  i.e.  f = -armature (v' - v) / dt : exactly
// a joint torque -armature * qdd, integrated implicitly, so it is stable for any armature and acts
// on that joint's coordinate alone. The motor is not a Gazebo joint, so it is invisible to the
// link graph (set_model_configuration works) and it leaves the hinge's own motor row free for the
// <dynamics friction> Gazebo implements with it.
//
// Usage, in a <gazebo> block of the robot description:
//   <plugin name="joint_armature" filename="libjoint_armature_plugin.so">
//     <joint name="lf_haa_joint">0.0175</joint>   <!-- kg m^2 -->
//     ...
//   </plugin>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/ode/ode.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>

#include <string>
#include <vector>

namespace gazebo
{
class JointArmaturePlugin : public ModelPlugin
{
public:
  ~JointArmaturePlugin() override
  {
    this->updateConnection.reset();
    for (const Motor &motor : this->motors)
      dJointDestroy(motor.id);
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->world = model->GetWorld();
    auto ode = boost::dynamic_pointer_cast<physics::ODEPhysics>(this->world->Physics());
    if (!ode)
    {
      gzerr << "[joint_armature] needs the ODE physics engine; no armature applied\n";
      return;
    }
    if (!sdf->HasElement("joint"))
    {
      gzwarn << "[joint_armature] no <joint> listed; nothing to do\n";
      return;
    }
    for (sdf::ElementPtr element = sdf->GetElement("joint"); element;
         element = element->GetNextElement("joint"))
    {
      const std::string name = element->Get<std::string>("name");
      const double armature = element->Get<double>();
      physics::JointPtr joint = model->GetJoint(name);
      if (!joint)
      {
        gzerr << "[joint_armature] model '" << model->GetName() << "' has no joint '" << name
              << "'\n";
        continue;
      }
      if (armature <= 0.0)
        continue;
      auto child = boost::dynamic_pointer_cast<physics::ODELink>(joint->GetChild());
      auto parent = boost::dynamic_pointer_cast<physics::ODELink>(joint->GetParent());
      if (!child)
      {
        gzerr << "[joint_armature] joint '" << name << "' has no child body\n";
        continue;
      }
      Motor motor;
      motor.child = child->GetODEId();
      motor.parent = parent ? parent->GetODEId() : nullptr;
      motor.armature = armature;
      motor.id = dJointCreateAMotor(ode->GetWorldId(), nullptr);
      dJointAttach(motor.id, motor.child, motor.parent);
      dJointSetAMotorMode(motor.id, dAMotorUser);
      dJointSetAMotorNumAxes(motor.id, 1);
      // rel = 1: the axis is stored in the child body's frame, so it turns with the joint
      const ignition::math::Vector3d axis = joint->GlobalAxis(0);
      dJointSetAMotorAxis(motor.id, 0, 1, axis.X(), axis.Y(), axis.Z());
      // The motor never has to deliver more than armature x the largest joint acceleration;
      // the bound only has to be out of the way.
      dJointSetAMotorParam(motor.id, dParamFMax, 1e4);
      this->motors.push_back(motor);
      gzmsg << "[joint_armature] " << name << ": " << armature << " kg m^2\n";
    }
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&JointArmaturePlugin::OnUpdate, this));
  }

private:
  struct Motor
  {
    dJointID id;
    dBodyID child;
    dBodyID parent;
    double armature;
  };

  // Runs before every physics step: hold each joint's current rate, softened into an inertia.
  void OnUpdate()
  {
    const double dt = this->world->Physics()->GetMaxStepSize();
    static const dReal zero[3] = {0.0, 0.0, 0.0};
    for (const Motor &motor : this->motors)
    {
      dVector3 axis;
      dJointGetAMotorAxis(motor.id, 0, axis);
      const dReal *w_child = dBodyGetAngularVel(motor.child);
      const dReal *w_parent = motor.parent ? dBodyGetAngularVel(motor.parent) : zero;
      const double rate = (w_child[0] - w_parent[0]) * axis[0] +
                          (w_child[1] - w_parent[1]) * axis[1] +
                          (w_child[2] - w_parent[2]) * axis[2];
      dJointSetAMotorParam(motor.id, dParamVel, rate);
      dJointSetAMotorParam(motor.id, dParamCFM, dt / motor.armature);
    }
  }

  physics::WorldPtr world;
  std::vector<Motor> motors;
  event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(JointArmaturePlugin)
}  // namespace gazebo
