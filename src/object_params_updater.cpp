#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/Float32MultiArray.h>

#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

using namespace gazebo;
class ModelPush : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;
    //this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));

    // ros node
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "model_params_updater",
                ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle("model_params_updater"));

    // subscriber
    ros::SubscribeOptions so
      = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/object_plugin/params",1,
                                                         boost::bind(&ModelPush::OnRosMsg, this, _1),
                                                         ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&ModelPush::QueueThread, this));
  }

  void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    physics::LinkPtr link = model->GetLink("link");
    physics::InertialPtr inertial = link->GetInertial();

    // update mass
    inertial->SetMass(msg->data.at(0));

    // update cog
    //math::Vector3 cog = inertial->GetCoG();
    inertial->SetCoG(msg->data.at(1), msg->data.at(2), 0, 0, 0, 0);
    //auto pos = model->WorldCoGPose();//.Pos();
    //std::cout << pos.x << " " << pos.y << " " << pos.z << std::endl;

    // update pos

    // update friction
    physics::CollisionPtr col = link->GetCollision("collision");
    physics::SurfaceParamsPtr surface = col->GetSurface();
    surface->FrictionPyramid()->SetMuPrimary(msg->data.at(3));
    surface->FrictionPyramid()->SetMuSecondary(msg->data.at(4));

    link->Update();
  }

  /// \brief ROS helper function that processes messages
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
  }

  /*
  // Called by the world update start event
  void OnUpdate()
  {
  physics::LinkPtr link = model->GetLink("link");
  physics::CollisionPtr col = link->GetCollision("collision");
  physics::SurfaceParamsPtr surface = col->GetSurface();
  surface->FrictionPyramid()->SetMuPrimary(0.001);
  surface->FrictionPyramid()->SetMuSecondary(0.001);
  }
  */

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;

  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
