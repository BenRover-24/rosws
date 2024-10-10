#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh> 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>  

namespace gazebo
{
class ServoSimPlugin : public ModelPlugin
{
public:
  ServoSimPlugin() : ModelPlugin(), node_(std::make_shared<rclcpp::Node>("servo_sim_plugin")) {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    joint_ = model_->GetJoint(_sdf->Get<std::string>("joint_name")); 
    if (!joint_) 
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found!", _sdf->Get<std::string>("joint_name").c_str());
      return;
    }

    // Abonnement au topic "servo_angle"
    subscription_ = node_->create_subscription<std_msgs::msg::Int16>(
      "/servo_angle", 10, std::bind(&ServoSimPlugin::OnServoAngleMsg, this, std::placeholders::_1));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ServoSimPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    joint_->SetPosition(0, target_angle_);
  }

  void OnServoAngleMsg(const std_msgs::msg::Int16::SharedPtr msg)
  {
    target_angle_ = msg->data * 3.14159265 / 180.0; // Convertir en radians
  }

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  double target_angle_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
  event::ConnectionPtr update_connection_;
  rclcpp::Node::SharedPtr node_;
};

GZ_REGISTER_MODEL_PLUGIN(ServoSimPlugin)
} // namespace gazebo