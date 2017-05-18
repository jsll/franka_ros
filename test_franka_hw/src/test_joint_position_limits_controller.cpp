#include <test_franka_hw/test_joint_position_limits_controller.h>

#include <random>

#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace test_franka_hw {

TestJointPositionLimitsController::TestJointPositionLimitsController()
    : uniform_distribution_(0.0, 3.0), random_engine_() {}

bool TestJointPositionLimitsController::init(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& node_handle) {
  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
    ROS_ERROR(
        "Could not initialize urdf_model parsing robot_description in "
        "JointLimitTestController");
  } else {
    ROS_INFO("Succesfully initialized urdf model");
  }

  XmlRpc::XmlRpcValue params;
  if (!node_handle.getParam("/mock_franka_hw_node/joint_names", params)) {
    ROS_ERROR("Could not parse joint names in JointLimitTestController");
  } else {
    ROS_INFO("Succesfully parsed joint names");
  }
  joint_names_.resize(params.size());
  joint_limits_.resize(params.size());
  for (size_t i = 0; i < params.size(); ++i) {
    joint_names_[i] = static_cast<std::string>(params[i]);
    boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_names_[i]);
    if (!joint_limits_interface::getJointLimits(urdf_joint, joint_limits_[i])) {
      ROS_ERROR_STREAM("Could not parse joint limits of joint "
                       << joint_names_[i]);
      return false;
    }
    ROS_INFO_STREAM("Got Joint with limits: effort="
                    << urdf_joint->limits->effort
                    << " upper=" << urdf_joint->limits->upper
                    << " lower=" << urdf_joint->limits->lower << " velocity"
                    << urdf_joint->limits->velocity);
  }
  position_interface_ =                                             // NOLINT
      robot_hw->get<hardware_interface::PositionJointInterface>();  // NOLINT
  if (position_interface_ == nullptr) {
    ROS_ERROR("interfaces for controller not properly initialized ");
    return false;
  }

  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i] =
        position_interface_->getHandle(joint_names_[i]);
  }
  return true;
}

void TestJointPositionLimitsController::update(
    const ros::Time& time,          // NOLINT
    const ros::Duration& period) {  // NOLINT
  std::array<double, 7> position_command;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (phase_) {
      position_command[i] =
          joint_limits_[i].max_position + uniform_distribution_(random_engine_);
    } else {
      position_command[i] =
          joint_limits_[i].min_position - uniform_distribution_(random_engine_);
    }
    position_joint_handles_[i].setCommand(position_command[i]);
  }
  phase_ = !phase_;
  ROS_INFO_STREAM("controller: Sent commands position =  \n"
                  << position_command[0] << " " << position_command[1] << " "
                  << position_command[2] << " " << position_command[3] << " "
                  << position_command[4] << " " << position_command[5] << " "
                  << position_command[6] << " ;\n ");
}

}  // namespace test_franka_hw

PLUGINLIB_EXPORT_CLASS(test_franka_hw::TestJointPositionLimitsController,
                       controller_interface::ControllerBase)
