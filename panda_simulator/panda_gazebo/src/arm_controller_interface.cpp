/***************************************************************************
* Adapted from arm_controller_interface.cpp (sawyer_simulator package)

*
* @package: panda_gazebo
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2021, Saif Sidhik
* Copyright (c) 2013-2018, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include <panda_gazebo/arm_controller_interface.h>
#include <controller_manager_msgs/SwitchController.h>

namespace panda_gazebo {

void ArmControllerInterface::init(ros::NodeHandle& nh,
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager) {
    current_mode_ = 5;

  if (!nh.getParam("/controllers_config/position_controller", position_controller_name_)) {
        position_controller_name_ = "position_joint_position_controller";
    }
  if (!nh.getParam("/controllers_config/torque_controller", torque_controller_name_)) {
        torque_controller_name_ = "effort_joint_torque_controller";
    }
  if (!nh.getParam("/controllers_config/velocity_controller", velocity_controller_name_)) {
        velocity_controller_name_ = "velocity_joint_velocity_controller";
    }
  if (!nh.getParam("/controllers_config/default_controller", default_controller_name_)) {
        default_controller_name_ = "position_joint_trajectory_controller";
    }

  current_controller_name_ = default_controller_name_;

  all_controllers_.clear();
  all_controllers_.push_back(position_controller_name_);
  all_controllers_.push_back(torque_controller_name_);
  all_controllers_.push_back(velocity_controller_name_);

  bool default_defined = false;

  for (size_t i = 0; i < all_controllers_.size(); ++i){
    if (all_controllers_[i] == default_controller_name_){
      default_defined = true;
      break;
    }
  }

  controller_name_to_mode_map_[position_controller_name_] = franka_core_msgs::JointCommand::POSITION_MODE;
  controller_name_to_mode_map_[torque_controller_name_] = franka_core_msgs::JointCommand::TORQUE_MODE;
  controller_name_to_mode_map_[velocity_controller_name_] = franka_core_msgs::JointCommand::VELOCITY_MODE;

  if (! default_defined){
    ROS_ERROR_STREAM_NAMED("ArmControllerInterface", "Default controller not present in the provided controllers!");
  }
  
  controller_manager_ = controller_manager;
  joint_command_sub_ = nh.subscribe("/panda_simulator/motion_controller/arm/joint_commands", 1,
                       &ArmControllerInterface::jointCommandCallback, this);

  // Command Timeout
  joint_command_timeout_sub_ = nh.subscribe("/panda_simulator/motion_controller/arm/joint_command_timeout", 1,
                       &ArmControllerInterface::jointCommandTimeoutCallback, this);
  double command_timeout_default;
  nh.param<double>("command_timeout", command_timeout_default, 0.2);
  auto p_cmd_timeout_length = std::make_shared<ros::Duration>(std::min(1.0,
                                std::max(0.0, command_timeout_default)));
  box_timeout_length_.set(p_cmd_timeout_length);

  // Update at 100Hz
  cmd_timeout_timer_ = nh.createTimer(100, &ArmControllerInterface::commandTimeoutCheck, this);
}

void ArmControllerInterface::commandTimeoutCheck(const ros::TimerEvent& e) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);
  // Check Command Timeout
  std::shared_ptr<const ros::Duration>  p_timeout_length;
  box_timeout_length_.get(p_timeout_length);
  std::shared_ptr<const ros::Time>  p_cmd_msg_time;
  box_cmd_timeout_.get(p_cmd_msg_time);
  bool command_timeout = (p_cmd_msg_time && p_timeout_length &&
      ((ros::Time::now() - *p_cmd_msg_time.get()) > (*p_timeout_length.get())));
  if(command_timeout && (current_controller_name_ != default_controller_name_)) {
    // Timeout violated, force robot back to Default Controller Mode

    ROS_WARN_STREAM("ArmControllerInterface: Command timeout violated: Switching to Default control mode. " << default_controller_name_);
    switchToDefaultController();
  }
}

bool ArmControllerInterface::switchToDefaultController() {

  std::vector<std::string> start_controllers; 
  std::vector<std::string> stop_controllers;

  for (size_t i = 0; i < all_controllers_.size(); ++i) {

    if (all_controllers_[i] == default_controller_name_)
      start_controllers.push_back(all_controllers_[i]);
    else stop_controllers.push_back(all_controllers_[i]);
  }
  if (!controller_manager_->switchController(start_controllers, stop_controllers,
                              controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("ArmControllerInterface", "Failed to switch controllers");
      return false;
    }
  current_controller_name_ = start_controllers[0];
  current_mode_ = controller_name_to_mode_map_[current_controller_name_];
  ROS_INFO_STREAM("ArmControllerInterface: Controller " << start_controllers[0]
                          << " started; Controllers " << stop_controllers[0] <<
                          ", " << stop_controllers[1] << " stopped.");
  return true;
}

void ArmControllerInterface::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "ArmControllerInterface", "Joint command timeout: " << msg.data);
  auto p_cmd_timeout_length = std::make_shared<ros::Duration>(
                                std::min(1.0, std::max(0.0, double(msg.data))));
  box_timeout_length_.set(p_cmd_timeout_length);
}

bool ArmControllerInterface::switchControllers(int control_mode) {
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  if(current_mode_ != control_mode)
  {
    switch (control_mode)
    {
      case franka_core_msgs::JointCommand::POSITION_MODE:
        start_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::IMPEDANCE_MODE:
        start_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::TORQUE_MODE:
        start_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::VELOCITY_MODE:
        start_controllers.push_back(velocity_controller_name_);
        stop_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        break;      
      default:
        ROS_ERROR_STREAM_NAMED("ArmControllerInterface", "Unknown JointCommand mode "
                                << control_mode << ". Ignoring command.");
        return false;
    }
    if (!controller_manager_->switchController(start_controllers, stop_controllers,
                              controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("ArmControllerInterface", "Failed to switch controllers");
      return false;
    }
    current_mode_ = control_mode;
    current_controller_name_ = start_controllers[0];
    ROS_INFO_STREAM("ArmControllerInterface: Controller " << start_controllers[0]
                            << " started; Controllers " << stop_controllers[0] <<
                            ", " << stop_controllers[1] << " stopped.");
  }
  return true;
}

void ArmControllerInterface::jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);
  if(switchControllers(msg->mode)) {
    auto p_cmd_msg_time = std::make_shared<ros::Time>(ros::Time::now());
    box_cmd_timeout_.set(p_cmd_msg_time);
  }
}

}
