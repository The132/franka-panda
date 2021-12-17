/***************************************************************************
* Adapted from sawyer_robot_hw_sim.h (sawyer_simulator package)

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

#ifndef _PANDA_GAZEBO___PANDA_ROBOT_HW_SIM_H_
#define _PANDA_GAZEBO___PANDA_ROBOT_HW_SIM_H_

// gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <panda_hardware_interface/sum_command_interface.h>
#include <panda_hardware_interface/shared_joint_interface.h>


namespace panda_gazebo
{

class PandaRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

  virtual void brakesActive(const bool active);

protected:
  bool initCustomInterfaces();
  void initBrakes();

  panda_hardware_interface::SharedJointInterface sum_ej_interface_;
  // secondary list of refs to SumJointHandle containers, so we can call updateCommandSum() before write
  std::vector<panda_hardware_interface::SumJointHandlePtr> sum_ej_handles_refs_;

  std::vector<double> joint_enable_;
  std::vector<double> joint_disable_;
  static const double BRAKE_VALUE;
};

typedef std::shared_ptr<PandaRobotHWSim> PandaRobotHWSimPtr;

}  // namespace

#endif // #ifndef __PANDA_GAZEBO_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_
