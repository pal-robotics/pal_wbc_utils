/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_dynamic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/constraint_dynamic_task.h>
#include <wbc_tasks/torque_limit_dynamic_task.h>
#include <wbc_tasks/joint_pos_limit_dynamic_task.h>
#include <wbc_tasks/dynamic_task_joint_pos_vel_acc_limits.h>

#include <wbc_tasks/physics_tools.h>


using namespace pal_wbc;

namespace common_stacks
{
namespace dynamic
{
class JointConstraints
{
protected:
  void setupJointConstraints(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh,
                             task_container_vector &constraints) const
  {
    // Joint limit task
    {
      pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param;
      if (nh.hasParam("joint_position_limits_parameters"))
      {
        param.readConfig<ariles::ros>(nh, "joint_position_limits_parameters");
      }
      else
      {
        param.position_time_parameter_ = 0.05;
        param.position_enable_recovery_ = true;


        param.velocity_time_parameter_ = 0.001;
        param.velocity_enable_recovery_ = false;
        param.velocity_clamp_ = true;

        param.acceleration_absmax_.setConstant(param.joint_names_.size(), 600);
      }

      if (0 == param.joint_names_.size())
      {
        param.joint_names_ = stack->getJointNames();

        pal::math_utils::copyVector(stack->getJointPositionLimitMin(), param.position_min_);
        pal::math_utils::copyVector(stack->getJointPositionLimitMax(), param.position_max_);

        pal::math_utils::copyVector(stack->getJointVelocityLimitMin(), param.velocity_min_);
        pal::math_utils::copyVector(stack->getJointVelocityLimitMax(), param.velocity_max_);
      }


      pal_wbc::JointPosVelAccLimitsDynamicTaskPtr joint_position_limit_task =
          boost::make_shared<pal_wbc::JointPosVelAccLimitsDynamicTask>(
              "joint_position_limit_task", param, stack.get(), nh);

      constraints.push_back(joint_position_limit_task);
    }


    // Torque limits
    {
      TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits_task =
          boost::make_shared<TorqueLimitDynamicAllJointsMetaTask>(
              "torque_limits", stack.get(), stack->getJointTorqueLimits(),
              stack->getJointNames(), nh);
      constraints.push_back(torque_limits_task);
    }
  }
};
}
}
