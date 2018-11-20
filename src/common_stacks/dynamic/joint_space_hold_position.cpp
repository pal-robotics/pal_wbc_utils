#include <Eigen/Dense>
#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_dynamic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/constraint_dynamic_task.h>
#include <wbc_tasks/go_to_dynamic_task.h>
#include <wbc_tasks/torque_damping_dynamic_task.h>
#include <wbc_tasks/physics_constraint_dynamic_task.h>
#include <wbc_tasks/com_dynamic_task.h>
#include <wbc_tasks/reference_dynamic_posture.h>
#include <wbc_tasks/torque_limit_dynamic_task.h>
#include <wbc_tasks/unilateral_forces_dynamic.h>
#include <wbc_tasks/joint_pos_limit_dynamic_task.h>
#include <wbc_tasks/dynamic_task_joint_pos_vel_acc_limits.h>
#include <pluginlib/class_list_macros.h>

#include <wbc_tasks/physics_tools.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>

using namespace pal_wbc;

namespace common_stacks
{
namespace dynamic
{
class joint_space_hold_position : public StackConfigurationDynamic
{
  void setupStack(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh)
  {
    task_container_vector constraints;
    setUpPhysics(stack, nh, constraints);


    // Joint limit task
    {
      pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param;

      param.joint_names_ = stack->getJointNames();

      pal::math_utils::copyVector(stack->getJointPositionLimitMin(), param.position_min_);
      pal::math_utils::copyVector(stack->getJointPositionLimitMax(), param.position_max_);
      param.position_time_parameter_ = 0.05;
      param.position_enable_recovery_ = true;


      pal::math_utils::copyVector(stack->getJointVelocityLimitMin(), param.velocity_min_);
      pal::math_utils::copyVector(stack->getJointVelocityLimitMax(), param.velocity_max_);
      param.velocity_time_parameter_ = 0.001;
      param.velocity_enable_recovery_ = false;
      param.velocity_clamp_ = true;

      param.acceleration_absmax_.setConstant(param.joint_names_.size(), 600);

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


    GenericMetaTaskPtr constraintMetatask(
        new GenericMetaTask(nh, stack.get(), constraints, "constraints"));
    stack->pushTask(constraintMetatask);



    std::vector<std::string> joint_names = stack->getJointNames();
    Eigen::VectorXd reference_posture;
    pal::rbcomposite::URDFModel::getDefaultConfiguration(nh, "/zeros", joint_names,
                                                         reference_posture);

    // vector_dynamic_reconfigure
    ReferenceDynamicPostureTaskMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskMetaTask(
        "reference_joint", stack.get(), joint_names,
        "vector_min_jerk_dynamic_reconfigure", reference_posture, 200, nh));
    reference_task->setWeight(1);

    TorqueDampingDynamicTaskAllJointsMetaTaskPtr joint_torque_regularization(
        new TorqueDampingDynamicTaskAllJointsMetaTask("torque_regularization", stack.get(),
                                                      stack->getJointNames(), nh));
    joint_torque_regularization->setWeight(1e-4);

    task_container_vector objectiveTasks;
    objectiveTasks.push_back(reference_task);
    objectiveTasks.push_back(joint_torque_regularization);


    GenericMetaTaskPtr objectiveMetatask(
        new GenericMetaTask(nh, stack.get(), objectiveTasks, "objectives"));
    stack->pushTask(objectiveMetatask);
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(common_stacks::dynamic::joint_space_hold_position,
                       pal_wbc::StackConfigurationDynamic);
