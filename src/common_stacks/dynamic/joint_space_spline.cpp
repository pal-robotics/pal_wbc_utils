#include <Eigen/Dense>
#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_dynamic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/constraint_dynamic_task.h>
#include <wbc_tasks/go_to_dynamic_task.h>
#include <wbc_tasks/torque_damping_dynamic_task.h>
#include <wbc_tasks/physics_constraint_dynamic_task.h>
#include <wbc_tasks/com_dynamic_task.h>
#include <wbc_tasks/dynamic_task_joint_reference.h>
#include <wbc_tasks/torque_limit_dynamic_task.h>
#include <wbc_tasks/unilateral_forces_dynamic.h>
#include <wbc_tasks/joint_pos_limit_dynamic_task.h>
#include <pluginlib/class_list_macros.h>

#include <pal_ros_utils/reference/vector/joint_trajectory_action.h>

#include <wbc_tasks/physics_tools.h>

using namespace pal_wbc;

namespace common_stacks
{
namespace dynamic
{
class joint_space_spline : public StackConfigurationDynamic
{
  void setupStack(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh)
  {
    task_container_vector constraintTasks;
    setUpPhysics(stack, nh, constraintTasks);

    /*
        // Modify the joint limits
        std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
        std::vector<double> joint_pos_max_override = stack->getJointPositionLimitMax();

        //Joint limit task
        JointPositionLimitDynamicAllJointsMetaTaskPtr joint_position_limit_task(
              new JointPositionLimitDynamicAllJointsMetaTask(*stack.get(),
                                                             joint_pos_min_override,
                                                             joint_pos_max_override,
                                                             stack->getJointVelocityLimitMin(),
                                                             stack->getJointVelocityLimitMax(),
                                                             stack->getJointNames(),
                                                             1.0, true, true, nh));

        constraintTasks.push_back(joint_position_limit_task);
    */
    TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits_task(new TorqueLimitDynamicAllJointsMetaTask(
        "torque_limits", *stack.get(), stack->getJointTorqueLimits(), stack->getJointNames(), nh));
    constraintTasks.push_back(torque_limits_task);


    GenericMetaTaskPtr constraintMetatask(
        new GenericMetaTask(nh, stack.get(), constraintTasks, "constraints"));
    stack->pushTask(constraintMetatask);


    task_container_vector objectiveTasks;


    pal_robot_tools::VectorJointTrajectoryActionSplineReferencePtr reference =
        boost::make_shared<pal_robot_tools::VectorJointTrajectoryActionSplineReference>(
            nh, "name", stack->getJointNames());

    JointReferenceDynamicPtr reference_task(
        new JointReferenceDynamic("reference_joint", nh, *stack.get(), stack->getJointNames(), reference, 200));
    reference_task->setWeight(1);
    objectiveTasks.push_back(reference_task);

    TorqueDampingDynamicTaskAllJointsMetaTaskPtr joint_torque_regularization(
        new TorqueDampingDynamicTaskAllJointsMetaTask("torque_regularization", *stack.get(), stack->getJointNames(), nh));
    joint_torque_regularization->setWeight(1e-4);

    objectiveTasks.push_back(joint_torque_regularization);


    GenericMetaTaskPtr objectiveMetatask(
        new GenericMetaTask(nh, stack.get(), objectiveTasks, "objectives"));
    stack->pushTask(objectiveMetatask);
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(common_stacks::dynamic::joint_space_spline,
                       pal_wbc::StackConfigurationDynamic);
