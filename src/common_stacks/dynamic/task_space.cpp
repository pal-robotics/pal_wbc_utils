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
#include <wbc_tasks/friction_constraint_task.h>
#include <wbc_tasks/dynamic_task_joint_pos_vel_acc_limits.h>
#include <pluginlib/class_list_macros.h>

#include <wbc_tasks/physics_tools.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>

namespace
{
/// @todo AS: add interactive goals
class TaskSpaceGoalTags : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TaskSpaceGoalTags"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(topic_goals, pal::rbcomposite::TagPointSet)                        \
  ARILES_TYPED_ENTRY_(interactive_goals, pal::rbcomposite::TagPointSet)
#include ARILES_INITIALIZE

public:
  TaskSpaceGoalTags()
  {
    setDefaults();
  }

  void setDefaults()
  {
    topic_goals_.setDefaults();
    interactive_goals_.setDefaults();
  }
};
}


using namespace pal_wbc;

namespace common_stacks
{
namespace dynamic
{
class task_space : public StackConfigurationDynamic
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

      for (std::size_t i = 0; i < param.position_min_.size(); ++i)
      {
        std::cout << param.joint_names_[i] << " -- " << param.position_min_[i] << "   "
                  << param.position_max_[i] << std::endl;
      }


      pal::math_utils::copyVector(stack->getJointVelocityLimitMin(), param.velocity_min_);
      pal::math_utils::copyVector(stack->getJointVelocityLimitMax(), param.velocity_max_);
      param.velocity_time_parameter_ = 0.000;
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



    std::vector<pal_wbc::ContactDescription> contacts = stack->getContactForceDescriptions();
    for (const pal_wbc::ContactDescription &contact : contacts)
    {
      double mu = 0.0;
      FrictionConstratintDynamicMetaTaskPtr friction_constraint(new FrictionConstratintDynamicMetaTask(
          "contact_friction", stack.get(), mu, contact.first, nh));
      constraints.push_back(friction_constraint);
    }

    GenericMetaTaskPtr constraintMetatask(
        new GenericMetaTask(nh, stack.get(), constraints, "constraints"));
    stack->pushTask(constraintMetatask);



    task_container_vector objectiveTasks;

    TaskSpaceGoalTags goals;
    goals.readConfig<ariles::ros>(nh, "task_space_goals");

    PAL_ASSERT_PERSIST(goals.topic_goals_.tags_.size() + goals.interactive_goals_.tags_.size() > 0,
                       "At least one topic goal must be specified.");

    const double p_gain = 300;
    const double d_gain = 0;
    bool critically_damped = true;

    /// @todo AS: it is unsafe to use unsynced link ids.
    for (const auto &tag : goals.topic_goals_.tags_)
    {
      switch (tag.type_)
      {
        case pal::rbcomposite::TagLink::Type::POSITION:
        {
          GoToPositionDynamicMetaTaskPtr go_to_position(new GoToPositionDynamicMetaTask(
              std::string("go_to_position_") + tag.link_id_.name_, stack.get(),
              tag.link_id_.name_, "ref_pose_minjerk_topic", tag.local_position_, nh,
              p_gain, d_gain, critically_damped));
          objectiveTasks.push_back(go_to_position);
        }
        break;

        default:
          PAL_THROW("Topic tag type not supported");
      }
    }
    for (const auto &tag : goals.interactive_goals_.tags_)
    {
      switch (tag.type_)
      {
        case pal::rbcomposite::TagLink::Type::POSITION:
        {
          GoToPositionDynamicMetaTaskPtr go_to_position(new GoToPositionDynamicMetaTask(
              std::string("go_to_position_interactive_") + tag.link_id_.name_,
              stack.get(), tag.link_id_.name_, "interactive_marker", tag.local_position_,
              nh, p_gain, d_gain, critically_damped));
          objectiveTasks.push_back(go_to_position);
        }
        break;


        case pal::rbcomposite::TagLink::Type::COMPLETE:
        {
          GoToPoseDynamicMetaTaskPtr go_to_pose(new GoToPoseDynamicMetaTask(
              std::string("go_to_pose_interactive_") + tag.link_id_.name_, stack.get(),
              tag.link_id_.name_, tag.local_position_, "interactive_marker", nh, p_gain,
              p_gain, d_gain, d_gain, critically_damped));

          objectiveTasks.push_back(go_to_pose);
        }
        break;

        default:
          PAL_THROW("Interactive tag type not supported");
      }
    }


    std::vector<std::string> joint_names = stack->getJointNames();
    Eigen::VectorXd reference_posture;
    pal::rbcomposite::URDFModel::getDefaultConfiguration(nh, "/zeros", joint_names,
                                                         reference_posture);
    // reference_posture.setZero(stack->getNumberDofJointState());

    ReferenceDynamicPostureTaskMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskMetaTask(
        "reference", stack.get(), joint_names, "vector_dynamic_reconfigure",
        reference_posture, 36, nh));
    reference_task->setWeight(1e-2);

    objectiveTasks.push_back(reference_task);

    GenericMetaTaskPtr objectiveMetatask(
        new GenericMetaTask(nh, stack.get(), objectiveTasks, "objectives"));
    stack->pushTask(objectiveMetatask);
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(common_stacks::dynamic::task_space, pal_wbc::StackConfigurationDynamic);
