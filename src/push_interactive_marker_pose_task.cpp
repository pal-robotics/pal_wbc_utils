#include <pal_wbc_utils/pal_wbc_utils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_pose_task");
  ros::NodeHandle nh("~");

  pal::WBCServiceHelper srv_helper(nh);

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker_reflexx_typeII");

  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_tool_link");

  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  std::string force_torque;
  nh.param<std::string>("admitance_ft", force_torque, "");

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id", std::string("position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", 0.2);

  if (!force_torque.empty())
  {
    task.addProperty("mass", double(1.));
    task.addProperty("spring", double(150.));
    task.addProperty("filter_gain", double(0.1));
    task.addProperty("ft_name", force_torque);
    task.removeProperty("damping");
    task.addProperty("damping", double(60.0));
    task.removeProperty("taskType");
    task.addProperty("taskType",
                     std::string("pal_wbc/GoToLocalVirtualAdmitancePositionMetaTask"));
  }

  srv_helper.pushTask(task, std::string("position_" + tip_name), order, previous_task_id);

  task.updateProperty("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
  task.updateProperty("task_id", std::string("orientation_" + tip_name));

  if (!force_torque.empty())
  {
    task.removeProperty("taskType");
    task.addProperty("taskType",
                     std::string("pal_wbc/GoToLocalVirtualAdmitanceOrientationMetaTask"));
  }

  srv_helper.pushTask(task, std::string("orientation_" + tip_name), order,
                      std::string("position_" + tip_name));

  return (0);
}
