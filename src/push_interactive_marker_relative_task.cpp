#include <pal_wbc_utils/pal_wbc_utils.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <pal_ros_utils/ParamUtils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_relative_task");
  ros::NodeHandle nh("~");

  pal::WBCServiceHelper srv_helper(nh);

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker_reflexx_typeII");

  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_left_tool_link");

  std::string constrained_link;
  nh.param<std::string>("constrained_link", constrained_link, "arm_right_tool_link");

  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  std::string force_torque;
  nh.param<std::string>("admitance_ft", force_torque, "");

  geometry_msgs::PointStamped target_position;
  geometry_msgs::QuaternionStamped target_orientation;

  target_position.point = pal::getParamPoint(nh, "target_position");
  target_orientation.quaternion = pal::getParamQuaternion(nh, "target_orientation");

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
    task.addProperty("damper", double(60.));
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

  task.updateProperty("taskType", std::string("pal_wbc/GoToRelativePositionMetaTask"));
  task.updateProperty("task_id", std::string("relative_position_" + constrained_link));
  task.updateProperty("signal_reference", std::string("pointer"));
  task.updateProperty("tip_name", constrained_link);
  task.addProperty("reference_link", tip_name);
  task.addProperty("target_position", target_position);

  order = pal_wbc_msgs::Order::After;

  srv_helper.pushTask(task, std::string("relative_position_" + constrained_link), order, std::string("position_" + tip_name));

  task.updateProperty("taskType", std::string("pal_wbc/GoToRelativeOrientationMetaTask"));
  task.updateProperty("task_id", std::string("relative_orientation_" + constrained_link));
  task.addProperty("target_orientation", target_orientation);

  srv_helper.pushTask(task, std::string("relative_orientation_" + constrained_link), order, std::string("relative_position_" + constrained_link));

  return (0);
}
