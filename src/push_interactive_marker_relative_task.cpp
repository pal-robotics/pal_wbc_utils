/*
  @file

  @author adriaroig

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/

#include <pal_wbc_utils/pal_wbc_utils.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <pal_ros_utils/ParamUtils.h>

/*
 * Node that pushes a relative task to constrain a link (slave)
 * respect anoteher one (master)
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_relative_task");
  ros::NodeHandle nh("~");

  /*
    Reference type used to control the master link:
    - interactive_marker_reflexx_typeII,
    - topic_reflexx_typeII,
    - pointer_reflexx_typeII
    - interactive_marker,
    - topic
    - pointer

    By default use reflexx_typeII which are time optimal and continous
  */
  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker_reflexx_typeII");

  // Name of the master link
  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_left_tool_link");

  // Name of the constrained / slave link
  std::string constrained_link;
  nh.param<std::string>("constrained_link", constrained_link, "arm_right_tool_link");

  // Task which respect the pose task is pushed
  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  /*
    Name of the FT. If defined uses admittance control.
    The tip name has to coincide with the FT name.
    Example: tip_name:= wrist_ft_link / admitance_ft:= wrist_ft
    Otherwise the admittance wouldn't be applied correctly
  */
  std::string force_torque;
  nh.param<std::string>("admitance_ft", force_torque, "");

  // Namespace of the controller
  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  // Creates a service helper that comunicates with all the WBC services
  pal::WBCServiceHelper srv_helper(nh, ns);

  // Set the position and orientation to mantain of the slave link respect the master link
  geometry_msgs::PointStamped target_position;
  geometry_msgs::QuaternionStamped target_orientation;

  target_position.point = pal::getParamPoint(nh, "target_position");
  target_orientation.quaternion = pal::getParamQuaternion(nh, "target_orientation");

  // Order respect the previous task id
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  // Set all he properties to control the master link in a property bag
  // to push a pose task online
  property_bag::PropertyBag task;
  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id", std::string("position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", 0.2);

  // If force torque is defined, add the specific properties for the admittance task
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

  // Push the position task in the stack
  srv_helper.pushTask(task, std::string("position_" + tip_name), order, previous_task_id);

  // Add the orientation properties
  task.updateProperty("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
  task.updateProperty("task_id", std::string("orientation_" + tip_name));

  if (!force_torque.empty())
  {
    task.removeProperty("taskType");
    task.addProperty("taskType",
                     std::string("pal_wbc/GoToLocalVirtualAdmitanceOrientationMetaTask"));
  }

  // Push the orientation task
  srv_helper.pushTask(task, std::string("orientation_" + tip_name), order,
                      std::string("position_" + tip_name));

  // Add / Update the properties to push the relative pose task
  task.updateProperty("taskType", std::string("pal_wbc/GoToRelativePositionMetaTask"));
  task.updateProperty("task_id", std::string("relative_position_" + constrained_link));
  task.updateProperty("signal_reference", std::string("pointer"));
  task.updateProperty("tip_name", constrained_link);
  task.addProperty("reference_link", tip_name);
  task.addProperty("target_position", target_position);

  order = pal_wbc_msgs::Order::After;

  // Push the relative position task
  srv_helper.pushTask(task, std::string("relative_position_" + constrained_link), order, std::string("position_" + tip_name));

  task.updateProperty("taskType", std::string("pal_wbc/GoToRelativeOrientationMetaTask"));
  task.updateProperty("task_id", std::string("relative_orientation_" + constrained_link));
  task.addProperty("target_orientation", target_orientation);

  // Push the relative orientation task
  srv_helper.pushTask(task, std::string("relative_orientation_" + constrained_link), order, std::string("relative_position_" + constrained_link));

  return (0);
}
