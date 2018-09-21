#include <pal_wbc_utils/pal_wbc_utils.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <pal_ros_utils/ParamUtils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_pose_task");
  ros::NodeHandle nh("~");

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker_reflexx_typeII");

  boost::optional<geometry_msgs::PointStamped> target_position;
  boost::optional<geometry_msgs::QuaternionStamped> target_orientation;

  if(reference_type == "pointer_reflexx_typeII" || reference_type == "pointer")
  {
    std::string reference_frame;
    nh.param<std::string>("reference_frame", reference_frame, "base_link");
    geometry_msgs::PointStamped point;
    point.point = pal::getParamPoint(nh, "target_position");
    point.header.frame_id = reference_frame;
    geometry_msgs::QuaternionStamped quaternion;
    quaternion.quaternion = pal::getParamQuaternion(nh, "target_orientation");
    quaternion.header.frame_id = reference_frame;
    target_position = point;
    target_orientation = quaternion;
  }

  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_tool_link");

  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  std::string force_torque;
  nh.param<std::string>("admitance_ft", force_torque, "");

  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  double damping;
  nh.param<double>("damping", damping, 0.2);

  double position_gain;
  nh.param<double>("position_gain", position_gain, 1.0);

  double orientation_gain;
  nh.param<double>("orientation_gain", orientation_gain, 1.0);

  pal::WBCServiceHelper srv_helper(nh, ns);

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id", std::string("position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", damping);
  task.addProperty("p_pos_gain", position_gain);
  task.addProperty("p_orient_gain", orientation_gain);

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
  if(target_position && target_orientation)
  {
    task.addProperty("target_position", target_position.get());
    task.addProperty("target_orientation", target_orientation.get());
  }

  if (!srv_helper.pushTask(task, std::string("position_" + tip_name), order, previous_task_id))
  {
    ROS_ERROR_STREAM("problem pushing position task");
  }

  task.updateProperty("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
  task.updateProperty("task_id", std::string("orientation_" + tip_name));

  if (!force_torque.empty())
  {
    task.removeProperty("taskType");
    task.addProperty("taskType",
                     std::string("pal_wbc/GoToLocalVirtualAdmitanceOrientationMetaTask"));
  }

  if (!srv_helper.pushTask(task, std::string("orientation_" + tip_name), order,
                           std::string("position_" + tip_name)))
  {
    ROS_ERROR_STREAM("problem pushing orientation task");
  }

  return (0);
}
