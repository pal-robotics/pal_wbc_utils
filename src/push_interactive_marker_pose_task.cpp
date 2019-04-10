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
 *  Node that pushes a Position + Orientation task into the stack
 *  and allows to control the robot in cartesian space
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_pose_task");
  ros::NodeHandle nh("~");

  /*
    Reference type:
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

  boost::optional<geometry_msgs::PointStamped> target_position;
  boost::optional<geometry_msgs::QuaternionStamped> target_orientation;

  // If the reference type is pointer, the pose is fixed and needs to be specified
  if (reference_type == "pointer_reflexx_typeII" || reference_type == "pointer")
  {
    // Reads the reference frame in which the pose is expressed (has to be a frame in the
    // robot)
    std::string reference_frame;
    nh.param<std::string>("reference_frame", reference_frame, "base_link");
    //  Sets the target position
    geometry_msgs::PointStamped point;
    point.point = pal::getParamPoint(nh, "target_position");
    point.header.frame_id = reference_frame;
    // Sets the target orientation
    geometry_msgs::QuaternionStamped quaternion;
    quaternion.quaternion = pal::getParamQuaternion(nh, "target_orientation");
    quaternion.header.frame_id = reference_frame;
    target_position = point;
    target_orientation = quaternion;
  }

  // Name of the frame to control in cartesian space
  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_tool_link");

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

  // Damping
  double damping;
  nh.param<double>("damping", damping, 0.2);

  // Position gain to reduce the position error in the task
  // Higher gains imply higher velocities
  double position_gain;
  nh.param<double>("position_gain", position_gain, 1.0);

  // Orientation gain to reduce the orientation error in the taks.
  // Higher gains implies higher velocities.
  double orientation_gain;
  nh.param<double>("orientation_gain", orientation_gain, 1.0);

  // Creates a service helper that comunicates with all the WBC services
  pal::WBCServiceHelper srv_helper(nh, ns);

  // Order respect the previous task id
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  // Set all he properties in a property bag to push the task online
  property_bag::PropertyBag task;
  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id", std::string("position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", damping);
  task.addProperty("p_pos_gain", position_gain);
  task.addProperty("p_orient_gain", orientation_gain);

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
  // If target position and target orientation are defined add them in the properties
  if (target_position && target_orientation)
  {
    task.addProperty("target_position", target_position.get());
    task.addProperty("target_orientation", target_orientation.get());
  }

  // Push the position task in the stack
  if (!srv_helper.pushTask(task, std::string("position_" + tip_name), order, previous_task_id))
  {
    ROS_ERROR_STREAM("There was a problem pushing the position task");
  }

  // Sets the properties for the orientation task
  task.updateProperty("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
  task.updateProperty("task_id", std::string("orientation_" + tip_name));

  if (!force_torque.empty())
  {
    task.removeProperty("taskType");
    task.addProperty("taskType",
                     std::string("pal_wbc/GoToLocalVirtualAdmitanceOrientationMetaTask"));
  }

  // Pushes the orientation task after the position task (lower priority)
  if (!srv_helper.pushTask(task, std::string("orientation_" + tip_name), order,
                           std::string("position_" + tip_name)))
  {
    ROS_ERROR_STREAM("There was a problem pushing the orientation task");
  }

  return (0);
}
