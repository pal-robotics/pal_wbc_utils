/*
  @file

  @author adriaroig

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/

#include <pal_wbc_utils/pal_wbc_utils.h>
#include <pal_ros_utils/reference/pose/pose_pointer_reference.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <geometry_msgs/PointStamped.h>

/*
 *  Node that mantains the torso at a specific height
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_torso_height_ref");
  ros::NodeHandle nh("~");

  // Name of the link to be constrained
  std::string link_name;
  nh.param<std::string>("link_name", link_name, "torso_lift_link");

  // Name of the base frame
  std::string base_frame;
  nh.param<std::string>("base_frame", base_frame, "odom");

  // Reference height
  double reference_height;
  nh.param<double>("reference_height", reference_height, 0.0);

  // Task which respect the pose task is pushed
  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  // Namespace of the controller
  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  // Creates a service helper that comunicates with all the WBC services
  pal::WBCServiceHelper srv_helper(nh, ns);

  // Order respect the previous task id
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  // Set all he properties in a property bag to push the task online
  property_bag::PropertyBag properties;
  std::string task_id = link_name + "_reference_height";
  properties.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  properties.addProperty("task_id", task_id);
  properties.addProperty("tip_name", link_name);
  properties.addProperty("damping", 1.0);

  /*
    Forces the link name frame at a certain height respect the base frame.
    Since the torso frame has the Z coordinate pointing up, constraint the Z coordinate
    at a specific position is equivalent to force it to be a certain height.
  */
  pal_wbc::coord_t c = pal_wbc::coord_t::Z;
  std::vector<int> coordinates = { c._to_integral() };
  properties.addProperty("coordinates", coordinates);
  geometry_msgs::PointStamped target_position;
  target_position.header.stamp = ros::Time::now();
  target_position.header.frame_id = base_frame;
  target_position.point.x = 0.0;
  target_position.point.y = 0.0;
  target_position.point.z = reference_height;
  properties.addProperty("target_position", target_position);
  properties.addProperty("p_pos_gain", 5.0);

  if(!srv_helper.pushTask(properties, "task_id", order, previous_task_id))
  {
    ROS_ERROR_STREAM("There was a problem pushing torso heigh reference task");
  }

  return (0);
}
