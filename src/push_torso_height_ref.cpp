#include <pal_wbc_utils/pal_wbc_utils.h>
#include <pal_ros_utils/reference/pose/pose_pointer_reference.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_torso_height_ref");
  ros::NodeHandle nh("~");

  std::string link_name;
  nh.param<std::string>("link_name", link_name, "torso_1_link");

  std::string base_frame;
  nh.param<std::string>("base_frame", base_frame, "odom");

  double reference_height;
  nh.param<double>("reference_height", reference_height, 0.0);

  std::string previous_task_id;
  nh.param<std::string>("before_task_id", previous_task_id, "default_reference");

  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  pal::WBCServiceHelper srv_helper(nh, ns);

  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::Before;

  property_bag::PropertyBag properties;
  std::string task_id = link_name + "_reference_height";
  properties.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  properties.addProperty("task_id", task_id);
  properties.addProperty("tip_name", link_name);
  properties.addProperty("damping", 1.0);

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

  srv_helper.pushTask(properties, "task_id", order, previous_task_id);

  return (0);
}
