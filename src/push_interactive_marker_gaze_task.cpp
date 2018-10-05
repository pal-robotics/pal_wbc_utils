#include <pal_wbc_utils/pal_wbc_utils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_gaze_task");
  ros::NodeHandle nh("~");

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker");

  std::string camera_frame;
  nh.param<std::string>("camera_frame", camera_frame, "xtion_optical_frame");

  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");

  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  pal::WBCServiceHelper srv_helper(nh, ns);

  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  property_bag::PropertyBag gaze_task;
  gaze_task.addProperty("taskType",
                        std::string("pal_wbc/GazePointKinematicMetaTask"));
  gaze_task.addProperty("task_id", std::string("gaze_task"));
  gaze_task.addProperty("reference_type", reference_type);
  gaze_task.addProperty("camera_frame", camera_frame);

  if (!srv_helper.pushTask(gaze_task, "gaze_task", order, previous_task_id))
  {
      ROS_ERROR_STREAM("problem pusing gaze task");
  }

  return (0);
}
