/*
  @file

  @author adriaroig

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#include <pal_wbc_utils/pal_wbc_utils.h>

/*
 *  Node that pushes a gaze task into the stack
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_gaze_task");
  ros::NodeHandle nh("~");

  // Reference type: interactive_marker, pointer, topic
  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker");

  // Name of the camera frame. Has to be one of the optical frames
  std::string camera_frame;
  nh.param<std::string>("camera_frame", camera_frame, "xtion_optical_frame");

  // Task which respect the gaze task is pushed
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
  property_bag::PropertyBag gaze_task;
  gaze_task.addProperty("taskType",
                        std::string("pal_wbc/GazePointKinematicMetaTask"));
  gaze_task.addProperty("task_id", std::string("gaze_task"));
  gaze_task.addProperty("reference_type", reference_type);
  gaze_task.addProperty("camera_frame", camera_frame);

  // Push the task int the stack
  if (!srv_helper.pushTask(gaze_task, "gaze_task", order, previous_task_id))
  {
      ROS_ERROR_STREAM("There was a problem pushing the gaze task");
  }

  return (0);
}
