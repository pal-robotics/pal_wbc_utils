/*
  @file

  @author adriaroig

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/

#include <pal_wbc_utils/pal_wbc_utils.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Eigen>
#include <pal_ros_utils/conversions.h>

/*
 * Node that pushed a reference taks in the stack
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_reference_task");
  ros::NodeHandle nh("~");

  // Namespace of the controller
  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");

  // Reference type: vector_topic, vector_pointer
  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "vector_topic");

  // Name of the topic where the reference could be modified
  std::string topic_name;
  nh.param<std::string>("topic_name", topic_name, ns + "/reference_ref");

  // Position gain to reduce the position error in the task
  // Higher gains imply higher velocities
  double position_gain;
  nh.param<double>("position_gain", position_gain, 3.0);

  // Joint names controlled by the reference
  std::vector<std::string> joint_names;
  if (!nh.getParam("joint_names", joint_names))
  {
    /*
      If no joint names specified in the parameter server...
      Reads from joint states, and uses all of the head, arm and torso
    */
    sensor_msgs::JointStateConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      if (msg->name[i].find("head") != std::string::npos)
        joint_names.push_back(msg->name[i]);
      else if (msg->name[i].find("torso") != std::string::npos)
        joint_names.push_back(msg->name[i]);
      else if (msg->name[i].find("arm") != std::string::npos)
        joint_names.push_back(msg->name[i]);
    }
  }

  /*
    Reference configuration.
    If a reference is specified, and it differs from the default one,
    when replacing this task by the default one there will be a discontinuity
    in which the robot will "jump" to the specified configuration
  */
  std::vector<double> reference;
  if (!nh.getParam("reference", reference))
  {
    // If no reference specified. Uses the default one in the WBC.
    reference.resize(joint_names.size());
    std::map<std::string, double> default_reference;
    if (nh.getParam(ns + "/default_configuration", default_reference))
    {
      for (size_t i = 0; i < joint_names.size(); i++)
      {
        if (default_reference.find(joint_names[i]) != default_reference.end())
        {
          reference[i] = default_reference[joint_names[i]];
        }
        else // If reference not found pushes 0.0
        {
          ROS_WARN_STREAM("Reference for joint " << joint_names[i]
                                                 << " not found in default reference");
          reference[i] = 0.0;
        }
      }
    }
  }

  // Assert that the reference size coincide with the joint names size
  if (reference.size() != joint_names.size())
  {
    ROS_ERROR("Joint names size and reference size should coincide");
    return (-1);
  }

  // Convert the reference to eigen type
  Eigen::VectorXd reference_eigen(reference.size());
  pal::convert(reference, reference_eigen);

  // Task which respect the reference task is pushed
  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "default_reference");

  // Creates a service helper that comunicates with all the WBC services
  pal::WBCServiceHelper srv_helper(nh, ns);

  // Order respect the previous task id
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::Replace;

  // Set all he properties in a property bag to push the task online
  property_bag::PropertyBag reference_task;
  reference_task.addProperty("taskType", std::string("pal_wbc/ReferenceKinematicTask"));
  reference_task.addProperty("task_id", std::string("topic_reference"));
  reference_task.addProperty("signal_reference", reference_type);
  reference_task.addProperty("topic_name", topic_name);
  reference_task.addProperty("joint_names", joint_names);
  reference_task.addProperty("reference", reference_eigen);
  reference_task.addProperty("p_pos_gain", position_gain);
  reference_task.addProperty("damping", double(0.2));

  /*
    Push the reference task in the stack
    This task has to replace any other reference task
  */
  if (!srv_helper.pushTask(reference_task, "topic_reference", order, previous_task_id))
  {
    ROS_ERROR_STREAM("There was a problem pushing the reference task");
  }

  return (0);
}
