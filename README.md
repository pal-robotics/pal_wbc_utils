\mainpage .

# PAL WBC Utils

This package contains all the generic utilities for pal wbc.

1. [ Utilities ](#utilities)
2. [ Push / Remove a task from the stack ](#stack)

@see wbc_tasks
@see pal_ros_utils
@see reflexxes_adapter

<a name="utilities"></a>
## 1. Utilities

### WBC Service helper

One of the main utilities of this package is the [WBCServiceHelper](./classpal_1_1WBCServiceHelper.html) class that allows to:

- push a task in the stack
- remove a task from the stack
- check if a task exists in the stack
- get task error to the target objective
- print the stack description
- wait until a certain task has converged to a specific tolerance

> Note: Remember that only those tasks which are exported as a plugin can be pushed on the stack once the controller already started.

Example of how a task is pushed:
```sh
int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_pose_task");
  ros::NodeHandle nh("~");

  std::string ns;
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");
  pal::WBCServiceHelper srv_helper(nh, ns);

  ...

  property_bag::PropertyBag task;
  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id", std::string("position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", damping);

  ...

  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "self_collision");
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  if (!srv_helper.pushTask(task, std::string("position_" + tip_name), order, previous_task_id))
  {
    ROS_ERROR_STREAM("problem pushing position task");
  }

  ...

```

### Useful nodes

This package also contains a set of ROS nodes that push WBC tasks in the stacks.

#### Push Gaze task

The push_interactive_marker_gaze_task node pushes a **GoToPointRayAngleGazeKinematicMetatask** in the stack.

This task allows the robot to look at a certain point in the space.

Some of the node properties are readed as a rosparam and could be modified.

Property name | Type |  Description | Default
------------- | ---- | ------------ | -------
source_data | std::string | Reference type. Defines how the target pose is specified (topic, pointer, interactive_marker). | interactive_marker |
camera_frame | std::string | Name of the camera frame. The camera frame has to be one of the optical frames. | xtion_optical_frame |
respect_task_id | std::string | The gaze_task is gonna be pushed after this specific task in the stack. | self_collision |
ns | std::string | Controller namespace. | /whole_body_kinematic_controller

#### Push Pose task

The push_interactive_marker_pose_task node pushes a task to control in position and orientation a specific link of the robot.

First it pushes a **GoToPositionMetaTask** / **GoToLocalVirtualAdmitancePositionMetaTask** task to control the position. And after this task it pushes a 
**GoToOrientationMetaTask** / **GoToLocalVirtualAdmitanceOrientationMetaTask** task to control the orientation.

Some of his properties are readed as a rosparam at could be modified.

Property name | Type |  Description | Default
------------- | ---- | ------------ | -------
source_data | std::string | Reference type. Defines how the target position is controlled (topic, pointer, interactive_marker). | interactive_marker_reflexx_typeII |
reference_frame | std::string | If source_data is pointer / pointer_reflexx_typeII, the reference frame is the frame for the reference | base_link. In the humanoids this param should be odom |
target_position | geometry_msgs::PointStamped | If source_data is pointer / pointer_reflexx_typeII, this param represents the target_position | This param is mandatory |
target_orientation | geometry_msgs::QuaternionStamped | If source_data is pointer / pointer_reflexx_typeII, this param represents the target_orientation | This param is mandatory |
tip_name | std::string | Controlled link name. | arm_tool_link |
respect_task_id | std::string | The position task is gonna be pushed after this specific task in the stack. | self_collision |
ns | std::string | Controller namespace. | /whole_body_kinematic_controller
damping | double | Damping to avoid local minima. | By default is set to 0.2
admitance_ft | std::string | FT name related to the controlled link. If ft name specified, it uses admitance control | By default is empty

@warning If the damping parameter is set to zero, the robot will start vibrating.

> Note: If admitance_ft the tasks pushed are of type **GoToLocalVirtualAdmitancePositionMetaTask** and **GoToLocalVirtualAdmitanceOrientationMetaTask**. Otherwise it pushes **GoToPositionMetaTask** and **GoToOrientationMetaTask**.

#### Push Relative task

The push_interactive_marker_relative_task node pushes relative task that constrains a link respect another one.

First it pushes a task to control a speficfic link in position and orientation. If a ft is specified this link is controlled via admittance control.

Then it pushes a relative task that constraints another link (slave) respect the controlled link (master).

This implies that the user control one link, but another link moves at the same time to keeps his position and orientation respect the controlled link.

Property name | Type |  Description | Default
------------- | ---- | ------------ | -------
source_data | std::string | Reference type. Defines how the master link is controlled (topic, pointer, interactive_marker). | interactive_marker_reflexx_typeII |
tip_name | std::string | Controlled link / Master link | arm_left_tool_link |
constrained_link | std::string | Constrained link / Slave link from the relative task | arm_right_tool_link |
respect_task_id | std::string | The position task is gonna be pushed after this specific task in the stack. | self_collision |
ns | std::string | Controller namespace. | /whole_body_kinematic_controller
admitance_ft | std::string | FT name related to the controlled link. If ft name specified, it uses admitance control | By default is empty
target_position | geometry_msgs::PointStamped | Relative position constraint from the master link to the constrained link | This param is mandatory |
target_orientation | geometry_msgs::QuaternionStamped | Relative orientation constraint from the master link to the constrained link | This param is mandatory |

<a name="stack"></a>
## 2. Push / Remove a task from the stack

There are two ways of push and remove tasks from the stack once the controller is loaded.

1. By using the WBC Service Helper class.
2. Using the WBC rosservices

- Prints the stack description
```sh
$ rosservice call /whole_body_kinematic_controller/get_stack_description "{}"
```
- Removes a specific task from the stack
```sh
$ rosservice call /whole_body_kinematic_controller/pop_task "name: 'task_id'
blend: false
blending_duration:
  data:
    secs: 0
    nsecs: 0"
```
- Get the task error of a specific task
```sh
$ rosservice call /whole_body_kinematic_controller/get_task_error "id: 'task_id'"
```
- Pushes a new task in the stack
```sh
$ rosservice call /whole_body_kinematic_controller/push_task "push_task_params:
  params: ''
  respect_task_id: ''
  order: {order: 0}
  blend: false
  blending_duration:
    data: {secs: 0, nsecs: 0}" 

```
The params contain the list of params for the property to be configured. The respect_task_id is the id of the task which the new task is pushed respect to, and the order corresponds to how the task
is pushed (before, after, same, replace).This method is not recomended because manually passing the parameters is extremely error prone.

The best way to do it is creating a node that pushes the desired tasks in the stack. As stated in the previous section, this package contains many nodes that push the most common tasks
in the stack.


> Note: In fact the WBC Service Helper class is a wrapper that uses the rosservices to push and pop the different tasks.

> Note: The blend parameter allows to push and remove tasks interpolating from the current position of the robot which will reduce discontinuities and smooth things. Although still under development.

@attention Don't remove tasks such as self_collision, joint_limits, foot_constraints... Removing this tasks may be potentially dangerous for the robot.
