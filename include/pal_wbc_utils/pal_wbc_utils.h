/**************************************************************************
**
**  File: wbc_task_state.h
**
**  Author: victor
**  Created on: 2017/11/09
**
**  Copyright (c) 2017 PAL Robotics SL. All Rights Reserved
**************************************************************************/
#ifndef PAL_WBC_UTILS_H
#define PAL_WBC_UTILS_H
#include <pal_wbc_msgs/Order.h>
#include <pal_wbc_msgs/TaskError.h>
#include <property_bag/property_bag.h>
#include <ros/ros.h>

namespace pal
{

std::string generateTaskDescription(const property_bag::PropertyBag properties);

/*! \class WBCServiceHelper
 @brief A helper class to push, pop and query tasks in the WBC stack.

 It simplifies interfacing with the WBC services via convenient functions.
*/
class WBCServiceHelper
{
public:

  /*! WBCServiceHelper
  @brief Default constructor
  @param nh
  @param ns: Controller namespace
  */
  WBCServiceHelper(ros::NodeHandle &nh, const std::string &ns = "/whole_body_kinematic_controller");

  virtual ~WBCServiceHelper();

  /*! orderToString
  @brief Translates pal_wbc_msgs::Order as a string
  @param order: Order msg type
  @return std::string: Order type
  */
  std::string orderToString(pal_wbc_msgs::Order::_order_type order);

  /*! pushTask
  @brief Pushes a new task in the stack
  @param properties: Contains the properties to configure the task, serialized in the PropertyBag.
  @param task_id: Id of the task
  @param order: Order in which the task is pushed (Before, After, Replace, Same)
  @param respect_task_id: Id of the task with respect to the new task is pushed.
  @return bool: True if success / False otherwise
  */
  bool pushTask(const property_bag::PropertyBag &properties, const std::string &task_id,
                const pal_wbc_msgs::Order::_order_type &order,
                const std::string &respect_task_id);

  /*! popTask
  @brief Removes a task from the stack
  @param task_id: Name of the task to be removed
  */
  bool popTask(const std::string &task_id);

//  bool pushPopTask(const property_bag::PropertyBag &properties, const std::string &push_task_id,
//                   const pal_wbc_msgs::Order::_order_type &order,
//                   const std::string &push_respect_task_id,
//                   const std::string &pop_task_id);

  /*! getTaskError
  @brief Returns task error
  @param task:Id: Name of the task
  @return pal_wbc_msgs::TaskError: Task error msg
  */
  pal_wbc_msgs::TaskError getTaskError(const std::string &task_id);

  /*! taskExists
  @brief Check if a task exists in the stack
  @param task:Id: Name of the task
  @return bool: True if task exists / False otherwise
  */
  bool taskExists(const std::string &task_id);

  /*! check_if_services_are_ready
  @brief Check if wbc services are ready
  @param timeout: Time duration until it returns failure becuase services aren't ready
  @return bool: True if services are ready / False otherwise
  */
  bool check_if_services_are_ready(const ros::Duration &timeout);

  /*! printStackDescription
  @brief Print the current stack in the ROS logs
  @return bool: True if service return ok / False otherwise
  */
  bool printStackDescription();

  /*! waitForConvergence
   @brief Wait until task_id error is below eps
   @param eps. Tolerance error
   @param timeout: Max time to wait
   @param log_throttle: How often to logs are printed
   @param check_error_rate: How often the error is being checked
   @param abort_condition: Function returning bool to be called to check if the wait should be aborted
   @return bool: True if task error is below eps and task is not preempted / False otherwise
  */
  bool waitForConvergence(const std::string &task_id, double eps,
                          const ros::Duration &timeout, double log_throttle = 1.0,
                          const ros::Duration &check_error_rate = ros::Duration(0.1),
                          const std::function<bool()> &abort_condition = [](){return false;});

  ros::NodeHandle nh_;
  ros::ServiceClient pop_task_srv_;
  ros::ServiceClient push_task_srv_;
//  ros::ServiceClient push_pop_task_srv_;
  ros::ServiceClient get_task_error_srv_;
  ros::ServiceClient stack_description_srv_;
  std::string ns_;
};

}  // namespace pal

#endif  // PAL_WBC_UTILS_H
