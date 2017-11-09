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

/**
 * @brief The WBCTaskState class is the base class for states that do WBC tasks
 *
 * It simplifies interfacing with the WBC services via convenient protected functions.
 *
 * @warning Automatic bookkeeping, this state and substates will try to use the user data
 * ~wbc_bookkeeping for keeping track of the tasks that have been pushed and popped.
 * Later you can call a state to pop everything remaining in this bookkeeping.
 */
class WBCServiceHelper
{
public:
  WBCServiceHelper(ros::NodeHandle &nh);

  virtual ~WBCServiceHelper();

  std::string orderToString(pal_wbc_msgs::Order::_order_type order);

  bool pushTask(const property_bag::PropertyBag &properties, const std::string &task_id,
                const pal_wbc_msgs::Order::_order_type &order,
                const std::string &respect_task_id);

  bool popTask(const std::string &task_id);

  bool pushPopTask(const property_bag::PropertyBag &properties, const std::string &push_task_id,
                   const pal_wbc_msgs::Order::_order_type &order,
                   const std::string &push_respect_task_id,
                   const std::string &pop_task_id);

  pal_wbc_msgs::TaskError getTaskError(const std::string &task_id);

  bool printStackDescription();

  /**
   * @brief waitForConvergence Wait until task_id error is below eps
   * @param timeout Max time to wait
   * @param log_throttle How often to print logs
   * @param check_error_rate How often to check the error
   * @param abort_condition Function returning bool to be called to check if the wait should be aborted
   * @return true if task error is below eps and task is not preempted, false otherwise
   */
  bool waitForConvergence(const std::string &task_id, double eps,
                          const ros::Duration &timeout, double log_throttle = 1.0,
                          const ros::Duration &check_error_rate = ros::Duration(0.1),
                          const std::function<bool()> &abort_condition = [](){return false;});

  ros::NodeHandle nh_;
  ros::ServiceClient pop_task_srv_;
  ros::ServiceClient push_task_srv_;
  ros::ServiceClient push_pop_task_srv_;
  ros::ServiceClient get_task_error_srv_;
  ros::ServiceClient stack_description_srv_;
};

}  // namespace pal

#endif  // PAL_WBC_UTILS_H
