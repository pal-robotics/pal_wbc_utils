/**************************************************************************
**
**  File: wbcservicehelper.cpp
**
**  Author: victor
**  Created on: 2017/11/09
**
**  Copyright (c) 2017 PAL Robotics SL. All Rights Reserved
**************************************************************************/
#include <pal_wbc_msgs/GetStackDescription.h>
#include <pal_wbc_msgs/GetTaskError.h>
#include <pal_wbc_msgs/PopTask.h>
#include <pal_wbc_msgs/PushPopTask.h>
#include <pal_wbc_msgs/PushTask.h>
#include <pal_wbc_utils/pal_wbc_utils.h>
#include <property_bag/serialization/eigen_boost_serialization.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <property_bag/serialization/property_boost_serialization.h>
#include <std_srvs/Empty.h>

namespace pal
{
std::string generateTaskDescription(const property_bag::PropertyBag properties)
{
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << properties;
  return ss.str();
}

WBCServiceHelper::WBCServiceHelper(ros::NodeHandle &nh, const std::string &ns)
  : nh_(nh), ns_(ns)
{
  bool persistent = false;  // Not persistent, because when we enable WBC the services are
                            // created, not when the states are created
  pop_task_srv_ = nh_.serviceClient<pal_wbc_msgs::PopTask>(ns_ + "/pop_task", persistent);
//  push_pop_task_srv_ =
//      nh_.serviceClient<pal_wbc_msgs::PushPopTask>(ns_ + "/push_pop_task", persistent);
  push_task_srv_ = nh_.serviceClient<pal_wbc_msgs::PushTask>(ns_ + "/push_task", persistent);
  get_task_error_srv_ =
      nh_.serviceClient<pal_wbc_msgs::GetTaskError>(ns_ + "/get_task_error", persistent);
  stack_description_srv_ = nh_.serviceClient<pal_wbc_msgs::GetStackDescription>(
      ns_ + "/get_stack_description", persistent);
}

WBCServiceHelper::~WBCServiceHelper()
{
}

bool WBCServiceHelper::check_if_services_are_ready(const ros::Duration &timeout)
{
  bool ready_1 = pop_task_srv_.waitForExistence(timeout);
  // bool ready_2 = push_pop_task_srv_.waitForExistence(timeout);
  bool ready_3 = push_task_srv_.waitForExistence(timeout);
  bool ready_4 = get_task_error_srv_.waitForExistence(timeout);
  bool ready_5 = stack_description_srv_.waitForExistence(timeout);

  if ((!ready_1) || (!ready_3) || (!ready_4) || (!ready_5))
  {
    ROS_ERROR_STREAM("problem waitting for existence of wbc services");
    return false;
  }

  return true;
}

std::string WBCServiceHelper::orderToString(pal_wbc_msgs::Order::_order_type order)
{
  switch (order)
  {
    case pal_wbc_msgs::Order::After:
      return "after";
    case pal_wbc_msgs::Order::Before:
      return "before";
    case pal_wbc_msgs::Order::Same:
      return "same";
    case pal_wbc_msgs::Order::Replace:
      return "replace";
    default:
      return "unknown order";
  }
}

bool WBCServiceHelper::pushTask(const property_bag::PropertyBag &properties,
                                const std::string &task_id,
                                const pal_wbc_msgs::Order::_order_type &order,
                                const std::string &respect_task_id)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  pal_wbc_msgs::PushTask srv;
  property_bag::PropertyBag task_properties;
  task_properties = properties;
  task_properties.addProperty("task_id", task_id);
  srv.request.push_task_params.params = generateTaskDescription(task_properties);
  srv.request.push_task_params.order.order = order;
  srv.request.push_task_params.respect_task_id = respect_task_id;
  if (push_task_srv_.call(srv))
  {
    ROS_INFO_STREAM("Succesfully pushed task:" << task_id << " " << orderToString(order)
                                               << " " << respect_task_id);
    printStackDescription();
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call push task service");
    return false;
  }
}

bool WBCServiceHelper::popTask(const std::string &task_id)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  pal_wbc_msgs::PopTask srv;
  srv.request.name = task_id;
  if (pop_task_srv_.call(srv))
  {
    ROS_INFO_STREAM("Succesfully popped task:" << task_id);
    printStackDescription();
    return true;
  }
  else
  {
    ROS_ERROR("Failed to pop call service ");
    return false;
  }
}

/*
bool WBCServiceHelper::pushPopTask(const property_bag::PropertyBag &properties,
                                   const std::string &push_task_id,
                                   const pal_wbc_msgs::Order::_order_type &order,
                                   const std::string &push_respect_task_id,
                                   const std::string &pop_task_id)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  pal_wbc_msgs::PushTaskParams push_task;
  push_task.params = generateTaskDescription(properties);
  push_task.order.order = order;
  push_task.respect_task_id = push_respect_task_id;

  pal_wbc_msgs::PushPopTask srv;
  srv.request.push_tasks.push_back(push_task);
  srv.request.pop_tasks.push_back(pop_task_id);
  if (push_pop_task_srv_.call(srv))
  {
    ROS_INFO_STREAM("Succesfully pushed task " << push_task_id << " and popped tasks "
                                               << pop_task_id);
    printStackDescription();
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call pushPop task service");
    return false;
  }
}
*/

pal_wbc_msgs::TaskError WBCServiceHelper::getTaskError(const std::string &task_id)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return pal_wbc_msgs::TaskError();

  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = task_id;
  get_task_error_srv_.call(srv);
  return srv.response.taskError;
}

bool WBCServiceHelper::taskExists(const std::string &task_id)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  pal_wbc_msgs::GetStackDescription statusSrv;
  if (stack_description_srv_.call(statusSrv))
  {
    for (auto task : statusSrv.response.tasks)
    {
      if (task.name == task_id)
        return true;
    }
    return false;
  }
  ROS_ERROR("Failed to call service ");
  return false;
}

bool WBCServiceHelper::printStackDescription()
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  pal_wbc_msgs::GetStackDescription statusSrv;
  if (stack_description_srv_.call(statusSrv))
  {
    ROS_INFO_STREAM("Stack description:");
    ROS_INFO_STREAM(statusSrv.response);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return false;
  }
}

bool WBCServiceHelper::waitForConvergence(const std::string &task_id, double eps,
                                          const ros::Duration &timeout, double log_throttle,
                                          const ros::Duration &check_error_rate,
                                          const std::function<bool()> &abort_condition)
{
  if (!check_if_services_are_ready(ros::Duration(10.0)))
    return false;

  // Create an instance of persistent service, because it will be spammed
  get_task_error_srv_ =
      nh_.serviceClient<pal_wbc_msgs::GetTaskError>(ns_ + "/get_task_error", true);
  pal_wbc_msgs::TaskError task_error;
  ros::Time timeout_time = ros::Time::now() + timeout;
  do
  {
    ros::Time next_error_check = ros::Time::now() + check_error_rate;
    while (ros::ok() && (ros::Time::now() < next_error_check))  // Inside this loop we
                                                                // check for peremptions
                                                                // at 100Hz
    {
      if (abort_condition())
      {
        ROS_WARN_STREAM("Wait for " << task_id << " preempted, remember to "
                                                  "servicePreempt() in your task");
        return false;
      }
      ros::Duration(0.01).sleep();
    }

    try
    {
      task_error = getTaskError(task_id);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM(e.what());
      return false;
    }

    ROS_INFO_STREAM_THROTTLE(
        log_throttle, "Task: " << task_id << " error norm is: " << task_error.error_norm);
    if (ros::Time::now() > timeout_time)
    {
      ROS_ERROR_STREAM("Task did not converge");
      return false;
    }
  } while (ros::ok() && (task_error.error_norm > eps));
  ROS_INFO_STREAM("FINISH Task: " << task_id << " error norm is: " << task_error.error_norm);

  return true;
}
}  // namespace pal
