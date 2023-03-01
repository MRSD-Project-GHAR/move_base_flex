#include <mbf_experimental_core/experimental_controller.hpp>
#include <iostream>

// This is what exports this class as ROS plugin. The syntax is PLUGINLIB_EXPORT_CLASS(derived class, base class)
// You need to add some stuff in package.xml and CMakeLists.txt as well. The best way to know what all you have to do to
// export a plugin is to go through this ROS tutorial
// http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalController, mbf_abstract_core::AbstractController)

namespace mbf_experimental_core
{
uint32_t ExperimentalController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                         const geometry_msgs::TwistStamped& velocity,
                                                         geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  std::cout << "Computing dummy velocity commands! \n";

  ros::Duration sleep_time(0.004);  // Some sleep time so that we are not bombarded with print messages
  sleep_time.sleep();

  cmd_vel.twist.linear.x = 1;  // Gives some dummy velocity command

  std::cout << "Computed dummy velocity commands! \n\n";

  if (cancel_requested_)  // If cancel requested, then return the error code for CANCELLED
  {
    cancel_requested_ = false;
    return 101;
  }

  return 0;
}

bool ExperimentalController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  // If the time since receiving the plan greater than 0.1 seconds, say that the robot has reached the goal
  if ((ros::Time::now() - plan_time).toSec() > 0.1)
  {
    std::cout << "Goal reached!\n\n";
    return true;
  }
  std::cout << "Checking if Goal Reached!\n";
  return false;
}

bool ExperimentalController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  std::cout << "Setting Plan!\n";
  plan_time = ros::Time::now();  // sets the time at which the plan was received
  return true;
}

bool ExperimentalController::cancel()
{
  // cancel requested, in the next controller loop the execution will return the error code for CANCELLED
  std::cout << "Cancelling!\n";
  cancel_requested_ = true;
  return true;
}

}  // namespace mbf_experimental_core
