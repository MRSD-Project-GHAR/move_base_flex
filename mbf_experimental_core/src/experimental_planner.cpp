#include <mbf_experimental_core/experimental_planner.hpp>

PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalPlanner, mbf_abstract_core::AbstractPlanner)

namespace mbf_experimental_core
{
uint32_t ExperimentalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                       double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                       std::string& message)

{
  std::cout << "Making plan!\n";

  int count = 0;

  // This is the loop where you would be you making the global plan. As you can see here, if a cancel is requested, I
  // exit out of the loop and return the error code for CANCELLED
  while (!cancel_requested_ && count < 10)
  {
    // Sleep for one second, which simulates execution of a computationally expensive planning algorithm
    ros::Duration sleep_time(1);
    sleep_time.sleep();
    std::cout << "Planning loop is running\n";
    count++;
  }

  if (cancel_requested_)
  {
    cancel_requested_ = false;
    return 51;
  }

  // Make a dummy plan consisting of just one point, (10, 0, 0)
  geometry_msgs::PoseStamped dummy_plan;
  dummy_plan.pose.position.x = 10.0;
  dummy_plan.pose.orientation.w = 1.0;
  dummy_plan.header.frame_id = "odom";
  plan.push_back(dummy_plan);
  std::cout << "Made a dummy plan!" << std::endl;

  return 0;
}

bool ExperimentalPlanner::cancel()
{
  std::cout << "Plan is cancelled!";
  cancel_requested_ = true;  // This will allow us to get out of the loop prematurely if we want to
  return true;
}

}  // namespace mbf_experimental_core
