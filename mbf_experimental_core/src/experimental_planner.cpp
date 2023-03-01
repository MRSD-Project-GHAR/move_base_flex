#include <mbf_experimental_core/experimental_planner.hpp>

PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalPlanner, mbf_abstract_core::AbstractPlanner)

namespace mbf_experimental_core
{
/**
 * @class ExperimentalPlanner
 * @brief Provides an interface for global planners used in navigation.
 * All global planners written to work as MBF plugins must adhere to this interface. Alternatively, this
 * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
//  */
// class ExperimentalPlanner : public mbf_abstract_core::AbstractPlanner
// {
// public:
//   typedef boost::shared_ptr< ::mbf_costmap_core::ExperimentalPlanner > Ptr;

/**
 * @brief Given a goal pose in the world, compute a plan
 * @param start The start pose
 * @param goal The goal pose
 * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
 *        in x and y before failing
 * @param plan The plan... filled by the planner
 * @param cost The cost for the the plan
 * @param message Optional more detailed outcome as a string
 * @return Result code as described on GetPath action result:
 *         SUCCESS         = 0
 *         1..9 are reserved as plugin specific non-error results
 *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
 *         CANCELED        = 51
 *         INVALID_START   = 52
 *         INVALID_GOAL    = 53
 *         NO_PATH_FOUND   = 54
 *         PAT_EXCEEDED    = 55
 *         EMPTY_PATH      = 56
 *         TF_ERROR        = 57
 *         NOT_INITIALIZED = 58
 *         INVALID_PLUGIN  = 59
 *         INTERNAL_ERROR  = 60
 *         71..99 are reserved as plugin specific errors
 */
uint32_t ExperimentalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                       double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                       std::string& message)

{
  if (cancel_requested_)
  {
    cancel_requested_ = false;
    return 51;
  }

  std::cout << "Making plan!\n";

  int count = 0;
  while (!cancel_requested_ && count < 10)
  {
    ros::Duration sleep_time(1);
    sleep_time.sleep();
    std::cout << "Planning loop is running\n";
    count++;
  }

  geometry_msgs::PoseStamped dummy_plan;
  dummy_plan.pose.position.x = 10.0;
  dummy_plan.pose.orientation.w = 1.0;
  dummy_plan.header.frame_id = "odom";
  plan.push_back(dummy_plan);
  std::cout << "Made a dummy plan!" << std::endl;

  return 0;
}

/**
 * @brief Requests the planner to cancel, e.g. if it takes too much time.
 * @remark New on MBF API
 * @return True if a cancel has been successfully requested, false if not implemented.
 */
bool ExperimentalPlanner::cancel()
{
  std::cout << "Plan is cancelled!";
  cancel_requested_ = true;
  return true;
}

/**
 * @brief Initialization function for the ExperimentalPlanner
 * @param name The name of this planner
 * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
 */
// void ExperimentalPlanner::initialize(std::string name, int map);

/**
 * @brief  Virtual destructor for the interface
 */
//  ~ExperimentalPlanner()
// {
// }

// ExperimentalPlanner()
// {
// }
}  // namespace mbf_experimental_core
