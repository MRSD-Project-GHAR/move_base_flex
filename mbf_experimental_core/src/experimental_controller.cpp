#include <mbf_experimental_core/experimental_controller.hpp>
// #include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalController, mbf_abstract_core::AbstractController)

namespace mbf_experimental_core
{
// class ExperimentalController : public AbstractController
// {
// public:
//   typedef boost::shared_ptr< ::mbf_abstract_core::ExperimentalController > Ptr;

/**
 * @brief Destructor
 */
//   virtual ~ExperimentalController(){};

/**
 * @brief Given the current position, orientation, and velocity of the robot,
 * compute velocity commands to send to the base.
 * @param pose The current pose of the robot.
 * @param velocity The current velocity of the robot.
 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
 * @param message Optional more detailed outcome as a string
 * @return Result code as described on ExePath action result:
 *         SUCCESS           = 0
 *         1..9 are reserved as plugin specific non-error results
 *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
 *         CANCELED          = 101
 *         NO_VALID_CMD      = 102
 *         PAT_EXCEEDED      = 103
 *         COLLISION         = 104
 *         OSCILLATION       = 105
 *         ROBOT_STUCK       = 106
 *         MISSED_GOAL       = 107
 *         MISSED_PATH       = 108
 *         BLOCKED_GOAL      = 109
 *         BLOCKED_PATH      = 110
 *         INVALID_PATH      = 111
 *         TF_ERROR          = 112
 *         NOT_INITIALIZED   = 113
 *         INVALID_PLUGIN    = 114
 *         INTERNAL_ERROR    = 115
 *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
 *         MAP_ERROR         = 117  # The map is not running properly
 *         STOPPED           = 118  # The controller execution has been stopped rigorously
 *         121..149 are reserved as plugin specific errors
 */
uint32_t ExperimentalController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                         const geometry_msgs::TwistStamped& velocity,
                                                         geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
    std::cout << "Computing dummy velocity commands! \n";

    ros::Duration sleep_time(0.004);
    sleep_time.sleep();

    cmd_vel.twist.linear.x = 1;

    std::cout << "computed dummy velocity commands! \n\n";

    // std::cout << "The pose I received is :" << pose << std::endl;
    if (cancel_requested_) {
        cancel_requested_ = false;
        return 101;
    }

    return 0;

}

/**
 * @brief Check if the goal pose has been achieved by the local planner
 * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
 * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
 * @return True if achieved, false otherwise
 */
bool ExperimentalController::isGoalReached(double dist_tolerance, double angle_tolerance)
{

    if((ros::Time::now() - plan_time).toSec() > 0.1) {
        std::cout << "Goal reached!\n\n";
        return true;
    }
    std::cout << "Checking if Goal Reached!\n";
    return false;
}

/**
 * @brief Set the plan that the local planner is following
 * @param plan The plan to pass to the local planner
 * @return True if the plan was updated successfully, false otherwise
 */
bool ExperimentalController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::cout << "Setting Plan!\n";
    plan_time = ros::Time::now();
    return true;
}

/**
 * @brief Requests the planner to cancel, e.g. if it takes too much time.
 * @return True if a cancel has been successfully requested, false if not implemented.
 */
bool ExperimentalController::cancel()
{
    std::cout << "Cancelling!\n";
    cancel_requested_ = true;
    return true;
}

// protected:
/**
 * @brief Constructor
 */
//   ExperimentalController(){};
}  // namespace mbf_experimental_core
