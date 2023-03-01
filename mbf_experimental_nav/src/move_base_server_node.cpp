#include <ros/ros.h>
#include "mbf_experimental_nav/experimental_navigation_server.hpp"
#include <tf2_ros/transform_listener.h>
#include <signal.h>

typedef std::shared_ptr<mbf_experimental_nav::ExperimentalNavigationServer> ExperimentalNavigationServerPtr;
ExperimentalNavigationServerPtr experimental_nav_srv_ptr;

/**
 * @brief Custom handler for when CTRL-C is pressed or node shutdown is called. We don't want ROS to handle it since we
 * want the navigation server to properly shutdown when the node is being shutdown.
 */
void sigintHandler(int sig)
{
  ROS_INFO_STREAM("Shutdown navigation server.");
  if (experimental_nav_srv_ptr)
  {
    // This ensures that all ongoing actions are properly stopped before the node exits.
    experimental_nav_srv_ptr->stop();
  }

  ros::shutdown();
}

int main(int argc, char** argv)
{
  // Tells ROS not to use its default SIGINT handler
  ros::init(argc, argv, "experimental_nav_server", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Specify our custom function as the SIGINT handler (this function will be called when CTRL-C is pressed)
  signal(SIGINT, sigintHandler);

  float cache_time = 5.0;
  TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);

  // makes a shared pointer to the ExperimentalNavigationServer
  auto experimental_nav_srv_ptr =
      boost::make_shared<mbf_experimental_nav::ExperimentalNavigationServer>(tf_listener_ptr);

  ros::spin();

  // Reset the experimental_nav_server so that the code destructs properly
  experimental_nav_srv_ptr.reset();
  return 0;
}
