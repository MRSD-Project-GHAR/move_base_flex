#include <ros/ros.h>
// #include "mbf_experimental_core/experimental_planner.hpp"
#include "mbf_experimental_nav/experimental_navigation_server.hpp"
#include <tf2_ros/transform_listener.h>
#include <signal.h>

typedef std::shared_ptr<mbf_experimental_nav::ExperimentalNavigationServer> ExperimentalNavigationServerPtr;
ExperimentalNavigationServerPtr experimental_nav_srv_ptr;

void sigintHandler(int sig)
{
  ROS_INFO_STREAM("Shutdown navigation server.");
  if (experimental_nav_srv_ptr)
  {
    experimental_nav_srv_ptr->stop();
  }

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "experimental_nav_server", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  signal(SIGINT, sigintHandler);
#ifdef USE_OLD_TF
  TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
#else
  float cache_time = 5.0;
  TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);
#endif

  // costmap_nav_srv_ptr = boost::make_shared<mbf_costmap_nav::CostmapNavigationServer>(tf_listener_ptr);
  // ros::spin();

  auto experimental_nav_srv_ptr =
      boost::make_shared<mbf_experimental_nav::ExperimentalNavigationServer>(tf_listener_ptr);

  ros::spin();

  //   double cache_time;
  //   private_nh.param("tf_cache_time", cache_time, 10.0);

  //   signal(SIGINT, sigintHandler);
  // #ifdef USE_OLD_TF
  //   TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
  // #else
  //   TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  //   tf2_ros::TransformListener tf_listener(*tf_listener_ptr);
  // #endif
  //   costmap_nav_srv_ptr = boost::make_shared<mbf_costmap_nav::CostmapNavigationServer>(tf_listener_ptr);
  //   ros::spin();

  // explicitly call destructor here, otherwise costmap_nav_srv_ptr will be
  // destructed after tearing down internally allocated static variables
  experimental_nav_srv_ptr.reset();

  return 0;
}
