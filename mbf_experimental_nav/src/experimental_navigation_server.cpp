#include "mbf_experimental_nav/experimental_navigation_server.hpp"
#include <iostream>
#include <string>

namespace mbf_experimental_nav
{
ExperimentalNavigationServer::ExperimentalNavigationServer(const TFPtr& tf_listener_ptr)
  // : AbstractNavigationServer(tf_listener_ptr)
  : mbf_abstract_nav::AbstractNavigationServer(tf_listener_ptr), 
    planner_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractPlanner"),
    recovery_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractRecovery"),
    controller_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractController")

{
  std::cout << "Constructor Called!\n\n";

  initializeServerComponents();


}

ExperimentalNavigationServer::~ExperimentalNavigationServer() {
  planner_plugin_manager_.clearPlugins();
  controller_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();
}

mbf_abstract_core::AbstractPlanner::Ptr ExperimentalNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr = NULL;
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));

    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based planner plugin " << planner_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << planner_type << " planner." << ex_mbf_core.what());
                                          // << " Try to load as a nav_core-based plugin. " << ex_mbf_core.what());
  }

  return planner_ptr;
  // return NULL;
}

bool ExperimentalNavigationServer::initializePlannerPlugin(const std::string& name,
                                                           const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  std::cout << "Dummy Planner plugin initialized\n\n";
  return true;
}

mbf_abstract_core::AbstractController::Ptr
ExperimentalNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  // std::cout << "Dummy controller plugin loaded\n\n";
  //   return true;
  // return NULL;

  mbf_abstract_core::AbstractController::Ptr controller_ptr = NULL;
  try
  {
    controller_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractController>(
        controller_plugin_loader_.createInstance(controller_type));

    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based controller plugin " << controller_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << controller_type << " controller." << ex_mbf_core.what());
                                          // << " Try to load as a nav_core-based plugin. " << ex_mbf_core.what());
  }

  return controller_ptr;
}

bool ExperimentalNavigationServer::initializeControllerPlugin(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  std::cout << "Dummy Controller plugin initialized\n\n";
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr
ExperimentalNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr = NULL;
  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        recovery_plugin_loader_.createInstance(recovery_type));

    std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based recovery plugin " << recovery_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException &ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << recovery_type << " recovery." << ex_mbf_core.what());
                                          // << " Try to load as a nav_core-based plugin. " << ex_mbf_core.what());
  }

  return recovery_ptr;
}

bool ExperimentalNavigationServer::initializeRecoveryPlugin(
    const std::string& name, const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  std::cout << "Dummy Recovery plugin initialized\n\n";
  return true;
}

}  // namespace mbf_experimental_nav