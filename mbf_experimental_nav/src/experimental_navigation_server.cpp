#include "mbf_experimental_nav/experimental_navigation_server.hpp"
#include <iostream>

namespace mbf_experimental_nav
{
ExperimentalNavigationServer::ExperimentalNavigationServer(const TFPtr& tf_listener_ptr)
  : AbstractNavigationServer(tf_listener_ptr)
{
  std::cout << "Constructor Called!\n\n";
}

mbf_abstract_core::AbstractPlanner::Ptr ExperimentalNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  std::cout << "Dummy Planner plugin loaded\n";
  // std::cout << "Type is " << planner_type << "\n\n"
  return NULL;

  // return true;
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
  std::cout << "Dummy controller plugin loaded\n\n";
//   return true;
  return NULL;

}

bool ExperimentalNavigationServer::initializeControllerPlugin(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  std::cout << "Dummy Controller plugin initialized\n\n";
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr ExperimentalNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  std::cout << "Dummy Recovery plugin loaded\n\n";
//   return true;
  return NULL;

}

bool ExperimentalNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  std::cout << "Dummy Recovery plugin initialized\n\n";
  return true;
}

}  // namespace mbf_experimental_nav