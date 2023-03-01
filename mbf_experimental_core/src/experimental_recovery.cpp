#include <mbf_experimental_core/experimental_recovery.hpp>
#include <iostream>

// This is what exports this class as ROS plugin. The syntax is PLUGINLIB_EXPORT_CLASS(derived class, base class)
// You need to add some stuff in package.xml and CMakeLists.txt as well. The best way to know what all you have to do to
// export a plugin is to go through this ROS tutorial
// http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalRecovery, mbf_abstract_core::AbstractRecovery)

namespace mbf_experimental_core
{

uint32_t ExperimentalRecovery::runBehavior(std::string& message)
{
  std::cout << "Recovery Behavior Running!\n";
  return 0;
}

bool ExperimentalRecovery::cancel()
{
  std::cout << "Cancelled! \n";
  return true;
}

}; /* namespace mbf_experimental_core */