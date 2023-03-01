#include <mbf_experimental_core/experimental_recovery.hpp>
#include <iostream>

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