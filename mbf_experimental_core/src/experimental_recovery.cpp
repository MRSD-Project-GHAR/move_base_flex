#include <mbf_experimental_core/experimental_recovery.hpp>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(mbf_experimental_core::ExperimentalRecovery, mbf_abstract_core::AbstractRecovery)

namespace mbf_experimental_core
{
/**
 * @class AbstractRecovery
 * @brief Provides an interface for recovery behaviors used in navigation.
 * All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
 */
//   class ExperimentalRecovery{
// public:

//   typedef boost::shared_ptr< ::mbf_experimental_core::AbstractRecovery > Ptr;

/**
 * @brief Runs the AbstractRecovery
 * @param message The recovery behavior could set, the message should correspond to the return value
 * @return An outcome which will be hand over to the action result.
 */
uint32_t ExperimentalRecovery::runBehavior(std::string& message)
{
  std::cout << "Recovery Behavior Running!\n";
  return 0;
}

/**
 * @brief Virtual destructor for the interface
 */
//   virtual ~ExperimentalRecovery(){}

/**
 * @brief Requests the recovery behavior to cancel, e.g. if it takes too much time.
 * @return True if a cancel has been successfully requested, false if not implemented.
 */
bool ExperimentalRecovery::cancel()
{
  std::cout << "Cancelled! \n";
  return true;
}

// protected:
/**
 * @brief Constructor
 */
//   ExperimentalRecovery(){};
}; /* namespace mbf_experimental_core */

// #endif  /* MBF_ABSTRACT_CORE__ABSTRACT_RECOVERY_H_ */
