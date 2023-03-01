#pragma once

#include <mbf_abstract_core/abstract_recovery.h>
#include <pluginlib/class_list_macros.h>

namespace mbf_experimental_core
{
/**
 * @class AbstractRecovery
 * @brief TODO
 */
class ExperimentalRecovery : public mbf_abstract_core::AbstractRecovery
{
public:
  /**
   * @brief Runs the AbstractRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message);

  /**
   * @brief Requests the recovery behavior to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel();
};
}; /* namespace mbf_experimental_core */
