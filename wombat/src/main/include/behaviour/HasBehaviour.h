#pragma once

#include <functional>
#include <memory>

namespace wom {
namespace behaviour {
class wom::Behaviour;
class BehaviourScheduler;

/**
 * wom::HasBehaviour is applied to a system that can be controlled by behaviours.
 * This is commonly implemented on shooters, drivetrains, elevators, etc.
 */
class wom::HasBehaviour {
 public:
  /**
   * Set the default behaviour to run if no behaviours are currently running.
   * This is commonly used to default to Teleoperated control.
   */
  void SetDefaultBehaviour(std::function<std::shared_ptr<wom::Behaviour>(void)> fn);

  /**
   * Get the currently running behaviour on this system.
   */
  std::shared_ptr<wom::Behaviour> GetActiveBehaviour();

 protected:
  std::shared_ptr<wom::Behaviour>                      _active_behaviour{nullptr};
  std::function<std::shared_ptr<wom::Behaviour>(void)> _default_behaviour_producer{nullptr};

 private:
  friend class BehaviourScheduler;
};
}  // namespace wom {
}