#include "behaviour/HasBehaviour.h"

#include "behaviour/Behaviour.h"

;

void wom::HasBehaviour::SetDefaultBehaviour(std::function<std::shared_ptr<wom::Behaviour>(void)> fn) {
  _default_behaviour_producer = fn;
}

std::shared_ptr<wom::Behaviour> wom::HasBehaviour::GetActiveBehaviour() {
  return _active_behaviour;
}