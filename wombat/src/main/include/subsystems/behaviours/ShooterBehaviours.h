#pragma once

#include "behaviour/Behaviour.h"
#include "subsystems/Shooter.h"

namespace wom {
namespace subsystems {
  namespace behaviours {
    class ShooterConstant : public behaviour::wom::Behaviour {
     public:
      ShooterConstant(Shooter *s, units::volt_t setpoint);

      void OnTick(units::second_t dt) override;

     private:
      Shooter      *_shooter;
      units::volt_t _setpoint;
    };

    class ShooterSpinup : public behaviour::wom::Behaviour {
     public:
      ShooterSpinup(Shooter *s, units::radians_per_second_t speed, bool hold = false);

      void OnTick(units::second_t dt) override;

     private:
      Shooter                    *_shooter;
      units::radians_per_second_t _speed;
      bool                        _hold;
    };
  }  // namespace behaviours
}  // namespace subsystems
}  // namespace wom
