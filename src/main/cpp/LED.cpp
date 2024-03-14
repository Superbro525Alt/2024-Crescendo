// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "LED.h"

LED::LED() {}

void LED::OnUpdate(units::second_t dt) {
  std::string stateName = "";
  switch (_state) {
    case LEDState::kIdle: {
      stateName = "IDLE";
      _led.frc::PWM::SetSpeed(0.99); //Off
    } break;
    case LEDState::kAiming: {
      stateName = "Aiming";
      _led.frc::PWM::SetSpeed(-0.03); // green flash
    } break;
    case LEDState::kShooterReady: {
      stateName = "Shooter";
      _led.frc::PWM::SetSpeed(0.89); //green solid
    } break;
    case LEDState::kAmpReady: {
      stateName = "Amp Ready";
      // _led.frc::PWM::SetSpeed(0.87);
    } break;
    case LEDState::kHold: {
      stateName = "Hold";
      _led.frc::PWM::SetSpeed(0.77); //white flash
    } break;
    case LEDState::kIntaking: {
      stateName = "Intaking";
      _led.frc::PWM::SetSpeed(0.76); //yellow solid
    } break;
    

    default:
      break;
  }

  std::cout << "LED State: " << stateName << std::endl;
}

void LED::SetState(LEDState state) {
  _state = state;
}

LEDState LED::GetState() {
  return _state;
}
