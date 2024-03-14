// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Shooter.h"

Shooter::Shooter(ShooterConfig config) : _config(config), _pid(frc::PIDController(0.02, 0.5, 0, 0.05_s)) {}

void Shooter::OnStart() {
  _pid.Reset();
}

void Shooter::OnUpdate(units::second_t dt) {
  table->GetEntry("Error").SetDouble(_pid.GetPositionError());
  table->GetEntry("SetPoint").SetDouble(_pid.GetSetpoint());
  table->GetEntry("Encoder Output").SetDouble(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value());
  table->GetEntry("Shooting").SetString(_statename);

  switch (_state) {
    case ShooterState::kIdle: 
    {
      _statename = "Idle";
      if (_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value() > 10) {
        _pid.SetSetpoint(0);
        units::volt_t pidCalculate = units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};
        _setVoltage = pidCalculate;
      } else {
        _pid.Reset();
        holdVoltage = 0_V;
        _setVoltage = 0_V;
      }
    } 
    break;
    case ShooterState::kSpinUp: {
      _statename = "SpinUp";
      _pid.SetSetpoint(_goal.value());
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};

      _setVoltage = pidCalculate;

      if (std::abs(_pid.GetVelocityError()) < 5) {
        _upToSpeed = true;
      } else {
        _upToSpeed = false;
      }

    } break;
    case ShooterState::kShooting: {
      _statename = "Shooting";
      _pid.SetSetpoint(_goal.value());
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};

      _setVoltage = pidCalculate;
    } break;

    case ShooterState::kReverse: {
      _statename = "Reverse";
      _pid.Reset();
      _setVoltage = -8_V;
      std::cout << "KReverse" << std::endl;
      // if (!_shooterSensor.Get()) {
      //   SetState(ShooterState::kIdle);
      // }
    } break;
    case ShooterState::kRaw: {
      _statename = "Raw";
      holdVoltage = 0_V;
      _pid.Reset();
      _setVoltage = _rawVoltage;
      std::cout << "KRaw" << std::endl;
      // if (_shooterSensor.Get()) {
      //   SetState(ShooterState::kRaw);
      // }
    } break;
    default: {
      std::cout << "Error shooter in invalid state" << std::endl;
    } break;
  }
  // table->GetEntry("Motor OutPut").SetDouble(_setVoltage.value());

  _config.ShooterGearbox.motorController->SetVoltage(_setVoltage);
}

//       // if (_pid.AtSetpoint()) {
//       //   SetState(ShooterState::kShooting);
//       // }
//       // table->GetEntry("PID Setpoint:").SetDouble(_pid.GetSetpoint());
//       std::cout << "KShooting" << std::endl;
//       _pid.SetSetpoint(20);
//       // _pid.SetSetpoint(_goal.value());

void Shooter::SetState(ShooterState state) {
  _state = state;
}
void Shooter::SetRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
  _state = ShooterState::kRaw;
}
void Shooter::SetPidGoal(units::radians_per_second_t goal) {
  _goal = goal;
}

ShooterState Shooter::GetState() {
  return _state;
}

bool Shooter::GetStable() {
  return _upToSpeed;
}
