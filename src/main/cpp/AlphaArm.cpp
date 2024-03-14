// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"

AlphaArm::AlphaArm(AlphaArmConfig* config /*, frc::Rotation2d initialAngle, wom::vision::Limelight* vision */)
    : _config(config),
      _pidArm{frc::PIDController(1.2, 0.4, 0)},
      _pidArmStates{frc::PIDController(37, 0.00070, 0.15)},
      _pidIntakeState{frc::PIDController(20, 0.00015, 0.005)},
      _pidClimberStates{frc::PIDController(25, 0.00015, 0.005)} {}

void AlphaArm::OnStart() {
  _pidArmStates.Reset();
  _pidIntakeState.Reset();
}

void AlphaArm::OnUpdate(units::second_t dt) {
  switch (_state) {
    case AlphaArmState::kIdle:
      _stringStateName = "kIdle";
      _setAlphaArmVoltage = 0_V;
      break;
    case AlphaArmState::kRaw:
      _stringStateName = "kRaw";
      _setAlphaArmVoltage = _rawArmVoltage;
      break;
    case AlphaArmState::kHoldAngle: 
    {
      _stringStateName = "kHoldAngle";
      _pidIntakeState.SetSetpoint(_goal);
      _setAlphaArmVoltage = -units::volt_t{ _pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
    } 
    break;
    case AlphaArmState::kAmpAngle:
      _stringStateName = "kAmpAngle";

      _pidArmStates.SetSetpoint(1.9);
      _setAlphaArmVoltage = -units::volt_t{ _pidArmStates.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kIntakeAngle:
      _stringStateName = "kIntakeAngle";

      _pidIntakeState.SetSetpoint(0.04);  // 3.8
      _setAlphaArmVoltage = -units::volt_t{_pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kIntakedAngle:
      _stringStateName = "kIntakedAngle";

      std::cout << "Intake Angle" << std::endl;
      _pidIntakeState.SetSetpoint(0.14);  //-0.55
      _setAlphaArmVoltage = -units::volt_t{_pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kClimbAngle:
      _stringStateName = "kClimbAngle";

      _pidArmStates.SetSetpoint(2);  //-0.48
      _setAlphaArmVoltage = -units::volt_t{_pidArmStates.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;

    case AlphaArmState::kClimbed:
      _stringStateName = "kClimbed";

      _pidClimberStates.SetSetpoint(0.2);  //-0.48
      _setAlphaArmVoltage = -units::volt_t{_pidClimberStates.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kSpeakerAngle:
      _stringStateName = "kSpeakerAngle";

      _pidIntakeState.SetSetpoint(0.4);
      _setAlphaArmVoltage = -units::volt_t{_pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};

      break;
    case AlphaArmState::kBackShooting:
      _pidIntakeState.SetSetpoint(0.33);
      _setAlphaArmVoltage = -units::volt_t{_pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kStowed:
      _stringStateName = "kStowed";

      _pidIntakeState.SetSetpoint(1.2);
      _setAlphaArmVoltage = -units::volt_t{_pidIntakeState.Calculate((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
    default:
      _stringStateName = "INVALID";

      std::cout << "Error: alphaArm in INVALID STATE" << std::endl;
      break;
  }
  _config->alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
  _config->alphaArmGearbox2.motorController->SetVoltage(_setAlphaArmVoltage);
 
  _table->GetEntry("PID Error").SetDouble(_pidArm.GetPositionError());
  _table->GetEntry("SetPoint").SetDouble(_pidArm.GetSetpoint());
  _table->GetEntry("Input").SetDouble((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)));

  _table->GetEntry("PID Error State").SetDouble(_pidArmStates.GetPositionError());
  _table->GetEntry("SetPoint State").SetDouble(_pidArmStates.GetSetpoint());

  _table->GetEntry("Intake SetPoint State").SetDouble(_pidIntakeState.GetSetpoint());
  _table->GetEntry("Intake PID Error State").SetDouble(_pidIntakeState.GetPositionError());

  _table->GetEntry("State Name").SetString(_stringStateName);
}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

void AlphaArm::SetArmRaw(units::volt_t voltage) {
  _rawArmVoltage = voltage;
}

void AlphaArm::SetGoal(double goal) {
  _goal = goal;
}

void AlphaArm::SetControllerRaw(units::volt_t voltage) {
  _controlledRawVoltage = voltage;
}
