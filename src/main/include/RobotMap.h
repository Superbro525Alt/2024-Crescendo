// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/length.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "AlphaArm.h"
#include "AlphaArmBehaviour.h"
#include "Intake.h"
#include "Shooter.h"
#include "Wombat.h"
#include "utils/Encoder.h"
#include "utils/PID.h"
#include "utils/Pathplanner.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver = frc::XboxController(0);
    frc::XboxController codriver = frc::XboxController(1);
    frc::XboxController testController = frc::XboxController(2);
  };
  Controllers controllers;

  // struct AlphaArmSystem {
  //   rev::CANSparkMax alphaArmMotor{21, rev::CANSparkMax::MotorType::kBrushless};
  //   rev::CANSparkMax wristMotor{26, rev::CANSparkMax::MotorType::kBrushless};
  //   wom::CANSparkMaxEncoder* alphaArmEncoder = new wom::CANSparkMaxEncoder(&alphaArmMotor, 0.1_m);
  //   wom::Gearbox alphaArmGearbox{&alphaArmMotor, nullptr, frc::DCMotor::NEO(1)};
  //   wom::Gearbox wristGearbox{&wristMotor, alphaArmEncoder, frc::DCMotor::NEO(1)};
  //
  //   AlphaArmConfig config{alphaArmGearbox, wristGearbox};
  // };
  // AlphaArmSystem alphaArmSystem;

  struct IntakeSystem {
    rev::CANSparkMax intakeMotor{31, rev::CANSparkMax::MotorType::kBrushless};
    // wom::CANSparkMaxEncoder intakeEncoder{&intakeMotor, 0.1_m};
    frc::DigitalInput intakeSensor{4};
    // frc::DigitalInput magSensor{0};
    // frc::DigitalInput shooterSensor{0};

    wom::Gearbox IntakeGearbox{&intakeMotor, nullptr, frc::DCMotor::NEO(1)};

    IntakeConfig config{IntakeGearbox, &intakeSensor /*, &magSensor, &shooterSensor*/};
  };
  IntakeSystem intakeSystem;

  struct Shooter {
    rev::CANSparkMax shooterMotor{35, rev::CANSparkMax::MotorType::kBrushless};  // Port 11
    // frc::DigitalInput shooterSensor{2};

    // wom::VoltageController shooterMotorGroup =
    // wom::VoltagedController::Group(shooterMotor);
    wom::CANSparkMaxEncoder* shooterEncoder = new wom::CANSparkMaxEncoder(&shooterMotor, 0.01_m);
    wom::Gearbox shooterGearbox{&shooterMotor, shooterEncoder, frc::DCMotor::NEO(1)};

    wom::utils::PIDConfig<units::radians_per_second, units::volts> pidConfigS{
        "/armavator/arm/velocityPID/config",
        0.1_V / (360_deg / 1_s),
        0.03_V / 25_deg,
        0.001_V / (90_deg / 1_s / 1_s),
        5_rad_per_s,
        10_rad_per_s / 1_s};

    ShooterConfig config{"shooterGearbox", shooterGearbox, pidConfigS};
  };
  Shooter shooterSystem;

  //   struct AlphaArmSystem {
  //     rev::CANSparkMax alphaArmMotor{12, rev::CANSparkMax::MotorType::kBrushless};
  //     wom::CANSparkMaxEncoder* armEncoder = new wom::CANSparkMaxEncoder(&alphaArmMotor, 0.02_m);

  //     wom::Gearbox alphaArmGearbox{&alphaArmMotor, armEncoder, frc::DCMotor::NEO(1)};

  //     wom::utils::PIDConfig<units::radian, units::volt> pidConfigA{
  //      "/path/to/pid/in/nt/tables",
  //     15_V / 180_deg,
  //     0_V / (1_deg * 1_s),
  //     0_V / (1_deg / 1_s),
  //     };

  //     AlphaArmConfig config {
  //         alphaArmGearbox,
  //         pidConfigA,
  //     };

  //   };
  //   AlphaArmSystem alphaArmSystem;

  //   struct IntakeSystem {
  //     rev::CANSparkMax intakeMotor{2, rev::CANSparkMax::MotorType::kBrushed};
  //     // wom::CANSparkMaxEncoder intakeEncoder{&intakeMotor, 0.1_m};
  //     frc::DigitalInput intakeSensor{4};
  //     // frc::DigitalInput magSensor{0};
  //     // frc::DigitalInput shooterSensor{0};

  //     wom::Gearbox IntakeGearbox{&intakeMotor, nullptr, frc::DCMotor::CIM(1)};

  //     IntakeConfig config{IntakeGearbox, &intakeSensor /*, &magSensor, &shooterSensor*/};
  //   };
  //   IntakeSystem intakeSystem;

  //     struct Shooter {
  //     rev::CANSparkMax shooterMotor{11, rev::CANSparkMax::MotorType::kBrushless};// Port 11
  //     // frc::DigitalInput shooterSensor{2};

  //     // wom::VoltageController shooterMotorGroup = wom::VoltagedController::Group(shooterMotor);
  //     wom::CANSparkMaxEncoder* shooterEncoder = new wom::CANSparkMaxEncoder(&shooterMotor, 0.01_m);
  //     wom::Gearbox shooterGearbox{&shooterMotor, shooterEncoder, frc::DCMotor::NEO(1)};

  //     wom::utils::PIDConfig<units::radians_per_second, units::volts> pidConfigS{
  //           "/armavator/arm/velocityPID/config",
  //           0.1_V / (360_deg / 1_s),
  //           0.03_V / 25_deg,
  //           0.001_V / (90_deg / 1_s / 1_s),
  //           5_rad_per_s,
  //           10_rad_per_s / 1_s
  //     };

  //     ShooterConfig config{
  //         "shooterGearbox",
  //         shooterGearbox,
  //         pidConfigS
  //     };

  //   };
  //   Shooter shooterSystem;
  //
  // struct Arm {
  //   // creates the motor used for the arm as well as the port it is plugged in
  //   rev::CANSparkMax leftArmMotor{21, rev::CANSparkMax::MotorType::kBrushless};   // 11
  //   rev::CANSparkMax rightArmMotor{26, rev::CANSparkMax::MotorType::kBrushless};  // 12
  //
  //   // rev::CANSparkMax leftPretendArmMotor{28, rev::CANSparkMax::MotorType::kBrushless};
  //   // rev::CANSparkMax rightPretendArmMotor{29, rev::CANSparkMax::MotorType::kBrushless};
  //
  //   // wom::DigitalEncoder encoder{0, 1, 2048};
  //   // sets the type sof encoder that is used up
  //   wom::CANSparkMaxEncoder* leftEncoder = new wom::CANSparkMaxEncoder(&leftArmMotor, 10_cm);
  //   wom::CANSparkMaxEncoder* rightEncoder = new wom::CANSparkMaxEncoder(&rightArmMotor, 10_cm);
  //
  //   // wom::DutyCycleEncoder *encoder = new wom::DutyCycleEncoder(3, 10_cm);
  //
  //   // wom::CANSparkMaxEncoder leftEncoder{&leftArmMotor, 100};
  //   // wom::CANSparkMaxEncoder rightEncoder{&rightArmMotor, 100};
  //
  //   // rev::SparkMaxAbsoluteEncoder leftOtherArmEncoder{leftArmMotor,
  //   // rev::CANSparkMax::SparkMaxAbsoluteEncoder::Type::kDutyCycle}; wom::CANSparkMaxEncoder
  //   // leftOtherArmEncoder = leftArmMotor.GetEncoder(); wom::CANSparkMaxEncoder rightOtherArmEncoder =
  //   // rightArmMotor.GetEncoder();
  //
  //   // creates an instance of the arm gearbox
  //   wom::Gearbox leftGearbox{&leftArmMotor, leftEncoder,
  //                            // nullptr,
  //                            frc::DCMotor::NEO(1).WithReduction(100)};
  //
  //   wom::Gearbox rightGearbox{&rightArmMotor, rightEncoder,
  //                             // nullptr,
  //                             frc::DCMotor::NEO(1).WithReduction(100)};
  //
  //   // creates arm config information
  //   wom::ArmConfig config{
  //       "/armavator/arm", leftGearbox, rightGearbox,
  //       // nullptr,
  //       leftEncoder,
  //       wom::PIDConfig<units::radian, units::volts>("/armavator/arm/pid/config",
  //                                                   18_V / 25_deg,        // prev 13_V/25_deg
  //                                                   0_V / (1_deg * 1_s),  // 0.1_V / (1_deg * 1_s)
  //                                                   0_V / (1_deg / 1_s),  // 0_V / (1_deg / 1_s)
  //                                                   5_deg, 2_deg / 1_s, 10_deg),
  //       wom::PIDConfig<units::radians_per_second, units::volts>("/armavator/arm/velocityPID/config",
  //                                                               9_V / (180_deg / 1_s), 0_V / 25_deg,
  //                                                               0_V / (90_deg / 1_s / 1_s)),
  //       1_kg, 0.5_kg, 1.15_m, -90_deg, 270_deg, 0_deg};
  //
  //   Arm() {
  //     // inverts the motor so that it goes in the right direction while using RAW controlls
  //     leftArmMotor.SetInverted(true);
  //     rightArmMotor.SetInverted(false);
  //   }
  // };
  // Arm arm;
  struct SwerveBase {
    ctre::phoenix6::hardware::CANcoder frontLeftCancoder{16, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder frontRightCancoder{18, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder backLeftCancoder{17, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder backRightCancoder{19, "Drivebase"};

    ctre::phoenix6::hardware::Pigeon2* gyro = new ctre::phoenix6::hardware::Pigeon2(20, "Drivebase");
    wpi::array<ctre::phoenix6::hardware::TalonFX*, 4> turnMotors{
        new ctre::phoenix6::hardware::TalonFX(6, "Drivebase"),   // front left
        new ctre::phoenix6::hardware::TalonFX(7, "Drivebase"),   // front right
        new ctre::phoenix6::hardware::TalonFX(4, "Drivebase"),   // back left
        new ctre::phoenix6::hardware::TalonFX(2, "Drivebase")};  // back right
    wpi::array<ctre::phoenix6::hardware::TalonFX*, 4> driveMotors{
        new ctre::phoenix6::hardware::TalonFX(5, "Drivebase"),   // front left
        new ctre::phoenix6::hardware::TalonFX(9, "Drivebase"),   // front right
        new ctre::phoenix6::hardware::TalonFX(3, "Drivebase"),   // back left
        new ctre::phoenix6::hardware::TalonFX(1, "Drivebase")};  // back right

    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
        wom::SwerveModuleConfig{
            // CORRECT
            // front left module
            frc::Translation2d(-10_in, 9_in),
            wom::Gearbox{driveMotors[0], new wom::TalonFXEncoder(driveMotors[0], 0.0445_m, 6.75),
                         frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[0], new wom::CanEncoder(16, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &frontLeftCancoder, 4_in / 2},
        wom::SwerveModuleConfig{
            // CORRECT
            // front right module
            frc::Translation2d(10_in, 9_in),
            wom::Gearbox{driveMotors[1], new wom::TalonFXEncoder(driveMotors[1], 0.0445_m, 6.75),
                         frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[1], new wom::CanEncoder(18, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &frontRightCancoder, 4_in / 2},
        wom::SwerveModuleConfig{
            // back left module
            frc::Translation2d(-10_in, 9_in),
            wom::Gearbox{driveMotors[2], new wom::TalonFXEncoder(driveMotors[2], 0.0445_m, 6.75),
                         frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[2], new wom::CanEncoder(17, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &backRightCancoder, 4_in / 2},
        wom::SwerveModuleConfig{
            // back right module
            frc::Translation2d(-10_in, -9_in),
            wom::Gearbox{driveMotors[3], new wom::TalonFXEncoder(driveMotors[3], 0.0445_m, 6.75),
                         frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[3], new wom::CanEncoder(19, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &backLeftCancoder, 4_in / 2},
    };

    // Setting the PID path and values to be used for SwerveDrive and
    // SwerveModules
    /*wom::SwerveModule::angle_pid_conf_t anglePID{
        "/drivetrain/pid/angle/config", 90_V / 360_deg, 0.0_V / (100_deg * 1_s),
        0_V / (100_deg / 1_s)};*/
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
        "/drivetrain/pid/velocity/config",
        12_V / 4_mps  // webers per metre
    };
    /*wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID{
        "/drivetrain/pid/pose/angle/config",
        0_deg / 1_s / 45_deg,
        wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0},
        0_deg / 1_deg};*/
    // wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
    //   "/drivetrain/pid/pose/position/config",
    //   20_mps / 1_m,
    //   wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0},//wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0.15},
    //   0,//0_m / 1_m,
    //   0.1_m,
    //   0.1_m / 1_s};
    wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
        "/drivetrain/pid/pose/position/config", 0_mps / 1_m,
        wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0.15}, 0_m / 1_m, 0_cm};

    // the config for the whole swerve drive
    wom::SwerveDriveConfig config{"/drivetrain",
                                  // anglePID,
                                  velocityPID,
                                  moduleConfigs,  // each module
                                  gyro,
                                  // poseAnglePID,
                                  // posePositionPID,
                                  //  poseAnglePID,
                                  // posePositionPID,
                                  60_kg,  // robot mass (estimate rn)
                                  {0.1, 0.1, 0.1},
                                  {0.9, 0.9, 0.9}};

    // current limiting and setting idle mode of modules to brake mode
    // SwerveBase() {
    //  for (size_t i = 0; i < 4; i++) {
    //    turnMotors[i]->ConfigSupplyCurrentLimit(
    //        SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    //    driveMotors[i]->SetNeutralMode(NeutralMode::Brake);
    //    turnMotors[i]->SetNeutralMode(NeutralMode::Brake);
    //    driveMotors[i]->SetInverted(true);
    //  }
    //}
  };

  SwerveBase swerveBase;

  struct SwerveTable {
    std::shared_ptr<nt::NetworkTable> swerveDriveTable =
        nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  };
  SwerveTable swerveTable;

  // struct AlphaArmSystem {
  //   rev::CANSparkMax alphaArmMotor{12, rev::CANSparkMax::MotorType::kBrushless};
  //   rev::CANSparkMax wristMotor{15, rev::CANSparkMax::MotorType::kBrushless};

  //   wom::Gearbox alphaArmGearbox{&alphaArmMotor, nullptr, frc::DCMotor::NEO(1)};
  //   wom::Gearbox wristGearbox{&wristMotor, nullptr, frc::DCMotor::NEO(1)};

  //   AlphaArmConfig config{alphaArmGearbox, wristGearbox};
  // };
  // AlphaArmSystem alphaArmSystem;
  wom::SwerveAutoBuilder* builder;
  wom::SimSwerve* simSwerve;
};
