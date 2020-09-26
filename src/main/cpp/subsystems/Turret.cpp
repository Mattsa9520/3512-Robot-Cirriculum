// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Vision.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Turret::Turret(Vision& vision, Drivetrain& drivetrain)
    : m_vision(vision), m_drivetrain(drivetrain) {
    SetCANSparkMaxBusUsage(m_motor, Usage::kMinimal);

    m_encoder.SetDistancePerRotation(TurretController::kDpR);
    Reset();
}

void Turret::SetDirection(Direction direction) {
    if (m_manualOverride) {
        if (direction == Direction::kCW) {
            SetVoltage(-7_V);
        } else if (direction == Direction::kCCW) {
            SetVoltage(7_V);
        } else {
            SetVoltage(0_V);
        }
    }
}

void Turret::SetManualOverride() { m_manualOverride = true; }

void Turret::Reset(units::radian_t initialHeading) {
    m_controller.Reset();

    if constexpr (frc::RobotBase::IsSimulation()) {
        Eigen::Matrix<double, 2, 1> x = Eigen::Matrix<double, 2, 1>::Zero();
        m_turretSim.SetState(x);

        m_encoderSimPosition.Set(HeadingToEncoderTurns(initialHeading));
    }
}

units::radian_t Turret::GetAngle() const {
    // Negative sign makes encoder counterclockwise positive. An offset is added
    // to make the absolute encoder return 0 rad when it's facing forward.
    return units::radian_t{-m_encoder.GetDistance()} + kOffset;
}

bool Turret::HasPassedCCWLimit() const {
    return GetAngle() > TurretController::kCCWLimit;
}

bool Turret::HasPassedCWLimit() const {
    return GetAngle() < TurretController::kCWLimit;
}

frc::Pose2d Turret::GetNextPose() const { return m_controller.GetNextPose(); }

void Turret::EnableController() {
    m_lastTime = frc2::Timer::GetFPGATimestamp();
    m_controller.Enable();
}

void Turret::DisableController() { m_controller.Disable(); }

bool Turret::AtGoal() const { return m_controller.AtGoal(); }

units::volt_t Turret::GetMotorOutput() const {
    return m_motor.Get() *
           units::volt_t{frc::RobotController::GetInputVoltage()};
}

void Turret::ControllerPeriodic() {
    auto now = frc2::Timer::GetFPGATimestamp();
    m_controller.SetMeasuredOutputs(GetAngle());

    m_controller.SetDrivetrainStatus(m_drivetrain.GetNextXhat());
    m_controller.Update(now - m_lastTime, now - GetStartTime());

    auto globalMeasurement = m_vision.GetGlobalMeasurement();
    if (globalMeasurement) {
        auto turretInGlobal = globalMeasurement.value();

        frc::Transform2d turretInGlobalToDrivetrainInGlobal{
            frc::Pose2d{
                TurretController::kDrivetrainToTurretFrame.Translation(),
                TurretController::kDrivetrainToTurretFrame.Rotation()
                        .Radians() +
                    GetAngle()},
            frc::Pose2d{}};
        auto drivetrainInGlobal =
            turretInGlobal.pose.TransformBy(turretInGlobalToDrivetrainInGlobal);

        m_drivetrain.CorrectWithGlobalOutputs(
            drivetrainInGlobal.X(), drivetrainInGlobal.Y(),
            globalMeasurement.value().timestamp);
    }

    // Set motor input
    if (!m_manualOverride) {
        auto u = m_controller.GetInputs();
        SetVoltage(units::volt_t{u(TurretController::Input::kVoltage)});
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_turretSim.SetInput(frc::MakeMatrix<1, 1>(
            m_motor.Get() * frc::RobotController::GetInputVoltage()));

        m_turretSim.Update(now - m_lastTime);

        m_encoderSimPosition.Set(
            HeadingToEncoderTurns(units::radian_t{m_turretSim.Y(0)}));
    }

    m_lastTime = now;
}

void Turret::TeleopPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};

    // Turret manual override
    if (appendageStick1.GetRawButtonPressed(11)) {
        SetManualOverride();
    }

    // Turrret manual spin
    int pov = appendageStick1.GetPOV();
    if (pov == 90) {
        SetDirection(Direction::kCW);
    } else if (pov == 270) {
        SetDirection(Direction::kCCW);
    } else {
        SetDirection(Direction::kNone);
    }
}

void Turret::SetVoltage(units::volt_t voltage) {
    if (voltage < 0_V && !HasPassedCWLimit()) {
        m_motor.SetVoltage(voltage);
    } else if (voltage > 0_V && !HasPassedCCWLimit()) {
        m_motor.SetVoltage(voltage);
    } else {
        m_motor.SetVoltage(0_V);
    }
}

double Turret::HeadingToEncoderTurns(units::radian_t heading) {
    // heading = -GetDistance() + kOffset
    // heading = -(Get() * kDpR) + kOffset
    // heading - kOffset = -Get() * kDpR
    // (heading - kOffset) / kDpR = -Get()
    // Get() = (kOffset - heading) / kDpR
    return (kOffset - heading).to<double>() / TurretController::kDpR;
}
