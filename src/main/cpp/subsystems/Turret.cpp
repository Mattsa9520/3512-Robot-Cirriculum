// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include "controllers/NormalizeAngle.hpp"

using namespace frc3512;

Turret::Turret(Drivetrain& drivetrain)
    : ControllerSubsystemBase("Turret"), m_drivetrain(drivetrain) {
    m_motor.Set(0);
#ifndef RUNNING_FRC_TESTS
    m_encoder.SetDistancePerRotation(TurretController::kDpR);
#else
    m_encoder.SetDistancePerPulse(TurretController::kDpR);
#endif
    m_lastAngle = units::radian_t{m_encoder.GetDistance()};
    Reset();
}

void Turret::SetVoltage(units::volt_t voltage) { m_motor.SetVoltage(voltage); }

void Turret::ResetEncoder() { m_encoder.Reset(); }

void Turret::Reset() { m_controller.Reset(); }

units::radian_t Turret::GetAngle() const {
    return units::radian_t{-m_encoder.GetDistance()};
}

bool Turret::IsPassedCCWLimit() const {
    return GetAngle() > (wpi::math::pi * 1_rad / 2.0);
}

bool Turret::IsPassedCWLimit() const {
    return GetAngle() < -(wpi::math::pi * 1_rad / 2.0);
}

void Turret::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
}

void Turret::DisableController() { m_controller.Disable(); }

void Turret::ControllerPeriodic() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredOutputs(GetAngle());

    m_controller.SetDrivetrainStatus(m_drivetrain.GetNextXhat());
    m_controller.SetHardLimitOutputs(IsPassedCCWLimit(), IsPassedCWLimit());
    m_controller.Update(now - m_lastTime, now - m_startTime);

    // Set motor input
    SetVoltage(m_controller.ControllerVoltage());

    m_lastTime = now;
}

frc::Pose2d Turret::GetNextPose() const { return m_controller.GetNextPose(); }
