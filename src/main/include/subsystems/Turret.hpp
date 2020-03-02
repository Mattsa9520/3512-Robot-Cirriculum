// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/RTNotifier.h>
#include <frc/Timer.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 *  State class which stores numerical values as "states" for the Turret to use.
 */
enum TurretState { kIDLE = 0, kMostLeft, KMostRight };

/**
 * Subsystem specifically designed for the Turret (bottom, movable part of the
 * Shooter)
 */
class Turret : public ControllerSubsystemBase {
public:
    explicit Turret(Drivetrain& drivetrain);

    Turret(const Turret&) = delete;
    Turret& operator=(const Turret&) = delete;

    /**
     *  Set the velocity of the Spark Max, which is wired to the Turret.
     *
     *  Can be seen as a way of having it turn in a certain direction.
     *  This depends on the value given to it.
     *
     *  @param velocity between [-1.0 ... 1.0]
     */
    void SetVoltage(units::volt_t voltage);

    /**
     *  Resets encoder
     */
    void ResetEncoder();

    /**
     * Resets the controller
     */
    void Reset();

    /**
     * Returns the angle of the turret
     *
     * @return angle in radians
     */
    units::radian_t GetAngle() const;

    /**
     * Returns if encoder is passed the counter-clockwise limit
     *
     * @return 'true' means triggered
     */
    bool IsPassedCCWLimit() const;

    /**
     * Returns if encoder is passed the clockwise limit
     *
     * @return 'true' means triggered
     */
    bool IsPassedCWLimit() const;

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

    void ControllerPeriodic() override;

    void DisabledInit() override { DisableController(); }

    void AutonomousInit() override { EnableController(); }

    void TeleopInit() override { EnableController(); }

    frc::Pose2d GetNextPose() const;

private:
#ifndef RUNNING_FRC_TESTS
    frc::DutyCycleEncoder m_encoder{Constants::Turret::kEncoderPort};
#else
    frc::Encoder m_encoder{0, 1};
#endif
    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::now();

    units::radian_t m_lastAngle;
    std::chrono::steady_clock::time_point m_timeSinceLastAngle =
        std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::now();
    TurretController m_controller;

    Drivetrain& m_drivetrain;
};
}  // namespace frc3512
