// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLoadingZoneShootThree() {
    // Initial Pose - Right in line with the Loading Zone
    const frc::Pose2d initialPose{12.89_m, 5.662_m,
                                  units::radian_t{wpi::math::pi}};
    // End Pose - Drive forward slightly
    const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 5.662_m,
                              units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(initialPose);
    m_drivetrain.SetWaypoints(initialPose, {}, endPose);

    while (!m_drivetrain.AtGoal()) {
        m_autonChooser.Yield();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    // Shoot 3x
    Shoot();

    while (IsShooting()) {
        m_autonChooser.Yield();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }
}

}  // namespace frc3512
