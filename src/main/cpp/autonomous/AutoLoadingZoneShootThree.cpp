// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kDriveAwayFromGoal, kShoot, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

// Initial Pose - Right in line with the Loading Zone
static const frc::Pose2d initialPose{12.89_m, 5.662_m,
                                     units::radian_t{wpi::math::pi}};
// End Pose - Drive forward slightly
static const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 5.662_m,
                                 units::radian_t{wpi::math::pi}};

void Robot::AutoLoadingZoneShootThreeInit() {
    m_drivetrain.Reset(initialPose);

    state = State::kDriveAwayFromGoal;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoLoadingZoneShootThreePeriodic() {
    switch (state) {
        case State::kDriveAwayFromGoal: {
            m_drivetrain.SetWaypoints(initialPose, {}, endPose);
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            // Shoot 3x
            if (m_drivetrain.AtGoal()) {
                Shoot();
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            break;
        }
    }

    if constexpr (IsSimulation()) {
        if (autonTimer.HasElapsed(14.97_s)) {
            EXPECT_EQ(State::kIdle, state);
        }
    }
}

}  // namespace frc3512
