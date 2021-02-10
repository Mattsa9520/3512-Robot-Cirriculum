// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoGalacticSearchB() {
    const frc::Pose2d kInitialPose{0.347_m, 2.801_m, units::radian_t{0}};

    const frc::Pose2d kFirstRedBall{2.38_m, 2.993_m,
                                    units::radian_t{8 * wpi::math::pi / 9}};
    const frc::Pose2d kSecondRedBall{3.901_m, 1.548_m, units::radian_t{0}};
    const frc::Pose2d kThirdRedBall{5.409_m, 3.057_m, units::radian_t{0}};

    const frc::Pose2d kFirstBlueBall{4.655_m, 1.574_m,
                                     units::radian_t{wpi::math::pi / 9}};
    const frc::Pose2d kSecondBlueBall{6.176_m, 3.069_m, units::radian_t{0}};
    const frc::Pose2d kThirdBlueBall{7.697_m, 1.587_m,
                                     units::radian_t{wpi::math::pi / 9}};

    const frc::Pose2d kEndPose{8.733_m, 2.149_m, units::radian_t{0}};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    // TODO: Make this tolerance wider
    if (m_drivetrain.GetUltrasonicDistance() !=
        kFirstRedBall.X() - kInitialPose.X()) {
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kFirstBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kFirstBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kFirstBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kFirstBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory({kInitialPose, kFirstBlueBall,
                                    kSecondBlueBall, kThirdBlueBall, kEndPose},
                                   config);

        m_intake.Deploy();
        m_intake.Start();

        while (!m_drivetrain.AtGoal()) {
            if (!m_autonChooser.Suspend()) {
                return;
            }

            // TODO: Stop intake when drivetrain is outside the region
            // constraints
            m_intake.Stop();
        }
    } else {
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kFirstRedBall.X() - 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kFirstRedBall.X() + 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondRedBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondRedBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdRedBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdRedBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory({kInitialPose, kFirstRedBall, kSecondRedBall,
                                    kThirdRedBall, kEndPose},
                                   config);

        m_intake.Deploy();
        m_intake.Start();

        while (!m_drivetrain.AtGoal()) {
            if (!m_autonChooser.Suspend()) {
                return;
            }

            // TODO: Stop intake when drivetrain is outside the region
            // constraints
            m_intake.Stop();
        }
    }
}
}  // namespace frc3512
