// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <fmt/core.h>
#include <frc/MathUtil.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h>

#include "EigenFormat.hpp"

using namespace frc3512;
using namespace frc3512::Constants;

const frc::LinearSystem<2, 2, 2> DrivetrainController::kPlant{GetPlant()};

DrivetrainController::DrivetrainController() {
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(frc::Pose2d{0_m, 0_m, units::radian_t{wpi::math::pi}});

    m_B = JacobianU(Eigen::Matrix<double, 2, 1>::Zero());

    m_trajectoryTimeElapsed.Start();
}

void DrivetrainController::AddTrajectory(
    const frc::Pose2d& start, const std::vector<frc::Translation2d>& interior,
    const frc::Pose2d& end, const frc::TrajectoryConfig& config) {
    m_trajectories.emplace_back(frc::TrajectoryGenerator::GenerateTrajectory(
        start, interior, end, config));
    m_goal = m_trajectories.front().States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (m_trajectories.size() == 1) {
        m_trajectoryTimeElapsed.Reset();
    }
}

void DrivetrainController::AddTrajectory(
    const std::vector<frc::Pose2d>& waypoints,
    const frc::TrajectoryConfig& config) {
    m_trajectories.emplace_back(
        frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config));
    m_goal = m_trajectories.front().States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (m_trajectories.size() == 1) {
        m_trajectoryTimeElapsed.Reset();
    }
}

bool DrivetrainController::HaveTrajectory() const {
    return m_trajectories.size() > 0;
}

void DrivetrainController::AbortTrajectories() { m_trajectories.reset(); }

bool DrivetrainController::AtGoal() const {
    frc::Pose2d ref{units::meter_t{m_r(State::kX)},
                    units::meter_t{m_r(State::kY)},
                    units::radian_t{m_r(State::kHeading)}};
    return m_goal == ref && m_atReferences;
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    Eigen::Matrix<double, 7, 1> xHat;
    xHat(0) = initialPose.X().to<double>();
    xHat(1) = initialPose.Y().to<double>();
    xHat(2) = initialPose.Rotation().Radians().to<double>();
    xHat.block<4, 1>(3, 0).setZero();

    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;
    m_goal = initialPose;

    UpdateAtReferences(Eigen::Matrix<double, 5, 1>::Zero());
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Calculate(
    const Eigen::Matrix<double, 7, 1>& x) {
    m_u << 0.0, 0.0;

    if (HaveTrajectory()) {
        frc::Trajectory::State ref =
            m_trajectories.front().Sample(m_trajectoryTimeElapsed.Get());
        auto [vlRef, vrRef] =
            ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

        m_nextR << ref.pose.X().to<double>(), ref.pose.Y().to<double>(),
            ref.pose.Rotation().Radians().to<double>(), vlRef.to<double>(),
            vrRef.to<double>(), 0, 0;

        Eigen::Matrix<double, 2, 1> u_fb = Controller(x, m_r);
        u_fb = frc::NormalizeInputVector<2>(u_fb, 12.0);
        m_u = u_fb + m_ff.Calculate(m_nextR);
        m_u = frc::NormalizeInputVector<2>(m_u, 12.0);

        Eigen::Matrix<double, 5, 1> error =
            m_nextR.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
        error(State::kHeading) =
            frc::AngleModulus(units::radian_t{error(State::kHeading)})
                .to<double>();
        UpdateAtReferences(error);

        m_r = m_nextR;
    }

    if (AtGoal() && HaveTrajectory()) {
        m_trajectories.pop_front();
        m_trajectoryTimeElapsed.Reset();

        if (HaveTrajectory()) {
            m_goal = m_trajectories.front().States().back().pose;
        }
    }

    return m_u;
}

frc::LinearSystem<2, 2, 2> DrivetrainController::GetPlant() {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(kLinearV, kLinearA,
                                                         kAngularV, kAngularA);
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig() {
    frc::TrajectoryConfig config{kMaxV, kMaxA - 14.5_mps_sq};

    frc::DifferentialDriveKinematics kinematics{kWidth};
    frc::DifferentialDriveVelocitySystemConstraint systemConstraint{
        kPlant, kinematics, 8_V};
    config.AddConstraint(systemConstraint);

    return config;
}

Eigen::Matrix<double, 2, 5> DrivetrainController::ControllerGainForState(
    const Eigen::Matrix<double, 7, 1>& x) {
    // Make the heading zero because the LTV controller controls forward error
    // and cross-track error
    Eigen::Matrix<double, 7, 1> x0 = x;
    x0(State::kHeading) = 0.0;

    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    double velocity =
        (x0(State::kLeftVelocity) + x0(State::kRightVelocity)) / 2.0;
    if (std::abs(velocity) < 1e-4) {
        return Eigen::Matrix<double, 2, 5>::Zero();
    }

    return frc::LinearQuadraticRegulator<5, 2>(JacobianX(x0), m_B, kControllerQ,
                                               kControllerR, kDt)
        .K();
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 7, 1>& r) {
    // This implements the linear time-varying differential drive controller in
    // theorem 8.6.4 of https://tavsys.net/controls-in-frc.
    Eigen::Matrix<double, 2, 5> K;

    try {
        K = ControllerGainForState(x);
    } catch (const std::runtime_error& e) {
        fmt::print(stderr, "{}\n", e.what());

        Eigen::Matrix<double, 7, 1> x0 = x;
        x0(State::kHeading) = 0.0;

        // The DARE is ill-conditioned if the velocity is close to zero, so
        // don't let the system stop.
        double velocity =
            (x0(State::kLeftVelocity) + x0(State::kRightVelocity)) / 2.0;
        if (std::abs(velocity) < 1e-3) {
            x0(State::kLeftVelocity) += 1e-3;
            x0(State::kRightVelocity) += 1e-3;
        }

        Eigen::Matrix<double, 5, 5> discA;
        Eigen::Matrix<double, 5, 2> discB;
        frc::DiscretizeAB<5, 2>(JacobianX(x0), m_B, kDt, &discA, &discB);
        fmt::print(stderr, "A = {}\n", discA);
        fmt::print(stderr, "B = {}\n", discB);
    }

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(2));
    inRobotFrame(0, 1) = std::sin(x(2));
    inRobotFrame(1, 0) = -std::sin(x(2));
    inRobotFrame(1, 1) = std::cos(x(2));

    Eigen::Matrix<double, 5, 1> error =
        r.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
    error(State::kHeading) =
        frc::AngleModulus(units::radian_t{error(State::kHeading)}).to<double>();
    return K * inRobotFrame * error;
}

Eigen::Matrix<double, 7, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = kPlant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 4> A;
    A.block<2, 2>(0, 0) = kPlant.A();
    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();

    double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

    Eigen::Matrix<double, 7, 1> xdot;
    xdot(0) = v * std::cos(x(State::kHeading));
    xdot(1) = v * std::sin(x(State::kHeading));
    xdot(2) = ((x(State::kRightVelocity) - x(State::kLeftVelocity)) / kWidth)
                  .to<double>();
    xdot.block<4, 1>(3, 0) = A * x.block<4, 1>(3, 0) + B * u;
    return xdot;
}

Eigen::Matrix<double, 5, 5> DrivetrainController::JacobianX(
    const Eigen::Matrix<double, 7, 1>& x) {
    // clang-format off
    return frc::MakeMatrix<5, 5>(
        0.0, 0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -1.0 / kWidth.to<double>(), 1.0 / kWidth.to<double>(),
        0.0, 0.0, 0.0, kPlant.A(0, 0), kPlant.A(0, 1),
        0.0, 0.0, 0.0, kPlant.A(1, 0), kPlant.A(1, 1));
    // clang-format on
}

Eigen::Matrix<double, 5, 2> DrivetrainController::JacobianU(
    const Eigen::Matrix<double, 2, 1>& u) {
    // clang-format off
    return frc::MakeMatrix<5, 2>(
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        kPlant.B(0, 0), kPlant.B(0, 1),
        kPlant.B(1, 0), kPlant.B(1, 1));
    // clang-format on
}

Eigen::Matrix<double, 3, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 3, 1> y;
    y << x(State::kHeading), x(State::kLeftPosition), x(State::kRightPosition);
    return y;
}

Eigen::Matrix<double, 2, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 2, 1> y;
    y << x(State::kX), x(State::kY);
    return y;
}

void DrivetrainController::UpdateAtReferences(
    const Eigen::Matrix<double, 5, 1>& error) {
    m_atReferences = std::abs(error(0)) < kPositionTolerance &&
                     std::abs(error(1)) < kPositionTolerance &&
                     std::abs(error(2)) < kAngleTolerance &&
                     std::abs(error(3)) < kVelocityTolerance &&
                     std::abs(error(4)) < kVelocityTolerance;
}
