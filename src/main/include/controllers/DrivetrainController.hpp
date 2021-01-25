// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <functional>
#include <tuple>
#include <vector>

#include <frc/controller/ControlAffinePlantInversionFeedforward.h>
#include <frc/estimator/AngleStatistics.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/Timer.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/math>
#include <wpi/static_circular_buffer.h>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class DrivetrainController : public ControllerBase<7, 2, 3> {
public:
    static constexpr units::meter_t kWheelRadius = 3_in;
    static constexpr double kDriveGearRatio = 1.0 / 1.0;
    static constexpr double kDpP =
        (2.0 * wpi::math::pi * kWheelRadius.to<double>()) * kDriveGearRatio /
        2048.0;
    static constexpr units::meter_t kWidth = 0.990405073902434_m;

    static constexpr std::array<double, 5> kControllerQ{0.0625, 0.125, 2.5,
                                                        0.95, 0.95};
    static constexpr std::array<double, 2> kControllerR{12.0, 12.0};

    /**
     * States of the drivetrain system.
     */
    class State {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftVelocity = 3;
        static constexpr int kRightVelocity = 4;
        static constexpr int kLeftPosition = 5;
        static constexpr int kRightPosition = 6;
    };

    /**
     * Inputs of the drivetrain system.
     */
    class Input {
    public:
        static constexpr int kLeftVoltage = 0;
        static constexpr int kRightVoltage = 1;
    };

    /**
     * Local outputs of the drivetrain system.
     */
    class LocalOutput {
    public:
        static constexpr int kHeading = 0;
        static constexpr int kLeftPosition = 1;
        static constexpr int kRightPosition = 2;
    };

    /**
     * Global outputs of the drivetrain system.
     */
    class GlobalOutput {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
    };

    /**
     * Constructs a drivetrain controller.
     */
    DrivetrainController();

    DrivetrainController(DrivetrainController&&) = default;
    DrivetrainController& operator=(DrivetrainController&&) = default;

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    void AddTrajectory(
        const frc::Pose2d& start,
        const std::vector<frc::Translation2d>& interior, const frc::Pose2d& end,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void AddTrajectory(
        const std::vector<frc::Pose2d>& waypoints,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

    /**
     * Abort trajectory tracking.
     */
    void AbortTrajectories();

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    void Predict(const Eigen::Matrix<double, 2, 1>& u, units::second_t dt);

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::second_t timestamp);

    const Eigen::Matrix<double, 7, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 7, 1>& GetStates() const override;

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const frc::Pose2d& initialPose);

    Eigen::Matrix<double, 2, 1> Update(const Eigen::Matrix<double, 3, 1>& y,
                                       units::second_t dt) override;

    /**
     * Returns the drivetrain's plant.
     */
    frc::LinearSystem<2, 2, 2> GetPlant() const;

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns the linear time-varying controller gain for the given state.
     *
     * @param x The state vector.
     */
    Eigen::Matrix<double, 2, 5> ControllerGainForState(
        const Eigen::Matrix<double, 7, 1>& x);

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 7, 1>& r);

    static Eigen::Matrix<double, 7, 1> Dynamics(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 2, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

private:
    static constexpr double kPositionTolerance = 0.5;  // meters
    static constexpr double kVelocityTolerance = 2.0;  // meters/second
    static constexpr double kAngleTolerance = 0.52;    // radians
    static constexpr auto kLinearV = 3.02_V / 1_mps;
    static constexpr auto kLinearA = 0.642_V / 1_mps_sq;
    static constexpr auto kAngularV = 1.382_V / 1_rad_per_s;
    static constexpr auto kAngularA = 0.08495_V / 1_rad_per_s_sq;
    static constexpr auto kMaxV = 12_V / kLinearV;
    static constexpr auto kMaxA = 12_V / kLinearA;

    // TODO: Find a good measurement covariance for global measurements
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    // Design observer. See the enums above for lists of the states, inputs, and
    // outputs.
    frc::UnscentedKalmanFilter<7, 2, 3> m_observer{
        Dynamics,
        LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5},
        {0.0001, 0.005, 0.005},
        frc::AngleMean<7, 7>(2),
        frc::AngleMean<3, 7>(0),
        frc::AngleResidual<7>(2),
        frc::AngleResidual<3>(0),
        frc::AngleAdd<7>(2),
        Constants::kDt};
    frc::KalmanFilterLatencyCompensator<7, 2, 3,
                                        frc::UnscentedKalmanFilter<7, 2, 3>>
        m_latencyComp;

    frc::ControlAffinePlantInversionFeedforward<7, 2> m_ff{Dynamics,
                                                           Constants::kDt};

    Eigen::Matrix<double, 5, 2> m_B;

    Eigen::Matrix<double, 2, 5> m_K = Eigen::Matrix<double, 2, 5>::Zero();

    // Controller reference
    Eigen::Matrix<double, 7, 1> m_r;
    Eigen::Matrix<double, 7, 1> m_nextR;

    wpi::static_circular_buffer<frc::Trajectory, 8> m_trajectories;
    frc::Pose2d m_goal;
    frc2::Timer m_trajectoryTimeElapsed;

    bool m_atReferences = false;

    /**
     * Converts velocity and curvature of drivetrain into left and right wheel
     * velocities.
     *
     * @param velocity Linear velocity of drivetrain chassis.
     * @param curvature Curvature of drivetrain arc.
     * @param trackWidth Track width of drivetrain.
     */
    static constexpr std::tuple<units::meters_per_second_t,
                                units::meters_per_second_t>
    ToWheelVelocities(units::meters_per_second_t velocity,
                      units::curvature_t curvature, units::meter_t trackWidth) {
        // clang-format off
        // v = (v_r + v_l) / 2     (1)
        // w = (v_r - v_l) / (2r)  (2)
        // k = w / v               (3)
        //
        // v_l = v - wr
        // v_l = v - (vk)r
        // v_l = v(1 - kr)
        //
        // v_r = v + wr
        // v_r = v + (vk)r
        // v_r = v(1 + kr)
        // clang-format on
        auto vl = velocity * (1 - (curvature / 1_rad * trackWidth / 2.0));
        auto vr = velocity * (1 + (curvature / 1_rad * trackWidth / 2.0));
        return {vl, vr};
    }
};
}  // namespace frc3512
