// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

class CollisionAvoidanceTest : public frc3512::SimulatorTest {
public:
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Turret turret{vision, drivetrain, flywheel};
    frc3512::Climber climber{turret};
    frc::Notifier controllerPeriodic{[&] {
        turret.TeleopPeriodic();
        turret.ControllerPeriodic();
        climber.TeleopPeriodic();
    }};

    CollisionAvoidanceTest() {
        frc3512::SubsystemBase::RunAllTeleopInit();
        turret.SetControlMode(frc3512::TurretController::ControlMode::kManual);
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);
    }
};

TEST_F(CollisionAvoidanceTest, Avoidance) {
    frc::sim::JoystickSim appendageStick1{
        frc3512::Constants::Robot::kAppendageStick1Port};

    // Set the climber to move to its clockwise limit (in which its in the way
    // of the climber)
    while (!turret.HasPassedCWLimit()) {
        appendageStick1.SetPOV(90);
        frc::sim::StepTiming(20_ms);
    }
    frc::sim::StepTiming(20_ms);
    EXPECT_TRUE(turret.HasPassedCWLimit());
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Set off the trigger button (or button 1) to activate the avoidance code
    // for moving the turret out of the way to its new clockwise limit
    appendageStick1.SetRawButton(1, true);
    frc::sim::StepTiming(20_ms);
    EXPECT_TRUE(turret.AtGoal());
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Move the climber and ensure turret stays at the new CW limit
    appendageStick1.SetY(100);
    EXPECT_GT(climber.GetElevatorPosition(), 0_m);
    appendageStick1.SetPOV(90);
    frc::sim::StepTiming(20_ms);
    EXPECT_TRUE(turret.HasPassedCWLimit());
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);
}
