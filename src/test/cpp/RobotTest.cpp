// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/RobotBase.h>
#include <gtest/gtest.h>

#include "Robot.hpp"

// Make sure robot initializes
TEST(RobotTest, Init) {
    frc::RunHALInitialization();
    frc3512::Robot robot;
}
