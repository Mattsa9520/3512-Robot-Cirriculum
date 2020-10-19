// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * The climber composes of an elevator and a traverser.
 *
 * To climb, the elevator reaches for the climbing bar, which can tilt in
 * different directions depending on how many robots are hanging on it. To keep
 * the bar level, the traverser can move left or right to equalize the robot
 * weights on each side of the bar.
 */
class Climber : public SubsystemBase {
public:
    Climber();
    Climber(Climber&&) = default;
    Climber& operator=(Climber&&) = default;

    /**
     *  Sets the traverser speed
     *
     *  @param speed, sets the speed of the traverser.
     */
    void SetTraverser(double speed);

    /**
     *  Sets the elevator speed
     *
     *  @param speed, sets the speed of the elevator.
     */
    void SetElevator(double speed);

    /**
     * Returns the position of the elevator.
     */
    units::meter_t GetElevatorPosition();

    void TeleopPeriodic() override;

private:
    rev::CANSparkMax m_elevator{Constants::Climber::kElevatorPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder m_elevatorEncoder{
        m_elevator, rev::CANEncoder::EncoderType::kHallSensor};

    rev::CANSparkMax m_traverser{frc3512::Constants::Climber::kTraverserPort,
                                 rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_pancake{frc3512::Constants::Climber::kClimberLock};

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_elevatorEncoderEntry =
        m_inst.GetEntry("Diagnostics/Climber/Elevator encoder");
};

}  // namespace frc3512
