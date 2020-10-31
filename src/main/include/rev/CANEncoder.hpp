// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/CANEncoder.h>

#else

namespace rev {

class CANSparkMax;

class CANEncoder {
public:
    enum class EncoderType {
        kNoSensor = 0,
        kHallSensor = 1,
        kQuadrature = 2,
        kSensorless = 3
    };

    explicit CANEncoder(
        CANSparkMax& device,
        EncoderType sensorType = CANEncoder::EncoderType::kHallSensor,
        int counts_per_rev = 42);

    double GetPosition();
};

}  // namespace rev
#endif
