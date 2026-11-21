/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CANCoderConfig {
    private final CANcoderConfiguration configuration = new CANcoderConfiguration();
    private final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    public void configure(CANcoder encoder) {
        CANcoderConfiguration defaultConfig = new CANcoderConfiguration();
        encoder.getConfigurator().apply(defaultConfig);

        encoder.getConfigurator().apply(configuration);
    }

    public CANcoderConfiguration getConfiguration() {
        return this.configuration;
    }

    // MAGNET SENSOR CONFIGS

    public CANCoderConfig withSensorDirection(SensorDirectionValue sensorDirection) {
        magnetSensorConfigs.SensorDirection = sensorDirection;

        configuration.withMagnetSensor(magnetSensorConfigs);

        return this;
    }

    public CANCoderConfig withAbsoluteSensorDiscontinuityPoint(double discontinuityPoint) {
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;

        configuration.withMagnetSensor(magnetSensorConfigs);

        return this;
    }

    public CANCoderConfig withMagnetOffset(double magnetOffset) {
        magnetSensorConfigs.MagnetOffset = magnetOffset;

        configuration.withMagnetSensor(magnetSensorConfigs);

        return this;
    }
}
