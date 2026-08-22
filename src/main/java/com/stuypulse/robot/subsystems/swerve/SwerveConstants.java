/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public interface SwerveConstants {
    public interface SwerveConstraints {
        LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.16);
        LinearVelocity MAX_VELOCITY_SOTM = MetersPerSecond.of(1.75);
        LinearVelocity MAX_VELOCITY_FOTM = MetersPerSecond.of(4.16);

        AngularVelocity MAX_ANGULAR_VEL = DegreesPerSecond.of(300.0);
        AngularVelocity MAX_ANGULAR_VEL_SOTM = DegreesPerSecond.of(75.0);
        AngularVelocity MAX_ANGULAR_VEL_FOTM = DegreesPerSecond.of(150.0);

        LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(15.0);
        LinearAcceleration MAX_ACCEL_SOTM = MetersPerSecondPerSecond.of(4.0);
        LinearAcceleration MAX_ACCEL_FOTM = MetersPerSecondPerSecond.of(15.0);
        AngularAcceleration MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED =
                DegreesPerSecondPerSecond.of(900.0);
    }

    public interface SwerveCharacterization {
        Time FF_START_DELAY = Seconds.of(2.0);
        double FF_RAMP_RATE = 0.1;

        double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
        double WHEEL_RADIUS_RAMP_RATE = 0.05;
    }
}
