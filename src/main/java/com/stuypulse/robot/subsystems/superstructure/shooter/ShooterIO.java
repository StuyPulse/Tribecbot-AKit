/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public Current shooterLeaderMotorSupplyCurrent = Amps.zero();
        public Current shooterLeaderMotorStatorCurrent = Amps.zero();
        public Temperature shooterLeaderMotorTemperature = Celsius.zero();
        public Angle shooterLeaderMotorPosition = Degrees.zero();
        public Voltage shooterLeaderMotorAppliedVoltage = Volts.zero();
        public AngularVelocity shooterLeaderMotorVelocity = DegreesPerSecond.zero();

        public Current shooterFollowerMotorSupplyCurrent = Amps.zero();
        public Current shooterFollowerMotorStatorCurrent = Amps.zero();
        public Temperature shooterFollowerMotorTemperature = Celsius.zero();
        public Angle shooterFollowerMotorPosition = Degrees.zero();
        public Voltage shooterFollowerMotorAppliedVoltage = Volts.zero();
        public AngularVelocity shooterFollowerMotorVelocity = DegreesPerSecond.zero();
    }

    public static enum ShooterIOOutputMode {
        VELOCITY,
        STOP
    }

    public static class ShooterIOOutputs {
        @AutoLogOutput(key = "Superstructure/Shooter/Output Mode")
        public ShooterIOOutputMode shooterMode = ShooterIOOutputMode.VELOCITY;

        @AutoLogOutput(key = "Superstructure/Shooter/Target Velocity")
        public AngularVelocity shooterVelocity = RPM.zero();
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void applyOutputs(ShooterIOOutputs outputs) {}
}
