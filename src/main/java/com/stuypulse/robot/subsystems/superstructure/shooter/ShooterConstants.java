/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface ShooterConstants {

    public interface ShooterSettings {
        final Current IS_SHOOTING_CURRENT = Amps.of(25.0);

        final double GEAR_RATIO = 1.0;

        final Distance FLYWHEEL_RADIUS = Inches.of(3.965 / 2.0);

        final AngularVelocity SHOOTER_TOLERANCE_RPM_HIGH = RPM.of(50.0);
        final AngularVelocity SHOOTER_TOLERANCE_RPM_LOW = RPM.of(80.0);

        final AngularVelocity SHOOTER_SOTM_TOLERANCE_RPM_HIGH = RPM.of(100.0);
        final AngularVelocity SHOOTER_SOTM_TOLERANCE_RPM_LOW = RPM.of(100.0);

        final AngularVelocity SHOOTER_FOTM_TOLERANCE_RPM_HIGH = RPM.of(150.0);
        final AngularVelocity SHOOTER_FOTM_TOLERANCE_RPM_LOW = RPM.of(250.0);
    }

    public interface ShooterDeviceIds {
        final int MOTOR_LEAD = 47;
        final int MOTOR_FOLLOW = 46;
    }

    public interface ShooterGains {
        // VTC PID
        final LoggedNetworkNumber kP =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kP", 10.5);

        final LoggedNetworkNumber kI =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kI", 0.0);

        final LoggedNetworkNumber kD =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kD", 0.0);

        final LoggedNetworkNumber kS =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kS", 2.47);

        final LoggedNetworkNumber kV =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kV", 0.01775);

        final LoggedNetworkNumber kA =
                new LoggedNetworkNumber("Superstructure/Shooter/Gains/kA", 0.0);
    }

    public interface ShooterRPMValues {
        final LoggedNetworkNumber MANUAL_OVERRIDE =
                new LoggedNetworkNumber(
                        "/Tuning/InterpolationTesting/Shoot State Target RPM", 3863.0);

        final AngularVelocity REVERSE = edu.wpi.first.units.Units.RPM.zero();

        final AngularVelocity KB = edu.wpi.first.units.Units.RPM.of(2675.0);

        final AngularVelocity LEFT_CORNER = edu.wpi.first.units.Units.RPM.of(3650.0);

        final AngularVelocity RIGHT_CORNER = edu.wpi.first.units.Units.RPM.of(3650.0);
    }

    public interface ShooterMotorConfigs {
        final TalonFXConfig SHOOTER_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withSupplyCurrentLimitEnabled(false)
                        .withStatorCurrentLimitEnabled(false)
                        .withPIDConstants(
                                ShooterGains.kP.get(),
                                ShooterGains.kI.get(),
                                ShooterGains.kD.get(),
                                0)
                        .withFFConstants(
                                ShooterGains.kS.get(),
                                ShooterGains.kV.get(),
                                ShooterGains.kA.get(),
                                0)
                        .withSensorToMechanismRatio(ShooterSettings.GEAR_RATIO)
                        .withStatorCurrentLimitAmps(140.0)
                        .withStatorCurrentLimitEnabled(false)
                        .withSupplyCurrentLimitAmps(100.0)
                        .withSupplyCurrentLimitEnabled(true)
                        .withLowerLimitSupplyCurrent(60.0, 1.0);
    }
}
