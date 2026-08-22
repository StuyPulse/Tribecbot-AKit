/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface TurretConstants {

    public interface TurretSettings {
        Angle TOLERANCE = Degrees.of(2.0);

        LoggedNetworkNumber SOTM_TOLERANCE_THRESHOLD_METERS =
                new LoggedNetworkNumber(
                        "/Tuning/Superstructure/Turret/SOTM Tolerance Dist Threshold (Meters)",
                        1.75);

        LoggedNetworkNumber SOTM_TOLERANCE_CLOSE_DEG =
                new LoggedNetworkNumber(
                        "/Tuning/Superstructure/Turret/SOTM Tolerance Close (Deg)", 10.0);

        LoggedNetworkNumber SOTM_TOLERANCE_FAR_DEG =
                new LoggedNetworkNumber(
                        "/Tuning/Superstructure/Turret/SOTM Tolerance Far (Deg)", 6.0);

        Angle FOTM_TOLERANCE = Degrees.of(10.0);

        double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
        Time WRAP_DEBOUNCE = Seconds.of(0.5);
        double SETPOINT_FILTER_THRESHOLD_DEG = 0.5;

        Angle MAX_THEORETICAL_ROTATION = Degrees.of(612.0);
        Angle MIN_THEORETICAL_ROTATION = Degrees.of(-612.0);

        double RANGE_CW = 90.0;
        double RANGE_CCW = -360.0;

        Angle GAIN_SWITCHING_THRESHOLD_START = Degrees.of(30.0);
        Angle GAIN_SWITCHING_THRESHOLD_END = Degrees.of(3.0);

        Translation2d TURRET_OFFSET =
                new Translation2d(
                        Inches.of(-4.0).in(edu.wpi.first.units.Units.Meters),
                        Inches.of(8.0).in(edu.wpi.first.units.Units.Meters));

        Distance TURRET_HEIGHT = Inches.of(0.0);

        double GEAR_RATIO_MOTOR_TO_MECH = (60.0 / 9.0) * (95.0 / 12.0);
    }

    public interface TurretDeviceIds {
        int MOTOR = 40;
        int ENCODER_17T = 42;
        int ENCODER_18T = 41;
    }

    public interface TurretAngles {
        Angle KB = Degrees.of(0.0);
        Angle LEFT_CORNER = Degrees.of(-233.0);
        Angle RIGHT_CORNER = Degrees.of(53.0);
    }

    public interface TurretGains {

        public interface slot0 {
            double kP = 200.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.4775;
            double kV = 0.0;
            double kA = 0.0;
        }

        public interface slot1 {
            LoggedNetworkNumber kP =
                    new LoggedNetworkNumber("Superstructure/Turret/Gains/kP", 150.0);
            LoggedNetworkNumber kI = new LoggedNetworkNumber("Superstructure/Turret/Gains/kI", 0.0);
            LoggedNetworkNumber kD = new LoggedNetworkNumber("Superstructure/Turret/Gains/kD", 3.0);
            LoggedNetworkNumber kS =
                    new LoggedNetworkNumber("Superstructure/Turret/Gains/kS", 0.4775);
            LoggedNetworkNumber kV = new LoggedNetworkNumber("Superstructure/Turret/Gains/kV", 0.0);
            LoggedNetworkNumber kA = new LoggedNetworkNumber("Superstructure/Turret/Gains/kA", 0.0);
        }

        LoggedNetworkNumber kOmega =
                new LoggedNetworkNumber("Superstructure/Turret/Gains/kOmega", 3.43);
        LoggedNetworkNumber kTranslation =
                new LoggedNetworkNumber("Superstructure/Turret/Gains/kTranslation", 0.0);
    }

    public interface TurretBigGear {
        int TEETH = 95;
    }

    public interface TurretEncoder17t {
        int TEETH = 17;
        Angle OFFSET = Rotations.of(-0.185);
    }

    public interface TurretEncoder18t {
        int TEETH = 18;
        Angle OFFSET = Rotations.of(-0.814);
    }

    public interface TurretSoftwareLimits {
        double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
        double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
    }

    public interface TurretMotorConfigs {
        TalonFXConfig TURRET_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimitAmps(80.0)
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.0)
                        .withPIDConstants(
                                TurretGains.slot0.kP, TurretGains.slot0.kI, TurretGains.slot0.kD, 0)
                        .withFFConstants(
                                TurretGains.slot0.kS, TurretGains.slot0.kV, TurretGains.slot0.kA, 0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
                        .withPIDConstants(0.0, 0.0, 10.0, 2)
                        .withFFConstants(
                                TurretGains.slot0.kS, TurretGains.slot0.kV, TurretGains.slot0.kA, 2)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 2)
                        .withPIDConstants(
                                TurretGains.slot1.kP.get(),
                                TurretGains.slot1.kI.get(),
                                TurretGains.slot1.kD.get(),
                                1)
                        .withFFConstants(
                                TurretGains.slot1.kS.get(),
                                TurretGains.slot1.kV.get(),
                                TurretGains.slot1.kA.get(),
                                1)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 1)
                        .withSensorToMechanismRatio(TurretSettings.GEAR_RATIO_MOTOR_TO_MECH)
                        .withSoftLimits(
                                false,
                                false,
                                TurretSoftwareLimits.FORWARD_MAX_ROTATIONS,
                                TurretSoftwareLimits.BACKWARDS_MAX_ROTATIONS);
    }
}
