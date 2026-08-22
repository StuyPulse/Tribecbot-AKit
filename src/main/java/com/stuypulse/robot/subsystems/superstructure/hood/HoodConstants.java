/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface HoodConstants {

    public interface HoodSettings {
        /*
         * DISCLAIMER: THERE IS NO ABS ENCODER ON THE BOT RN.
         *
         * The absolute encoder is mounted on an 11:1 gear reduction
         * relative to the hood mechanism. This means:
         *
         * - The encoder rotates 11 times for every 1 full rotation of the hood.
         * - The hood's physical range of motion is only 30 degrees.
         *
         * Because 30 degrees * 11 = 330 degrees, the encoder will never
         * exceed 360 degrees over the entire hood travel. Therefore, the
         * absolute encoder reading (0-330 degrees) uniquely maps to the
         * hood's 0-30 degree mechanical range without ambiguity.
         */

        double GEAR_RATIO = 125.4;
        double ENCODER_TO_MECH = 11.0;

        Voltage HOOD_HOMING_VOLTAGE = Volts.of(0.5);

        Angle ENCODER_OFFSET = Rotations.of(0.795);

        Angle MAX_FROM_HORIZON = Degrees.of(45.0);
        Angle MIN_FROM_HORIZON = Degrees.of(15.0);

        Angle SOFT_LIMIT = Degrees.of(0.25);

        Angle FORWARD_SOFT_LIMIT = MAX_FROM_HORIZON.minus(SOFT_LIMIT);

        Angle REVERSE_SOFT_LIMIT = MIN_FROM_HORIZON.plus(SOFT_LIMIT);

        Current STALL_CURRENT_LIMIT = Amps.of(0.55);
        Time STALL_DEBOUNCE = Seconds.of(0.5);

        Angle HOOD_TOLERANCE = Degrees.of(0.5);
        Angle HOOD_SOTM_TOLERANCE = Degrees.of(2.0);

        // for sim
        Distance HOOD_ARM_LENGTH = Meters.of(0.3);
        Distance MIN_HEIGHT = HOOD_ARM_LENGTH.times(Math.sin(MIN_FROM_HORIZON.in(Radians)));
        Distance MAX_HEIGHT = HOOD_ARM_LENGTH.times(Math.sin(MAX_FROM_HORIZON.in(Radians)));
        Distance DRUM_RADIUS = Meters.of(0.01);
    }

    public interface HoodDeviceIds {
        int MOTOR = 45;
        int THROUGHBORE_ENCODER = 44;
    }

    public interface HoodGains {
        double kP = 250.0;
        double kI = 0.0;
        double kD = 2.0;

        double kS = 0.25;
        double kV = 0.0;
        double kA = 0.0;
    }

    public interface HoodAngles {
        LoggedNetworkNumber MANUAL_OVERRIDE_DEG =
                new LoggedNetworkNumber(
                        "/Tuning/InterpolationTesting/Shoot State Target Angle (deg)", 44.0);

        Angle MAX = HoodSettings.FORWARD_SOFT_LIMIT;
        Angle MIN = HoodSettings.REVERSE_SOFT_LIMIT;
        Angle FERRY_ANGLE = MAX;

        Angle STOW = Degrees.of(21.0);
        Angle KB = Degrees.of(20.0);
        Angle LEFT_CORNER = Degrees.of(39.0);
        Angle RIGHT_CORNER = Degrees.of(39.0);
    }

    public interface HoodMotorConfigs {
        TalonFXConfig HOOD_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimitAmps(80.0)
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.25)
                        .withPIDConstants(HoodGains.kP, HoodGains.kI, HoodGains.kD, 0)
                        .withFFConstants(HoodGains.kS, HoodGains.kV, HoodGains.kA, 0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
                        .withSensorToMechanismRatio(HoodSettings.GEAR_RATIO)
                        .withSoftLimits(
                                true,
                                true,
                                HoodSettings.FORWARD_SOFT_LIMIT.in(Rotations),
                                HoodSettings.REVERSE_SOFT_LIMIT.in(Rotations));
    }
}
