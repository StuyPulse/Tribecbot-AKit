/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.units.measure.MomentOfInertia;

public interface SpindexerConstants {
    public interface SpindexerSettings {
        double FORWARD_DUTY_CYCLE = 1.0;
        double ANTI_POPCORN_DUTY_CYCLE = 0.2;
        double REVERSE_DUTY_CYCLE = -1.0;
        double STOP_SPEED = 0.0;
        double REVERSE_TIME = 2.0;
        double ANTI_POPCORN_FREQ = 100;
        double ANTI_POPCORN_LENGTH = 10;

        double RPM_TOLERANCE = 800.0;
        double TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE = 1500.0;
        double STALL_CURRENT_LIMIT = 40.0; // random number as of 3/9

        double GEAR_RATIO = 11.04 / 1.0;

        // sim
        MomentOfInertia SPINDEXER_MOI = KilogramSquareMeters.of(0.01);
    }

    public interface SpindexerDeviceIds {
        int LEADER = 30;
        int FOLLOWER = 31;
    }

    public interface SpindexerGains {
        double kP = 1.2;
        double kI = 0.0;
        double kD = 10.0;

        double kS = 0.25;
        double kV = 1.2;
        double kA = 0.010876;
    }

    public interface SpindexerMotorConfigs {
        TalonFXConfig SPINDEXER_LEAD_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimitAmps(45)
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.25)
                        .withPIDConstants(
                                SpindexerGains.kP, SpindexerGains.kI, SpindexerGains.kD, 0)
                        .withFFConstants(SpindexerGains.kS, SpindexerGains.kV, SpindexerGains.kA, 0)
                        .withSensorToMechanismRatio(SpindexerSettings.GEAR_RATIO);

        TalonFXConfig SPINDEXER_FOLLOW_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimitAmps(45)
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.25)
                        .withPIDConstants(
                                SpindexerGains.kP, SpindexerGains.kI, SpindexerGains.kD, 0)
                        .withFFConstants(SpindexerGains.kS, SpindexerGains.kV, SpindexerGains.kA, 0)
                        .withSensorToMechanismRatio(SpindexerSettings.GEAR_RATIO);
    }
}
