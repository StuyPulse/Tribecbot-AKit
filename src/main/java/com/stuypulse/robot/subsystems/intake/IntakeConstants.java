/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface IntakeConstants {
    public interface IntakeSettings {
        Angle PIVOT_STOW_ANGLE = Degrees.of(71.0);
        Angle PIVOT_DEPLOY_ANGLE = Degrees.of(-10.0);
        Angle PIVOT_DIGEST_ANGLE = Degrees.of(30);

        Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(5.0);

        Angle PIVOT_MAX_ANGLE = Degrees.of(76.4);
        Angle PIVOT_MIN_ANGLE = Degrees.of(-10.0);

        Angle THRESHOLD_TO_START_ROLLERS = Degrees.of(10.0);

        Angle ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Degrees.of(15.0);
        Voltage HOMING_VOLTAGE = Volts.of(3.0);

        Voltage PUSHDOWN_VOLTAGE = Volts.of(-3.0);
        Current PUSHDOWN_CURRENT_TELEOP =
                Amps.of(-75.0); // new LoggedNetworkNumber("Intake/Pushdown Current", -65.0);
        // //TODO: GET ACTUAL
        // TYTY
        Current PUSHDOWN_CURRENT_AUTON = Amps.of(-80.0);
        double PIVOT_GEAR_RATIO = 32.0 / 20.0 * 64.0 / 18.0 * 60.0 / 8.0;

        Current PIVOT_STALL_CURRENT = Amps.of(0); // TODO: set value
        Time PIVOT_STALL_DEBOUNCE = Seconds.of(1.0); // TODO: VERIFY

        Time ROLLER_STALL_DEBOUNCE = Seconds.of(0.05); // TODO: VERIFY
        Current ROLLER_STALL_CURRENT = Amps.of(50.0);

        // Sim
        Distance ARM_LENGTH = Meters.of(0.4);
        Mass ARM_MASS = Kilograms.of(2.0);
        MomentOfInertia PIVOT_MOI =
                KilogramSquareMeters.of(
                        SingleJointedArmSim.estimateMOI(
                                ARM_MASS.in(Kilograms), ARM_LENGTH.in(Meters)));
    }

    public interface IntakeDeviceIds {
        int PIVOT = 20;
        int ROLLER_LEADER = 21;
        int ROLLER_FOLLOWER = 22;
    }

    public interface IntakeGains {
        public interface Pivot {
            LoggedNetworkNumber kP =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kP", 125.0);
            LoggedNetworkNumber kI =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kI", 0.0);
            LoggedNetworkNumber kD =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kD", 10.0);

            LoggedNetworkNumber kS =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kS", 0.0);
            LoggedNetworkNumber kV =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kV", 0.12);
            LoggedNetworkNumber kA =
                    new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kA", 0.0);

            double kG = 0.5;
        }
    }

    public interface IntakeMotorConfigs {
        TalonFXConfig PIVOT_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimitAmps(10.0) // was 60 on practice day
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.25)
                        .withPIDConstants(
                                IntakeGains.Pivot.kP.get(),
                                IntakeGains.Pivot.kI.get(),
                                IntakeGains.Pivot.kD.get(),
                                0)
                        .withFFConstants(
                                IntakeGains.Pivot.kS.get(),
                                IntakeGains.Pivot.kV.get(),
                                IntakeGains.Pivot.kA.get(),
                                IntakeGains.Pivot.kG,
                                0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withSensorToMechanismRatio(IntakeSettings.PIVOT_GEAR_RATIO);

        TalonFXConfig ROLLER_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withSupplyCurrentLimitAmps(37.0)
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.50);
    }
}
