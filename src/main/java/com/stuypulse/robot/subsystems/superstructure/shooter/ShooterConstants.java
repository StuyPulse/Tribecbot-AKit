package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface ShooterConstants {

    public interface ShooterSettings {
        final Current IS_SHOOTING_CURRENT = Amps.of(25.0);

        final double GEAR_RATIO = 1.0;

        final Distance FLYWHEEL_RADIUS =
            Inches.of(3.965 / 2.0);
    }

    public interface MotorIds {
        final int MOTOR_LEAD = 47;
        final int MOTOR_FOLLOW = 46;
    }

    public interface Gains {
        // VTC PID
        final LoggedNetworkNumber kP =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kP",
                10.5);

        final LoggedNetworkNumber kI =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kI",
                0.0);

        final LoggedNetworkNumber kD =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kD",
                0.0);

        final LoggedNetworkNumber kS =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kS",
                2.47);

        final LoggedNetworkNumber kV =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kV",
                0.01775);

        final LoggedNetworkNumber kA =
            new LoggedNetworkNumber(
                "Superstructure/Shooter/Gains/kA",
                0.0);
    }

    public interface RPMValues {
        final LoggedNetworkNumber MANUAL_OVERRIDE =
            new LoggedNetworkNumber(
                "/Tuning/InterpolationTesting/Shoot State Target RPM",
                3863.0);

        final AngularVelocity REVERSE =
            edu.wpi.first.units.Units.RPM.zero();

        final AngularVelocity KB =
            edu.wpi.first.units.Units.RPM.of(2675.0);

        final AngularVelocity LEFT_CORNER =
            edu.wpi.first.units.Units.RPM.of(3650.0);

        final AngularVelocity RIGHT_CORNER =
            edu.wpi.first.units.Units.RPM.of(3650.0);
    }

    public interface MotorConfig {
        final TalonFXConfig SHOOTER_CONFIG =
            new TalonFXConfig()
                .withInvertedValue(
                    InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
                .withSupplyCurrentLimitEnabled(false)
                .withStatorCurrentLimitEnabled(false)
                .withPIDConstants(
                    Gains.kP.get(),
                    Gains.kI.get(),
                    Gains.kD.get(),
                    0)
                .withFFConstants(
                    Gains.kS.get(),
                    Gains.kV.get(),
                    Gains.kA.get(),
                    0)
                .withSensorToMechanismRatio(ShooterSettings.GEAR_RATIO)
                .withStatorCurrentLimitAmps(140.0)
                .withStatorCurrentLimitEnabled(false)
                .withSupplyCurrentLimitAmps(100.0)
                .withSupplyCurrentLimitEnabled(true)
                .withLowerLimitSupplyCurrent(60.0, 1.0);
    }
}