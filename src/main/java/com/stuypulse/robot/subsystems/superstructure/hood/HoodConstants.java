package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.util.config.TalonFXConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface HoodConstants {

  public interface Settings {
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

    final double GEAR_RATIO = 125.4;
    final double ENCODER_TO_MECH = 11.0;

    final Voltage HOOD_HOMING_VOLTAGE = Volts.of(0.5);

    final Angle ENCODER_OFFSET = Rotations.of(0.795);

    final Angle MAX_FROM_HORIZON = Degrees.of(45.0);
    final Angle MIN_FROM_HORIZON = Degrees.of(15.0);

    final Angle SOFT_LIMIT = Degrees.of(0.25);

    final Angle FORWARD_SOFT_LIMIT = MAX_FROM_HORIZON.minus(SOFT_LIMIT);

    final Angle REVERSE_SOFT_LIMIT = MIN_FROM_HORIZON.plus(SOFT_LIMIT);

    final Current STALL_CURRENT_LIMIT = Amps.of(0.55);
    final Time STALL_DEBOUNCE = Seconds.of(0.5);

    final Angle HOOD_TOLERANCE = Degrees.of(0.5);
    final Angle HOOD_SOTM_TOLERANCE = Degrees.of(2.0);
  }

  public interface MotorIds {
    final int MOTOR = 45;
    final int THROUGHBORE_ENCODER = 44;
  }

  public interface Gains {
    final double kP = 250.0;
    final double kI = 0.0;
    final double kD = 2.0;

    final double kS = 0.25;
    final double kV = 0.0;
    final double kA = 0.0;
  }

  public interface Angles {
    final LoggedNetworkNumber MANUAL_OVERRIDE_DEG =
        new LoggedNetworkNumber(
            "/Tuning/InterpolationTesting/Shoot State Target Angle (deg)", 44.0);

    final Angle MAX = Settings.FORWARD_SOFT_LIMIT;
    final Angle MIN = Settings.REVERSE_SOFT_LIMIT;
    final Angle FERRY_ANGLE = MAX;

    final Angle STOW = Degrees.of(21.0);
    final Angle KB = Degrees.of(20.0);
    final Angle LEFT_CORNER = Degrees.of(39.0);
    final Angle RIGHT_CORNER = Degrees.of(39.0);
  }

  public interface MotorConfig {
    final TalonFXConfig HOOD_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withPIDConstants(Gains.kP, Gains.kI, Gains.kD, 0)
            .withFFConstants(Gains.kS, Gains.kV, Gains.kA, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
            .withSensorToMechanismRatio(Settings.GEAR_RATIO)
            .withSoftLimits(
                true,
                true,
                Settings.FORWARD_SOFT_LIMIT.in(Rotations),
                Settings.REVERSE_SOFT_LIMIT.in(Rotations));
  }
}
