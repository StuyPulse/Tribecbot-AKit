package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;

public interface SpindexerConstants {
  public interface SpindexerSettings {
    final double FORWARD_DUTY_CYCLE = 1.0;
    final double ANTI_POPCORN_DUTY_CYCLE = 0.2;
    final double REVERSE_DUTY_CYCLE = -1.0;
    final double STOP_SPEED = 0.0;
    final double REVERSE_TIME = 2.0;
    final double ANTI_POPCORN_FREQ = 100;
    final double ANTI_POPCORN_LENGTH = 10;

    final double RPM_TOLERANCE = 800.0;
    final double TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE = 1500.0;
    final double STALL_CURRENT_LIMIT = 40.0; // random number as of 3/9

    final double GEAR_RATIO = 11.04 / 1.0;
  }

  public interface SpindexerDeviceIds {
    final int LEADER = 30;
    final int FOLLOWER = 31;
  }

  public interface SpindexerGains {
    double kP = 1.2;
    double kI = 0.0;
    double kD = 10.0;

    double kS = 0.25;
    double kV = 1.2;
    double kA = 0.010876;
  }

  public interface SpindexerMotorConfig {
    final TalonFXConfig SPINDEXER_LEAD_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(45)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withPIDConstants(SpindexerGains.kP, SpindexerGains.kI, SpindexerGains.kD, 0)
            .withFFConstants(SpindexerGains.kS, SpindexerGains.kV, SpindexerGains.kA, 0)
            .withSensorToMechanismRatio(SpindexerSettings.GEAR_RATIO);

    final TalonFXConfig SPINDEXER_FOLLOW_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(45)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withPIDConstants(SpindexerGains.kP, SpindexerGains.kI, SpindexerGains.kD, 0)
            .withFFConstants(SpindexerGains.kS, SpindexerGains.kV, SpindexerGains.kA, 0)
            .withSensorToMechanismRatio(SpindexerSettings.GEAR_RATIO);
  }
}
