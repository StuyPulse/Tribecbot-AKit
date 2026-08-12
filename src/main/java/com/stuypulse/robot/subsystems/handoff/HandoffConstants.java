package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface HandoffConstants {
  public interface HandoffSettings {
    final double GEAR_RATIO = 3.0 / 1.0;

    final double HANDOFF_STOP = 0.0;
    final double HANDOFF_MAX = 4800.0;
    final double HANDOFF_REVERSE = -500.0;
    final double RPM_TOLERANCE = 2200.0;
    final double REVERSE_TIME = 2.0;
    final double RPM_SOTM_TOLERANCE = 700.0;
    final LoggedNetworkNumber HANDOFF_RPM =
        new LoggedNetworkNumber("/Tuning/Handoff/Target RPM", HANDOFF_MAX);

    final double FORWARD_DUTY_CYCLE = 1.0;
    final double REVERSE_DUTY_CYCLE = -1.0;

    final LoggedNetworkNumber HANDOFF_STALL_CURRENT_AMPS =
        new LoggedNetworkNumber("/Tuning/Handoff/Stall Current Limit for Reverse", 30.0);
    final Time HANDOFF_STALL_DEBOUNCE = Seconds.of(0.5);
  }

  public interface HandoffIds {
    final int MOTOR_LEAD = 43;
    final int MOTOR_FOLLOW = 48;
  }

  public interface HandoffMotorConfig {
    final TalonFXConfig HANDOFF_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25);
  }
}
