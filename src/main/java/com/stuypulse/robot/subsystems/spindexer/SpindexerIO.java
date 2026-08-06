package com.stuypulse.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public Current spindexerLeaderMotorSupplyCurrent = Amps.zero();
    public Current spindexerLeaderMotorStatorCurrent = Amps.zero();
    public Temperature spindexerLeaderMotorTemperature = Celsius.zero();
    public Angle spindexerLeaderMotorPosition = Degrees.zero();
    public Voltage spindexerLeaderMotorAppliedVoltage = Volts.zero();
    public AngularVelocity spindexerLeaderMotorVelocity = DegreesPerSecond.zero();

    public Current spindexerFollowerMotorSupplyCurrent = Amps.zero();
    public Current spindexerFollowerMotorStatorCurrent = Amps.zero();
    public Temperature spindexerFollowerMotorTemperature = Celsius.zero();
    public Angle spindexerFollowerMotorPosition = Degrees.zero();
    public Voltage spindexerFollowerMotorAppliedVoltage = Volts.zero();
    public AngularVelocity spindexerFollowerMotorVelocity = DegreesPerSecond.zero();
  }

  public enum SpindexerIOOutputMode {
    DUTY_CYCLE,
    STOP
  }

  public static class SpindexerIOOutputs {
    @AutoLogOutput(key = "Spindexer/Output Mode")
    public SpindexerIOOutputMode spindexerMode = SpindexerIOOutputMode.DUTY_CYCLE;

    @AutoLogOutput(key = "Spindexer/Duty Cycle Setpoint")
    public double spindexerLeaderDutyCycle = 0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
