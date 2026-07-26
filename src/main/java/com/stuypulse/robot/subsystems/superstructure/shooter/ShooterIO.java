package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public Current shooterLeaderMotorSupplyCurrent = Amps.zero();
    public Current shooterLeaderMotorStatorCurrent = Amps.zero();
    public Temperature shooterLeaderMotorTemperature = Celsius.zero();
    public Angle shooterLeaderMotorPosition = Degrees.zero();
    public Voltage shooterLeaderMotorAppliedVoltage = Volts.zero();
    public AngularVelocity shooterLeaderMotorVelocity = DegreesPerSecond.zero();

    public Current shooterFollowerMotorSupplyCurrent = Amps.zero();
    public Current shooterFollowerMotorStatorCurrent = Amps.zero();
    public Temperature shooterFollowerMotorTemperature = Celsius.zero();
    public Angle shooterFollowerMotorPosition = Degrees.zero();
    public Voltage shooterFollowerMotorAppliedVoltage = Volts.zero();
    public AngularVelocity shooterFollowerMotorVelocity = DegreesPerSecond.zero();
  }

  class ShooterIOOutputs {
    public AngularVelocity shooterVelocity = RPM.zero();
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}
}
