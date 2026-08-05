package com.stuypulse.robot.subsystems.intake;

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

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Current pivotMotorSupplyCurrent = Amps.zero();
    public Current pivotMotorStatorCurrent = Amps.zero();
    public Temperature pivotMotorTemperature = Celsius.zero();
    public Angle pivotMotorPosition = Degrees.zero();
    public Voltage pivotMotorAppliedVoltage = Volts.zero();
    public AngularVelocity pivotMotorVelocity = DegreesPerSecond.zero();

    public Current rollerLeaderMotorSupplyCurrent = Amps.zero();
    public Current rollerLeaderMotorStatorCurrent = Amps.zero();
    public Temperature rollerLeaderMotorTemperature = Celsius.zero();
    public Angle rollerLeaderMotorPosition = Degrees.zero();
    public Voltage rollerLeaderMotorAppliedVoltage = Volts.zero();
    public AngularVelocity rollerLeaderMotorVelocity = DegreesPerSecond.zero();

    public Current rollerFollowerMotorSupplyCurrent = Amps.zero();
    public Current rollerFollowerMotorStatorCurrent = Amps.zero();
    public Temperature rollerFollowerMotorTemperature = Celsius.zero();
    public Angle rollerFollowerMotorPosition = Degrees.zero();
    public Voltage rollerFollowerMotorAppliedVoltage = Volts.zero();
    public AngularVelocity rollerFollowerMotorVelocity = DegreesPerSecond.zero();
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public static enum PivotIOOutputMode {
    POSITION,
    TORQUE_CURRENT,
    VOLTAGE,
    STOP
  }

  public static enum RollerIOOutputMode {
    DUTY_CYCLE,
    STOP
  }

  public static class IntakeIOOutputs {
    public PivotIOOutputMode pivotMode = PivotIOOutputMode.POSITION;
    public Angle pivotTargetPosition = Degrees.zero();
    public Current pivotTargetTorqueCurrent = Amps.zero();
    public Voltage pivotTargetVoltage = Volts.zero();

    public RollerIOOutputMode rollerMode = RollerIOOutputMode.DUTY_CYCLE;
    public double rollerTargetDutyCycle = 0.0;
  }

  public default void applyOutputs(IntakeIOOutputs outputs) {}

  public default void seedPivotPosition(Angle position) {}
}
