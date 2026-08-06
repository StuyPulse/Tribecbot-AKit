package com.stuypulse.robot.subsystems.superstructure.turret;

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

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Current turretMotorSupplyCurrent = Amps.zero();
    public Current turretMotorStatorCurrent = Amps.zero();
    public Temperature turretMotorTemperature = Celsius.zero();
    public Angle turretMotorPosition = Degrees.zero();
    public Voltage turretMotorAppliedVoltage = Volts.zero();
    public AngularVelocity turretMotorVelocity = DegreesPerSecond.zero();

    public Angle encoder17tPosition = Degrees.zero();
    public Angle encoder18tPosition = Degrees.zero();
  }

  public enum TurretIOOutputMode {
    POSITION,
    STOP
  }

  public static class TurretIOOutputs {
    public TurretIOOutputMode turretMode = TurretIOOutputMode.POSITION;
    public Angle turretPosition = Degrees.zero();
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}
