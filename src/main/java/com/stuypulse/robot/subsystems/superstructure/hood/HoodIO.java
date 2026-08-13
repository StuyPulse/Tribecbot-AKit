package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public Current hoodMotorSupplyCurrent = Amps.zero();
    public Current hoodMotorStatorCurrent = Amps.zero();
    public Temperature hoodMotorTemperature = Celsius.zero();
    public Angle hoodMotorPosition = Degrees.zero();
    public Voltage hoodMotorAppliedVoltage = Volts.zero();
    public AngularVelocity hoodMotorVelocity = DegreesPerSecond.zero();
  }

  public static enum HoodIOOutputMode {
    POSITION,
    VOLTAGE,
    STOP
  }

  public static class HoodIOOutputs {
    @AutoLogOutput(key = "Superstructure/Hood/Output Mode")
    public HoodIOOutputMode outputMode = HoodIOOutputMode.POSITION;

    @AutoLogOutput(key = "Superstructure/Hood/Target Position")
    public Angle position = Degrees.zero();

    @AutoLogOutput(key = "Superstructure/Hood/Target Voltage")
    public Voltage voltage = Volts.zero();
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs ouptuts) {}

  public default void seedHoodPosition(Angle position) {}
}
