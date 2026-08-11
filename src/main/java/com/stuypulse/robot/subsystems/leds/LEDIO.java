package com.stuypulse.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public boolean isConnected = false;
    public Voltage supplyVoltage = Volts.zero();
    public Voltage fiveVRailVoltage = Volts.zero();
    public Current outputCurrentAmps = Amps.zero();
    public Temperature LEDTemperature = Celsius.zero();
    public boolean hardwareFault = false;
    public boolean underVoltageFault = false;
  }

  public static class LEDIOOutputs {
    @AutoLogOutput(key = "LEDs/Pattern")
    public ControlRequest pattern = LEDConstants.solidColorRequest.withColor(LEDConstants.DISABLED);

    @AutoLogOutput(key = "LEDs/Left Limelight Dead")
    public boolean leftLimelightDead = false;

    @AutoLogOutput(key = "LEDs/Right Limelight Dead")
    public boolean rightLimelightDead = false;

    @AutoLogOutput(key = "LEDs/Back Limelight Dead")
    public boolean backLimelightDead = false;
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void applyOutputs(LEDIOOutputs outputs) {}

  default void periodic() {};
}
