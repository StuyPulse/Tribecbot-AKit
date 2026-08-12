package com.stuypulse.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.RGBWColor;

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

  public record LEDPattern(int start, int end, RGBWColor color) {}

  public static class LEDIOOutputs {
    @AutoLogOutput(key = "LEDs/Pattern")
    public final List<LEDPattern> patterns = new ArrayList<>();
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void applyOutputs(LEDIOOutputs outputs) {}

  default void periodic() {}
  ;
}
