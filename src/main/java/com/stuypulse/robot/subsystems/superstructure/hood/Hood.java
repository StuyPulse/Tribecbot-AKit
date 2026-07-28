package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static final Hood instance;

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new Hood(new HoodIOTalonFX());

      case SIM -> instance = new Hood(new HoodIOSim());

      default -> instance = new Hood(new HoodIO() {});
    }
  }

  public static Hood getInstance() {
    return instance;
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;
  private final HoodIOOutputs outputs;

  // Using SOTM or FOTM
  private boolean OTM;

  private final Debouncer hoodStallingDebouncer;

  private Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();
    outputs = new HoodIOOutputs();

    OTM = false;

    hoodStallingDebouncer =
        new Debouncer(Settings.Superstructure.Hood.STALL_DEBOUNCE, DebounceType.kBoth);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood/Output Mode", outputs.outputMode);
    Logger.recordOutput("Hood/Position Setpoint", outputs.position);
    Logger.recordOutput("Hood/Voltage Setpoint", outputs.voltage);

    io.applyOutputs(outputs);
  }

  public boolean atTolerance() {
    Angle error = inputs.hoodMotorPosition.minus(outputs.position);

    if (Robot.isReal()) {
      if (OTM) {
        return error.abs(Degrees) < Settings.Superstructure.HOOD_SOTM_TOLERANCE.in(Degrees);
      } else {
        return error.abs(Degrees) < Settings.Superstructure.HOOD_TOLERANCE.in(Degrees);
      }
    } else {
      return error.abs(Degrees) < Settings.Superstructure.HOOD_TOLERANCE.in(Degrees) + 5;
    }
  }

  private void runPosition(Angle position, boolean OTM) {
    this.OTM = OTM;

    outputs.outputMode = HoodIOOutputMode.POSITION;
    outputs.position = position;
  }

  private void runVoltage(Voltage voltage) {
    outputs.outputMode = HoodIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  private Angle hoodAnalogToInput(CommandXboxController gamepad) {
    double hoodMin = Settings.Superstructure.Hood.Angles.MIN.in(Degrees);
    double hoodMax = Settings.Superstructure.Hood.Angles.MAX.in(Degrees);

    return Degrees.of(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2));
  }

  private boolean isStalling() {
    return hoodStallingDebouncer.calculate(
        inputs.hoodMotorStatorCurrent.gt(Settings.Superstructure.Hood.STALL_CURRENT_LIMIT));
  }

  public Command runHomingUpper() {
    return run(() -> {
          runVoltage(Settings.Superstructure.Hood.HOOD_HOMING_VOLTAGE);
        })
        .until(this::isStalling)
        .andThen(() -> io.seedHoodPosition(Settings.Superstructure.Hood.MAX_FROM_HORIZON))
        .andThen(runStow());
  }

  public Command runStow() {
    return run(() -> runPosition(Settings.Superstructure.Hood.Angles.STOW, false));
  }

  public Command runAnalog(CommandXboxController gamepad) {
    return run(() -> runPosition(hoodAnalogToInput(gamepad), false));
  }
}
