package com.stuypulse.robot.subsystems.superstructure.hood;

import static com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.*;
import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  private static final Hood instance;

  static {
    switch (GlobalSettings.currentMode) {
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

  @AutoLogOutput(key = "States/Intake")
  private HoodState state;

  private final Debouncer hoodStallingDebouncer;
  private final Debouncer hoodAtToleranceDebouncer;

  private Angle driverInput;

  private boolean atTolerance;

  private Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();
    outputs = new HoodIOOutputs();

    setState(HoodState.STOW);

    hoodStallingDebouncer =
        new Debouncer(HoodSettings.STALL_DEBOUNCE.in(Seconds), DebounceType.kBoth);
    hoodAtToleranceDebouncer = new Debouncer(0.05, DebounceType.kBoth);

    this.atTolerance = false;
  }

  public enum HoodState {
    STOW,
    FERRY,
    MANUAL_OVERRIDE,
    KB,
    LEFT_CORNER,
    RIGHT_CORNER,
    INTERPOLATION,
    SOTM,
    FOTM,
    ANALOG,
    HOMING_UPPER,
    HOMING_LOWER,
    IDLE;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (!GlobalSettings.EnabledSubsystems.HOOD.get()) {
      stopHood();

      return;
    }

    switch (state) {
      case HOMING_UPPER -> {
        if (isStalling()) {
          io.seedHoodPosition(HoodSettings.MAX_FROM_HORIZON);
          setState(HoodState.STOW);
        } else {
          runVoltage(HoodSettings.HOOD_HOMING_VOLTAGE);
        }
      }

      case HOMING_LOWER -> {
        if (isStalling()) {
          io.seedHoodPosition(HoodSettings.MIN_FROM_HORIZON);
          setState(HoodState.STOW);
        } else {
          runVoltage(HoodSettings.HOOD_HOMING_VOLTAGE.unaryMinus());
        }
      }

      case STOW -> runPosition(HoodAngles.STOW);
      case FERRY -> runPosition(InterpolationCalculator.getInterpolatedFerryAngle());
      case MANUAL_OVERRIDE -> runPosition(Degrees.of(HoodAngles.MANUAL_OVERRIDE_DEG.get()));
      case KB -> runPosition(HoodAngles.KB);
      case LEFT_CORNER -> runPosition(HoodAngles.LEFT_CORNER);
      case RIGHT_CORNER -> runPosition(HoodAngles.RIGHT_CORNER);
      case INTERPOLATION -> runPosition(InterpolationCalculator.getInterpolatedShotAngle());
      case SOTM -> runPosition(SOTMCalculator.calculateHoodAngleSOTM());
      case FOTM -> runPosition(SOTMCalculator.calculateHoodAngleFOTM());
      case ANALOG -> runPosition(driverInput);
      case IDLE -> stopHood();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public boolean readyToShoot() {
    return hoodAtToleranceDebouncer.calculate(atTolerance);
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public Angle getHoodAngle() {
    return inputs.hoodMotorPosition;
  }

  private void stopHood() {
    outputs.outputMode = HoodIOOutputMode.STOP;
  }

  private void runPosition(Angle position) {
    outputs.outputMode = HoodIOOutputMode.POSITION;
    outputs.position = position;

    Angle error = inputs.hoodMotorPosition.minus(position);

    if (state == HoodState.SOTM || state == HoodState.FOTM) {
      atTolerance = error.abs(Degrees) < HoodSettings.HOOD_SOTM_TOLERANCE.in(Degrees);
    } else {
      atTolerance = error.abs(Degrees) < HoodSettings.HOOD_TOLERANCE.in(Degrees);
    }
  }

  private void runVoltage(Voltage voltage) {
    outputs.outputMode = HoodIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  private void hoodAnalogToInput(CommandXboxController gamepad) {
    double hoodMin = HoodSettings.HOOD_TOLERANCE.in(Degrees);
    double hoodMax = HoodSettings.HOOD_SOTM_TOLERANCE.in(Degrees);

    this.driverInput = Degrees.of(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2));
  }

  private boolean isStalling() {
    return hoodStallingDebouncer.calculate(
        inputs.hoodMotorStatorCurrent.gt(HoodSettings.STALL_CURRENT_LIMIT));
  }

  public void setState(HoodState state) {
    this.state = state;
  }

  public Command homeUpper() {
    return runOnce(() -> setState(HoodState.HOMING_UPPER)).withName("Hood Home Upper");
  }

  public Command homeLower() {
    return runOnce(() -> setState(HoodState.HOMING_LOWER)).withName("Hood Home Lower");
  }

  public Command hoodAnalog(CommandXboxController gamepad) {
    return runOnce(() -> setState(HoodState.ANALOG))
        .andThen(() -> hoodAnalogToInput(gamepad))
        .withName("Hood Analog");
  }

  public Command seedRelativeEncoderAtUpperHardstop() {
    return runOnce(() -> io.seedHoodPosition(HoodSettings.MAX_FROM_HORIZON))
        .withName("Hood Seed Relative Encoder at Upper Hardstop");
  }

  public Command seedRelativeEncoderAtLowerHardstop() {
    return runOnce(() -> io.seedHoodPosition(HoodSettings.MIN_FROM_HORIZON))
        .withName("Hood Seed Relative Encoder at Lower Hardstop");
  }
}
