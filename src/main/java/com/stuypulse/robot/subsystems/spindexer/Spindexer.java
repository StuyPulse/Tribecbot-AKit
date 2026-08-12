package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.GlobalSettings;
import static com.stuypulse.robot.subsystems.spindexer.SpindexerConstants.*;

import com.stuypulse.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.FullSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends FullSubsystem {
  private static final Spindexer instance;

  static {
    switch (GlobalSettings.currentMode) {
      case REAL -> instance = new Spindexer(new SpindexerIOTalonFX());

      case SIM -> instance = new Spindexer(new SpindexerIOSim());

      default -> instance = new Spindexer(new SpindexerIO() {});
    }
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs;
  private final SpindexerIOOutputs outputs;

  @AutoLogOutput(key = "States/Spindexer")
  private SpindexerState state;

  private Spindexer(SpindexerIO io) {
    this.io = io;
    this.inputs = new SpindexerIOInputsAutoLogged();
    this.outputs = new SpindexerIOOutputs();

    setState(SpindexerState.STOP);
  }

  public enum SpindexerState {
    FORWARD,
    REVERSE,
    STOP
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

    if (!GlobalSettings.EnabledSubsystems.SPINDEXER.get()) {
      stop();

      return;
    }

    if (Superstructure.getInstance().shouldStop()) {
      stop();

      return;
    }

    switch (state) {
      case FORWARD -> runDutyCycle(SpindexerSettings.FORWARD_DUTY_CYCLE);
      case REVERSE -> runDutyCycle(SpindexerSettings.REVERSE_DUTY_CYCLE);
      case STOP -> stop();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runDutyCycle(double dutyCycle) {
    outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.DUTY_CYCLE;
    outputs.spindexerLeaderDutyCycle = dutyCycle;
  }

  private void stop() {
    outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.STOP;
  }

  public void setState(SpindexerState state) {
    this.state = state;
  }

  public SpindexerState getState() {
    return state;
  }

  public Command runSpindexerForward() {
    return runOnce(() -> setState(SpindexerState.FORWARD)).withName("Spindexer Forward");
  }

  public Command runSpindexerReverse() {
    return runOnce(() -> setState(SpindexerState.REVERSE)).withName("Spindexer Reverse");
  }

  public Command stopSpindexer() {
    return runOnce(() -> setState(SpindexerState.STOP)).withName("Spindexer Stop");
  }
}
