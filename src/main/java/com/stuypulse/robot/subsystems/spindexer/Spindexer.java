package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private static final Spindexer instance;

  static {
    switch (Settings.currentMode) {
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
  }

  public void periodicAfterScheduler() {
    if (!Settings.EnabledSubsystems.SPINDEXER.get()) {
      stop();

      return;
    }

    switch (state) {
      case FORWARD -> runDutyCycle(Settings.Spindexer.FORWARD_DUTY_CYCLE);
      case REVERSE -> runDutyCycle(Settings.Spindexer.REVERSE_DUTY_CYCLE);
      case STOP -> stop();
    }

    Logger.recordOutput("States/Spindexer", state);
    Logger.recordOutput("Spindexer/Output Mode", outputs.spindexerMode);
    Logger.recordOutput("Spindexer/Duty Cycle Setpoint", outputs.spindexerLeaderDutyCycle);
    io.applyOutputs(outputs);
  }

  private void runDutyCycle(double dutyCycle) {
    outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.DUTY_CYCLE;
    outputs.spindexerLeaderDutyCycle = dutyCycle;
  }

  private void stop() {
    outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.STOP;
  }

  private void setState(SpindexerState state) {
    this.state = state;
  }

  public Command runSpindexerForward() {
    return run(() -> setState(SpindexerState.FORWARD)).withName("Spindexer Forward");
  }

  public Command runSpindexerReverse() {
    return run(() -> setState(SpindexerState.REVERSE)).withName("Spindexer Reverse");
  }

  public Command stopSpindexer() {
    return run(() -> setState(SpindexerState.STOP)).withName("Spindexer Stop");
  }
}
