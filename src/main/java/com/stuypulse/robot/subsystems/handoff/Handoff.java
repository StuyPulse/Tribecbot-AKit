package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.Amps;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputMode;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Handoff extends SubsystemBase {
  private static final Handoff instance;

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new Handoff(new HandoffIOTalonFX());

      case SIM -> instance = new Handoff(new HandoffIOSim());

      default -> instance = new Handoff(new HandoffIO() {});
    }
  }

  public static Handoff getInstance() {
    return instance;
  }

  private final HandoffIO io;
  private final HandoffIOInputsAutoLogged inputs;
  private final HandoffIOOutputs outputs;

  private HandoffState state;

  private final Debouncer handoffStallingDebouncer;

  private Handoff(HandoffIO io) {
    this.io = io;
    this.inputs = new HandoffIOInputsAutoLogged();
    this.outputs = new HandoffIOOutputs();

    setState(HandoffState.STOP);

    this.handoffStallingDebouncer =
        new Debouncer(Settings.Handoff.HANDOFF_STALL_DEBOUNCE_SEC, DebounceType.kBoth);
  }

  public enum HandoffState {
    FORWARD,
    REVERSE,
    STOP
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Handoff", inputs);

    if (!Settings.EnabledSubsystems.HANDOFF.get()) {
      stopHandoff();

      return;
    }

    switch (state) {
      case FORWARD -> runHandoffDutyCycle(1);
      case REVERSE -> runHandoffDutyCycle(-1);
      case STOP -> stopHandoff();
    }
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Handoff/Output Mode", outputs.handoffMode);
    Logger.recordOutput("Handoff/Duty Cycle Setpoint", outputs.handoffDutyCycle);
    io.applyOutputs(outputs);
  }

  private void runHandoffDutyCycle(double dutyCycle) {
    outputs.handoffMode = HandoffIOOutputMode.DUTY_CYCLE;
    outputs.handoffDutyCycle = dutyCycle;
  }

  private void stopHandoff() {
    outputs.handoffMode = HandoffIOOutputMode.STOP;
  }

  public boolean isHandoffStalling() {
    return handoffStallingDebouncer.calculate(
        inputs.motorLeadSupplyCurrent.abs(Amps) > Settings.Handoff.HANDOFF_STALL_CURRENT.in(Amps));
  }

  private void setState(HandoffState state) {
    this.state = state;
  }

  public Command runHandoffForward() {
    return runOnce(() -> setState(HandoffState.FORWARD)).withName("Handoff Forward");
  }

  public Command runHandoffReverse() {
    return runOnce(() -> setState(HandoffState.REVERSE)).withName("Handoff Reverse");
  }

  public Command stopHandoffCommand() {
    return runOnce(() -> setState(HandoffState.STOP)).withName("Handoff Stop");
  }
}
