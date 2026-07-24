package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Mode;
import com.stuypulse.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private static final Spindexer instance;

  static {
    if (Settings.currentMode == Mode.SIM) {
      instance = new Spindexer(new SpindexerIOSim());
    } else {
      instance = new Spindexer(new SpindexerIOTalonFX());
    }
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs;
  private final SpindexerIOOutputs outputs;

  private Spindexer(SpindexerIO io) {
    this.io = io;
    this.inputs = new SpindexerIOInputsAutoLogged();
    this.outputs = new SpindexerIOOutputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runDutyCycle(double dutyCycle) {
    outputs.spindexerLeaderDutyCycle = dutyCycle;
  }

  public Command runSpindexerForward() {
    return run(() -> runDutyCycle(Settings.Spindexer.FORWARD_DUTY_CYCLE));
  }

  public Command runSpindexerReverse() {
    return run(() -> runDutyCycle(Settings.Spindexer.REVERSE_DUTY_CYCLE));
  }

  public Command stopSpindexer() {
    return run(() -> runDutyCycle(0));
  }
}
