package com.stuypulse.robot.subsystems.handoff;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.HandoffIO;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import com.stuypulse.robot.subsystems.handoff.HandoffIOSim;
import com.stuypulse.robot.subsystems.handoff.HandoffIOTalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase{
    private static final Handoff instance;

    static {
    if (Settings.currentMode == Settings.Mode.SIM) {
      instance = new Handoff(new HandoffIOSim());
    } else {
      instance = new Handoff(new HandoffIOTalonFX());
    }
  }

    public static Handoff getInstance() {
    return instance;
  }
  
    private final HandoffIO io;
    private final HandoffIOInputsAutoLogged inputs;
    private final HandoffIOOutputs outputs;

    private Handoff(HandoffIO io) {
        this.io = io;
        this.inputs = new HandoffIOInputsAutoLogged;
        this.outputs = new HandoffIOOutputs;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
  }

    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
  }
  
    private void stopMotors() {
        outputs.handoffDutyCycle = 0.0;
  }
}
