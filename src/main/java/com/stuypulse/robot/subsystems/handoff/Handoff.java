package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.HandoffIO;
import com.stuypulse.robot.subsystems.handoff.HandoffIOSim;
import com.stuypulse.robot.subsystems.handoff.HandoffIOTalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
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

    private final Debouncer handoffStallingDebouncer;

    private Handoff(HandoffIO io) {
        this.io = io;
        this.inputs = new HandoffIOInputsAutoLogged();
        this.outputs = new HandoffIOOutputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
  }

    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
  }

    private void runHandoffDutyCycle(double dutyCycle) {
      outputs.handoffDutyCycle = dutyCycle;
    }

    private boolean handoffStalling() {
    return handoffStallingDebouncer.calculate(
        inputs.motorLeadSupplyCurrent.abs(Amps) > Settings.Intake.PIVOT_STALL_CURRENT.in(Amps));
  }
  
    public Command runHandoff() {
      return run(
        () -> {
          runHandoffDutyCycle(1.0);
        })
        .until(this::handoffStalling());
    }

    public Command runReverseHandoff() {
      return run(
        () -> {
          runHandoffDutyCycle(-1.0);
        })
        .until(this::handoffStalling());
    }

    public Command runStopHandoff() {
      return run(
        () -> {
          runHandoffDutyCycle(0.0);
        });
    }
}
