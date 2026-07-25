package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Mode;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputs;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static final Shooter instance;

  static {
    if (Settings.currentMode == Mode.SIM) {
      instance = new Shooter(new ShooterIOSim());
    } else {
      instance = new Shooter(new ShooterIOTalonFX());
    }
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private final ShooterIOOutputs outputs;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
    this.outputs = new ShooterIOOutputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runVelocity(AngularVelocity velocity) {
    outputs.shooterVelocity = velocity;
  }
  
  public Command stopShooter() {
    return run(() -> runVelocity(RPM.zero()));
  }

  public Command runManualOverride() {
    return run(() -> runVelocity(RPM.of(Settings.Superstructure.Shooter.RPM.MANUAL_OVERRIDE.get())));
  }

  public Command runLeftCorner() {
    return run(() -> runVelocity(Settings.Superstructure.Shooter.RPM.LEFT_CORNER));
  }

  public Command runRightCorner() {
    return run(() -> runVelocity(Settings.Superstructure.Shooter.RPM.RIGHT_CORNER));
  }
}
