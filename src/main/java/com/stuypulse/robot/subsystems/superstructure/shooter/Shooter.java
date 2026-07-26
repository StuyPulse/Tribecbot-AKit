package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final Shooter instance;

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new Shooter(new ShooterIOTalonFX());

      case SIM -> instance = new Shooter(new ShooterIOSim());

      default -> instance = new Shooter(new ShooterIO() {});
    }
  }

  public static Shooter getInstance() {
    return instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private final ShooterIOOutputs outputs;

  private final Debouncer readyToShootDebouncer;
  private boolean atTolerance;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
    this.outputs = new ShooterIOOutputs();

    readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    atTolerance = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Shooter/Velocity Setpoint", outputs.shooterVelocity);
    io.applyOutputs(outputs);
  }

  private void runVelocity(AngularVelocity velocity) {
    outputs.shooterVelocity = velocity;
  }

  public boolean readyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public Command stopShooter() {
    return run(() -> runVelocity(RPM.zero()));
  }

  // Anything that isn't SOTM or FOTM
  private Command runManual(DoubleSupplier rpmSupplier) {
    return run(
        () -> {
          double targetRPM = rpmSupplier.getAsDouble();

          runVelocity(RPM.of(targetRPM));
          double error = inputs.shooterLeaderMotorVelocity.in(RPM) - targetRPM;

          atTolerance =
              error > -Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW
                  && error < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
        });
  }

  public Command runManualOverride() {
    return runManual(Settings.Superstructure.Shooter.RPM.MANUAL_OVERRIDE::get);
  }

  public Command runLeftCorner() {
    return runManual(() -> Settings.Superstructure.Shooter.RPM.LEFT_CORNER.in(RPM));
  }

  public Command runRightCorner() {
    return runManual(() -> Settings.Superstructure.Shooter.RPM.RIGHT_CORNER.in(RPM));
  }

  public Command runReverse() {
    return runManual(() -> Settings.Superstructure.Shooter.RPM.REVERSE.in(RPM));
  }
}
