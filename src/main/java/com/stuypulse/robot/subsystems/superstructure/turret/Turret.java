package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private static final Turret instance;
  private Translation2d driverInput;

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new Turret(new TurretIOTalonFX());

      case SIM -> instance = new Turret(new TurretIOSim());

      default -> instance = new Turret(new TurretIO() {});
    }
  }

  public static Turret getInstance() {
    return instance;
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;
  private final TurretIOOutputs outputs;

  private boolean OTM;
  private boolean atTolerance;

  private final Debouncer readyToShootDebouncer;

  public Turret(TurretIO io) {
    driverInput = Translation2d.kZero;

    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();
    this.outputs = new TurretIOOutputs();

    readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    OTM = false;
    atTolerance = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public boolean atTolerance() {
    Angle error = inputs.turretMotorPosition.minus(outputs.turretPosition);

    if (Robot.isReal()) {
      if (OTM) {
        return error.abs(Degrees) < Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_HIGH;
      } else {
        return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
      }
    } else {
      return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW;
    }
  }

  public boolean turretReadyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public Rotation2d getTurretYaw() {
    return Rotation2d.fromDegrees(inputs.turretMotorPosition.in(Degrees));
  }

  private void runPosition(Angle position, boolean OTM) {
    this.OTM = OTM;

    outputs.turretPosition = position;
  }

  private Angle driverInputToAngle() {
    Logger.recordOutput("Superstructure/Input/Driver Output", driverInput.getX());
    return Degrees.of(driverInput.getX() * 180);
  }

  public Command runFerry() {
    return run(() -> runPosition(Settings.Superstructure.Turret.FOTM_TOLERANCE, OTM));
  }

  public Command runLeftCorner() {
    return run(() -> runPosition(Settings.Superstructure.Turret.LEFT_CORNER, OTM));
  }

  public Command runRightCorner() {
    return run(() -> runPosition(Settings.Superstructure.Turret.RIGHT_CORNER, OTM));
  }

  public Command runKB() {
    return run(() -> runPosition(Settings.Superstructure.Turret.KB, OTM));
  }

  public Command runShoot() {
    return run(
        () ->
            runPosition(
                Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_FAR.getAsDouble()), OTM));
  }

  public Command runIdle() {
    return run(() -> runPosition(Degrees.of(0), OTM));
  }

  public Command runAnalog(CommandXboxController gamepad) {
    return run(() -> runPosition(driverInputToAngle(), OTM));
  }
}
