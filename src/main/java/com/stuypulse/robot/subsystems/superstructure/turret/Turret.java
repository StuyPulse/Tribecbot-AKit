package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;
import com.stuypulse.robot.subsystems.swerve.Drive;

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

  private TurretState state;

  private boolean atTolerance;

  private final Debouncer readyToShootDebouncer;

  private Angle driverInput;

  public Turret(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();
    this.outputs = new TurretIOOutputs();

    setState(TurretState.SCORE);

    readyToShootDebouncer = new Debouncer(0.05, DebounceType.kBoth);
    atTolerance = false;
  }

  public enum TurretState {
    IDLE,
    ZERO,
    SCORE,
    SOTM,
    FOTM,
    FERRY,
    LEFT_CORNER,
    RIGHT_CORNER,
    KB,
    TESTING;
}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public boolean turretReadyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public Rotation2d getTurretYaw() {
    return Rotation2d.fromDegrees(inputs.turretMotorPosition.in(Degrees));
  }

  private void stopTurret() {
    outputs.turretMode = TurretIOOutputMode.STOP;
  }

  private void runPosition(Angle position) {
    outputs.turretPosition = position;
    outputs.turretPosition = position;

    Angle error = inputs.turretMotorPosition.minus(position);
    Drive swerve = Drive.getInstance();

    Angle tolerance = switch (state) {
        case SOTM -> 
            swerve.getTurretPose().getTranslation().getDistance(Field.HUB_CENTER.getTranslation()) > 
            Settings.Superstructure.Turret.SOTM_TOLERANCE_THRESHOLD_METERS.get() ?
            Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_CLOSE_DEG.get()):
            Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_FAR_DEG.get());
        case FOTM -> Settings.Superstructure.Turret.FOTM_TOLERANCE;
        default  -> Settings.Superstructure.Turret.TOLERANCE;
    };

    atTolerance = error.abs(Degrees) < tolerance.in(Degrees);
  }

  private void driverInputToAngle(CommandXboxController gamepad) {
    driverInput = Degrees.of(gamepad.getLeftX() * 180);
  }

  private void setState(TurretState state) {
    this.state = state;
  }
}
