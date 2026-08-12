package com.stuypulse.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants.SuperstructureSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.ShooterSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.RPMValues;

import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {
  private static final Shooter instance;

  static {
    switch (GlobalSettings.currentMode) {
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

  @AutoLogOutput(key = "States/Shooter")
  private ShooterState state;

  private final Debouncer readyToShootDebouncer;
  private final Debouncer currentlyShootingDebouncer;

  private boolean atTolerance;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
    this.outputs = new ShooterIOOutputs();

    setState(ShooterState.MANUAL_OVERRIDE);

    readyToShootDebouncer = new Debouncer(0.05, DebounceType.kBoth);
    currentlyShootingDebouncer = new Debouncer(2, DebounceType.kFalling);

    this.atTolerance = false;
  }

  public enum ShooterState {
    STOP,
    MANUAL_OVERRIDE,
    FERRY,
    REVERSE,
    KB,
    LEFT_CORNER,
    RIGHT_CORNER,
    INTERPOLATION,
    SOTM,
    FOTM;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (!GlobalSettings.EnabledSubsystems.SHOOTER.get()) {
      stopShooter();

      return;
    }

    switch (state) {
      case STOP -> stopShooter();
      case MANUAL_OVERRIDE -> runVelocity(
          RPM.of(RPMValues.MANUAL_OVERRIDE.get()));
      case FERRY -> runVelocity(InterpolationCalculator.getInterpolatedFerryRPM());
      case REVERSE -> runVelocity(RPMValues.REVERSE);
      case KB -> runVelocity(RPMValues.KB);
      case LEFT_CORNER -> runVelocity(RPMValues.LEFT_CORNER);
      case RIGHT_CORNER -> runVelocity(RPMValues.RIGHT_CORNER);
      case INTERPOLATION -> runVelocity(InterpolationCalculator.getInterpolatedShotRPM());
      case SOTM -> runVelocity(SOTMCalculator.calculateShooterRPMSOTM());
      case FOTM -> runVelocity(SOTMCalculator.calculateShooterRPMFOTM());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public AngularVelocity getShooterVelocity() {
    return inputs.shooterLeaderMotorVelocity;
  }

  private void stopShooter() {
    outputs.shooterMode = ShooterIOOutputMode.STOP;
  }

  private void runVelocity(AngularVelocity velocity) {
    outputs.shooterMode = ShooterIOOutputMode.VELOCITY;
    outputs.shooterVelocity = velocity;

    AngularVelocity error = inputs.shooterLeaderMotorVelocity.minus(velocity);

    AngularVelocity toleranceHigh =
        switch (state) {
          case SOTM -> SuperstructureSettings.SHOOTER_SOTM_TOLERANCE_RPM_HIGH;
          case FOTM -> SuperstructureSettings.SHOOTER_FOTM_TOLERANCE_RPM_HIGH;
          default -> SuperstructureSettings.SHOOTER_TOLERANCE_RPM_HIGH;
        };

    AngularVelocity toleranceLow =
        switch (state) {
          case SOTM -> SuperstructureSettings.SHOOTER_SOTM_TOLERANCE_RPM_LOW;
          case FOTM -> SuperstructureSettings.SHOOTER_FOTM_TOLERANCE_RPM_LOW;
          default -> SuperstructureSettings.SHOOTER_TOLERANCE_RPM_LOW;
        };

    atTolerance = error.lt(toleranceLow.unaryMinus()) && error.gt(toleranceHigh);
  }

  public boolean readyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public boolean isShooting() {
    return currentlyShootingDebouncer.calculate(
        inputs.shooterLeaderMotorStatorCurrent.gt(
            ShooterSettings.IS_SHOOTING_CURRENT));
  }

  public void setState(ShooterState state) {
    this.state = state;
  }
}
