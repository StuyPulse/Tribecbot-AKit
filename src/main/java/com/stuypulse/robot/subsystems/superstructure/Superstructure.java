package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.DriverConstants.Driver;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret.TurretState;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;

public class Superstructure extends FullSubsystem {

  private static final Superstructure instance;

  static {
    instance = new Superstructure();
  }

  public static Superstructure getInstance() {
    return instance;
  }

  private SuperstructureState cachedState;
  private SuperstructureState state;

  private final Timer sotmStoppedTimer;
  private final Timer fotmStoppedTimer;

  private final Hood hood;
  private final Shooter shooter;
  private final Turret turret;

  private Optional<Boolean> shouldStop;

  private Debouncer cachedStateIdleDebouncer;

  private Superstructure() {
    state = SuperstructureState.INTERPOLATION;

    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    turret = Turret.getInstance();

    sotmStoppedTimer = new Timer();
    sotmStoppedTimer.restart();
    sotmStoppedTimer.stop();

    fotmStoppedTimer = new Timer();
    fotmStoppedTimer.restart();
    fotmStoppedTimer.stop();

    cachedStateIdleDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    this.shouldStop = Optional.empty();
  }

  public enum SuperstructureState {
    STOW(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
    MANUAL_OVERRIDE(HoodState.MANUAL_OVERRIDE, ShooterState.MANUAL_OVERRIDE, TurretState.SCORE),
    FERRY(HoodState.FERRY, ShooterState.FERRY, TurretState.FERRY),
    FOTM(HoodState.FOTM, ShooterState.FOTM, TurretState.FOTM),
    REVERSE(HoodState.MANUAL_OVERRIDE, ShooterState.REVERSE, TurretState.SCORE),
    KB(HoodState.KB, ShooterState.KB, TurretState.KB),
    LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER, TurretState.LEFT_CORNER),
    RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER, TurretState.RIGHT_CORNER),
    INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION, TurretState.SCORE),
    AUTO_INTERPOLATION(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
    AUTO_INTERPOLATION_SOTM(HoodState.STOW, ShooterState.SOTM, TurretState.SOTM),
    SOTM(HoodState.SOTM, ShooterState.SOTM, TurretState.SOTM);

    private HoodState hoodState;
    private ShooterState shooterState;
    private TurretState turretState;

    private SuperstructureState(
        HoodState hoodState, ShooterState shooterState, TurretState TurretState) {
      this.hoodState = hoodState;
      this.shooterState = shooterState;
      this.turretState = TurretState;
    }

    public HoodState getHoodState() {
      return hoodState;
    }

    public ShooterState getShooterState() {
      return shooterState;
    }

    public TurretState getTurretState() {
      return turretState;
    }
  }

  private void setState(SuperstructureState state) {
    this.state = state;
    hood.setState(state.getHoodState());
    shooter.setState(state.getShooterState());
    turret.setState(state.getTurretState());
  }

  public SuperstructureState getState() {
    return state;
  }

  public boolean shouldStop() {
    if (!shouldStop.isEmpty()) {
      return shouldStop.get();
    }
    Drive swerve = Drive.getInstance();

    boolean isSpindexerStopState = Spindexer.getInstance().getState() == SpindexerState.STOP;
    boolean isHandOffStopState = Handoff.getInstance().getState() == HandoffState.STOP;

    boolean isBehindHubWhileFerrying = state == SuperstructureState.FOTM && swerve.isBehindHub();
    boolean isOutsideAllianceZone =
        Drive.getInstance().isOutsideAllianceZone() && state != SuperstructureState.FOTM;
    boolean isUnderTrench =
        Drive.getInstance().isUnderTrench() && state != SuperstructureState.FOTM;
    boolean inManualState =
        state == SuperstructureState.LEFT_CORNER
            && state == SuperstructureState.RIGHT_CORNER
            && state == SuperstructureState.KB;
    boolean isBehindTower = swerve.isBehindTower() && state == SuperstructureState.SOTM;
    boolean isBtwnOppHubAndWall = swerve.isBtwnOppHubAndWall() && state == SuperstructureState.FOTM;

    boolean turretLaggingSOTM = !turret.atTolerance() && state == SuperstructureState.SOTM;
    boolean turretLaggingFOTM = turret.isTurretLaggingFOTM();

    boolean shouldStop =
        isSpindexerStopState
            || isHandOffStopState
            || (isBehindHubWhileFerrying && !inManualState)
            || isBtwnOppHubAndWall
            || turretLaggingSOTM
            || turretLaggingFOTM
            || (isOutsideAllianceZone && !inManualState)
            || (isUnderTrench && !inManualState)
            || isBehindTower;

    this.shouldStop = Optional.of(shouldStop);

    return shouldStop;
  }

  public boolean isReadyToShoot() {
    return hood.readyToShoot() && shooter.readyToShoot() && turret.readyToShoot();
  }

  public boolean atTolerance() {
    return hood.atTolerance() && shooter.atTolerance() && turret.atTolerance();
  }

  public void clearMemoized() {
    this.shouldStop = Optional.empty();
  }

  public boolean isHopperEmpty() {
    return !shooter.isShooting();
  }

  @Override
  public void periodicAfterScheduler() {
    if (state == SuperstructureState.SOTM && shouldStop() && DriverStation.isEnabled()) {
      sotmStoppedTimer.start();
    } else if (state == SuperstructureState.FOTM && shouldStop() && DriverStation.isEnabled()) {
      fotmStoppedTimer.start();
    }

    if (state != SuperstructureState.SOTM) sotmStoppedTimer.stop();
    if (state != SuperstructureState.FOTM) fotmStoppedTimer.stop();

    if (!shouldStop() || DriverStation.isDisabled()) {
      sotmStoppedTimer.stop();
      fotmStoppedTimer.stop();
    }

    if (Drive.getInstance().isOutsideAllianceZone()
        && state == SuperstructureState.SOTM
        && !DriverStation
            .isAutonomous()) { // allows us to start SOTM earlier in auto, but currently not desired
      // in teleop
      setState(SuperstructureState.STOW);
      Spindexer.getInstance().setState(SpindexerState.STOP);
      Handoff.getInstance().setState(HandoffState.STOP);
    }
  }

  private Command setStateCommand(SuperstructureState state) {
    return runOnce(() -> setState(state));
  }

  public Command autoInterpolation() {
    return setStateCommand(SuperstructureState.AUTO_INTERPOLATION)
        .withName("Superstructure Auto Interpolation");
  }

  public Command autoInterpolationSOTM() {
    return setStateCommand(SuperstructureState.AUTO_INTERPOLATION_SOTM)
        .withName("Superstructure Auto Interpolation SOTM");
  }

  public Command FOTM() {
    return setStateCommand(SuperstructureState.FOTM).withName("Superstructure FOTM");
  }

  public Command ferry() {
    return setStateCommand(SuperstructureState.FERRY).withName("Superstructure Ferry");
  }

  public Command interpolation() {
    return setStateCommand(SuperstructureState.INTERPOLATION)
        .withName("Superstructure Interpolation");
  }

  public Command kb() {
    return setStateCommand(SuperstructureState.KB).withName("Superstructure KB");
  }

  public Command leftCorner() {
    return setStateCommand(SuperstructureState.LEFT_CORNER).withName("Superstructure Left Corner");
  }

  public Command manualOverride() {
    return setStateCommand(SuperstructureState.MANUAL_OVERRIDE)
        .withName("Superstructure Manual Override");
  }

  public Command reverse() {
    return setStateCommand(SuperstructureState.REVERSE).withName("Superstructure Reverse");
  }

  public Command rightCorner() {
    return setStateCommand(SuperstructureState.RIGHT_CORNER)
        .withName("Superstructure Right Corner");
  }

  public Command SOTM() {
    return setStateCommand(SuperstructureState.SOTM).withName("Superstructure SOTM");
  }

  public Command stow() {
    return setStateCommand(SuperstructureState.STOW).withName("Superstructure Stow");
  }

  public Command cacheState(CommandXboxController driver) {
    return Commands.startEnd(
            () -> {
              this.cachedState = state;
              setState(SuperstructureState.INTERPOLATION);
              Drive.getInstance().stopWithX();
            },
            () -> setState(cachedState),
            this,
            Drive.getInstance())
        .until(
            () -> {
              double driverInputMagnitude =
                  new Translation2d(-driver.getLeftY(), -driver.getLeftX()).getNorm();

              return cachedStateIdleDebouncer.calculate(
                  driverInputMagnitude <= Driver.Drive.DEADBAND
                      && Math.abs(driver.getRightX()) <= Turn.DEADBAND);
            })
        .withName("Superstructure Cache State");
  }
}
