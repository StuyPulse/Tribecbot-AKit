package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private static final Turret instance;

  static {
    switch (GlobalSettings.currentMode) {
      case REAL -> instance = new Turret(new TurretIOReal());

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

  @AutoLogOutput(key = "States/Turret")
  private TurretState state;

  private boolean atTolerance;
  private boolean lagging;

  private boolean hasUsedAbsoluteEncoder;
  private boolean hasInitializedFilter;
  private boolean zeroingEncoders;
  private boolean hasRefreshedEncoderMagnetOffsets;

  private double prevActualTargetAngle;
  private boolean isWrapping;

  private final Debouncer readyToShootDebouncer;

  private Angle driverInput;

  public Turret(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();
    this.outputs = new TurretIOOutputs();

    setState(TurretState.SCORE);

    readyToShootDebouncer = new Debouncer(0.05, DebounceType.kBoth);
    atTolerance = false;

    prevActualTargetAngle = getScoringAngle().in(Degrees);

    hasUsedAbsoluteEncoder = false;
    hasInitializedFilter = false;
    zeroingEncoders = false;
    hasRefreshedEncoderMagnetOffsets = false;
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

  private Angle getScoringAngle() {
    Drive swerve = Drive.getInstance();

    Translation2d target = Field.HUB_CENTER.getTranslation();
    Translation2d turret = getTurretPose().getTranslation();

    return TurretAngleCalculator.getPointAtTargetAngle(
        target, turret, swerve.getPose().getRotation());
  }

  private Angle getFerryAngle() {
    Drive swerve = Drive.getInstance();

    Pose2d robot = swerve.getPose();
    Translation2d target = Field.getFerryZonePose(robot.getTranslation()).getTranslation();
    Translation2d turret = getTurretPose().getTranslation();

    return TurretAngleCalculator.getPointAtTargetAngle(target, turret, robot.getRotation());
  }

  private double getWrappedTargetAngle(Angle targetAngle) {
    double currentAngle = inputs.turretMotorPosition.in(Degrees);
    return currentAngle + getDelta(targetAngle.in(Degrees), currentAngle);
  }

  private double getDelta(double target, double current) {
    double delta = (target - current) % 360;

    if (delta > 180.0) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }

    if (current + delta > TurretSettings.RANGE_CW) {
      return delta - 360;
    }
    if (current + delta < TurretSettings.RANGE_CCW) {
      return delta + 360;
    }

    return delta;
  }

  @AutoLogOutput(key = "Superstructure/Turret/Absolute Angle")
  private Angle getVectorSpaceAngle() {
    return TurretAngleCalculator.getAbsoluteAngle(
        inputs.encoder17tPosition.in(Degrees), inputs.encoder18tPosition.in(Degrees));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (!GlobalSettings.EnabledSubsystems.TURRET.get()) {
      stopTurret();

      return;
    }

    switch (state) {
      case IDLE -> runPosition(inputs.turretMotorPosition);
      case ZERO -> runPosition(Degrees.zero());
      case SCORE -> runPosition(getScoringAngle());
      case SOTM -> runPosition(SOTMCalculator.calculateTurretAngleSOTM());
      case FOTM -> runPosition(SOTMCalculator.calculateTurretAngleFOTM());
      case FERRY -> runPosition(getFerryAngle());
      case LEFT_CORNER -> runPosition(TurretAngles.LEFT_CORNER);
      case RIGHT_CORNER -> runPosition(TurretAngles.RIGHT_CORNER);
      case KB -> runPosition(TurretAngles.KB);
      case TESTING -> runPosition(driverInput);
    }
    ;
  }

  @Override
  public void periodicAfterScheduler() {
    if (zeroingEncoders && hasRefreshedEncoderMagnetOffsets) {
      double currentOffset17T = inputs.encoder17tMagnetOffset;
      double currentOffset18T = inputs.encoder18tMagnetOffset;

      double newOffset17T = currentOffset17T - inputs.encoder17tPosition.in(Rotations);
      double newOffset18T = currentOffset18T - inputs.encoder18tPosition.in(Rotations);

      io.reconfigureEncoderMagnetOffsets(newOffset17T, newOffset18T);

      zeroingEncoders = false;
      hasRefreshedEncoderMagnetOffsets = false;
    } else if (zeroingEncoders && !hasRefreshedEncoderMagnetOffsets) {
      io.refreshEncoderMagnetSensorConfigurations();
      hasRefreshedEncoderMagnetOffsets = true;
    }

    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Superstructure/Turret/Ready To Shoot")
  public boolean readyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public Rotation2d getTurretYaw() {
    return new Rotation2d(inputs.turretMotorPosition);
  }

  public Pose2d getTurretPose() {
    Transform2d turretTransform = new Transform2d(TurretSettings.TURRET_OFFSET, getTurretYaw());

    return Drive.getInstance().getPose().transformBy(turretTransform);
  }

  public boolean isWrapping() {
    return isWrapping;
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public boolean isTurretLaggingFOTM() {
    return lagging && state == TurretState.FOTM;
  }

  private void seedTurret() {
    io.seedTurretPosition(getVectorSpaceAngle());
  }

  private void stopTurret() {
    outputs.turretMode = TurretIOOutputMode.STOP;
  }

  private void zeroEncoders() {
    zeroingEncoders = true;
  }

  private void runPosition(Angle position) {
    if (!hasUsedAbsoluteEncoder) {
      seedTurret();
      hasUsedAbsoluteEncoder = true;
    }

    double currentAngle = inputs.turretMotorPosition.in(Degrees);
    double actualTargetAngle = currentAngle + getDelta(position.in(Degrees), currentAngle);

    if (!hasInitializedFilter) {
      prevActualTargetAngle = actualTargetAngle;
      hasInitializedFilter = true;
    }

    double delta = actualTargetAngle - prevActualTargetAngle;

    boolean deltaIsSignificant = Math.abs(delta) >= TurretSettings.SETPOINT_FILTER_THRESHOLD_DEG;

    boolean driverIsMoving =
        Math.abs(RobotContainer.driver.getLeftX()) > DriverConstants.Driver.Drive.DEADBAND
            || Math.abs(RobotContainer.driver.getLeftY()) > DriverConstants.Driver.Drive.DEADBAND
            || Math.abs(RobotContainer.driver.getRightX()) > DriverConstants.Driver.Drive.DEADBAND;

    if (deltaIsSignificant || driverIsMoving) {
      prevActualTargetAngle = actualTargetAngle;
    }

    if (isWrapping) {
      isWrapping =
          Math.abs(getWrappedTargetAngle(position) - currentAngle)
              > TurretSettings.GAIN_SWITCHING_THRESHOLD_END.in(Degrees);
    } else {
      isWrapping =
          Math.abs(getWrappedTargetAngle(position) - currentAngle)
              > TurretSettings.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
    }

    int slot = 0;

    if (isWrapping) {
      slot = 1;
    }

    double omega = Drive.getInstance().getChassisSpeeds().omegaRadiansPerSecond;
    double omegaFF = TurretGains.kOmega.get() * omega;
    double setpointVelocityRPS = delta / (360 * GlobalSettings.DT.in(Seconds));

    // the component of the turret's setpoint velocity that comes from robot translation
    double translationalComponentVelocityRPS = setpointVelocityRPS - omega / (2 * Math.PI);
    double translationFF = TurretGains.kTranslation.get() * translationalComponentVelocityRPS;

    outputs.turretMode = TurretIOOutputMode.POSITION;
    outputs.turretPosition = Degrees.of(prevActualTargetAngle);
    outputs.gainSlot = slot;
    outputs.feedForward = omegaFF + translationFF;

    // At Tolerance Calculation
    Angle error = inputs.turretMotorPosition.minus(position);

    Angle tolerance =
        switch (state) {
          case SOTM -> getTurretPose()
                      .getTranslation()
                      .getDistance(Field.HUB_CENTER.getTranslation())
                  > TurretSettings.SOTM_TOLERANCE_THRESHOLD_METERS.get()
              ? Degrees.of(TurretSettings.SOTM_TOLERANCE_CLOSE_DEG.get())
              : Degrees.of(TurretSettings.SOTM_TOLERANCE_FAR_DEG.get());
          case FOTM -> TurretSettings.FOTM_TOLERANCE;
          default -> TurretSettings.TOLERANCE;
        };

    atTolerance = error.abs(Degrees) < tolerance.in(Degrees);
    lagging = error.abs(Degrees) >= TurretSettings.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
  }

  private void setDriverInput(CommandXboxController gamepad) {
    driverInput = Degrees.of(gamepad.getLeftX() * 180);
  }

  public void setState(TurretState state) {
    this.state = state;
  }

  public Command seedTurretCommand() {
    return runOnce(this::seedTurret).ignoringDisable(true).withName("Seed Turret");
  }

  public Command turretAnalog(CommandXboxController gamepad) {
    return runOnce(() -> setState(TurretState.TESTING))
        .andThen(() -> setDriverInput(gamepad))
        .withName("Turret Analog");
  }

  public Command turretFerry() {
    return runOnce(() -> setState(TurretState.FERRY)).withName("Turret Ferry");
  }

  public Command turretIdle() {
    return runOnce(() -> setState(TurretState.IDLE)).withName("Turret Idle");
  }

  public Command turretLeftCorner() {
    return runOnce(() -> setState(TurretState.LEFT_CORNER)).withName("Turret Left Corner");
  }

  public Command turretRightCorner() {
    return runOnce(() -> setState(TurretState.RIGHT_CORNER)).withName("Turret Right Corner");
  }

  public Command turretShoot() {
    return runOnce(() -> setState(TurretState.SCORE)).withName("Turret Shoot");
  }

  public Command zeroTurret() {
    return runOnce(this::zeroEncoders)
        .andThen(Commands.waitUntil(() -> zeroingEncoders == false))
        .andThen(runOnce(this::seedTurret))
        .withName("Zero Turret")
        .ignoringDisable(true);
  }
}
