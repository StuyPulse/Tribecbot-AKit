package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.AutoLogOutput;
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

  @AutoLogOutput(key = "States/Turret")
  private TurretState state;

  private boolean atTolerance;
  private boolean lagging;

  private boolean hasUsedAbsoluteEncoder;
  private boolean hasInitializedFilter;

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
    Translation2d turret = swerve.getTurretPose().getTranslation();

    return TurretAngleCalculator.getPointAtTargetAngle(
        target, turret, swerve.getPose().getRotation());
  }

  private Angle getFerryAngle() {
    Drive swerve = Drive.getInstance();

    Pose2d robot = swerve.getPose();
    Translation2d target = Field.getFerryZonePose(robot.getTranslation()).getTranslation();
    Translation2d turret = swerve.getTurretPose().getTranslation();

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

    if (current + delta > Settings.Superstructure.Turret.RANGE_CW) {
      return delta - 360;
    }
    if (current + delta < Settings.Superstructure.Turret.RANGE_CCW) {
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

    if (!Settings.EnabledSubsystems.TURRET.get()) {
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
      case LEFT_CORNER -> runPosition(Settings.Superstructure.Turret.LEFT_CORNER);
      case RIGHT_CORNER -> runPosition(Settings.Superstructure.Turret.RIGHT_CORNER);
      case KB -> runPosition(Settings.Superstructure.Turret.KB);
      case TESTING -> runPosition(driverInput);
    }
    ;
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Superstructure/Turret/Ready To Shoot")
  public boolean turretReadyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public Rotation2d getTurretYaw() {
    return new Rotation2d(inputs.turretMotorPosition);
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
    double encoderPos17T = inputs.encoder17tPosition.in(Rotations);
    double encoderPos18T = inputs.encoder18tPosition.in(Rotations);

    io.refreshMagnetSensorConfigurations(
        Motors.Superstructure.Turret.ENCODER_17T_CONFIG.getConfiguration().MagnetSensor,
        Motors.Superstructure.Turret.ENCODER_18T_CONFIG.getConfiguration().MagnetSensor);

    double currentOffset17T =
        Motors.Superstructure.Turret.ENCODER_17T_CONFIG.getConfiguration()
            .MagnetSensor
            .MagnetOffset;
    double currentOffset18T =
        Motors.Superstructure.Turret.ENCODER_18T_CONFIG.getConfiguration()
            .MagnetSensor
            .MagnetOffset;

    double newOffset17T = currentOffset17T - encoderPos17T;
    double newOffset18T = currentOffset18T - encoderPos18T;

    Motors.Superstructure.Turret.ENCODER_17T_CONFIG.withMagnetOffset(newOffset17T);
    Motors.Superstructure.Turret.ENCODER_18T_CONFIG.withMagnetOffset(newOffset18T);

    io.configureEncoders();
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

    boolean deltaIsSignificant =
        Math.abs(delta) >= Settings.Superstructure.Turret.SETPOINT_FILTER_THRESHOLD_DEG;

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
              > Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD_END.in(Degrees);
    } else {
      isWrapping =
          Math.abs(getWrappedTargetAngle(position) - currentAngle)
              > Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
    }

    int slot = 0;

    if (isWrapping) {
      slot = 1;
    }

    double omega = Drive.getInstance().getChassisSpeeds().omegaRadiansPerSecond;
    double omegaFF = Gains.Superstructure.Turret.kOmega.get() * omega;
    double setpointVelocityRPS = delta / (360 * Settings.DT);

    // the component of the turret's setpoint velocity that comes from robot translation
    double translationalComponentVelocityRPS = setpointVelocityRPS - omega / (2 * Math.PI);
    double translationFF =
        Gains.Superstructure.Turret.kTranslation.get() * translationalComponentVelocityRPS;

    outputs.turretMode = TurretIOOutputMode.POSITION;
    outputs.turretPosition = Degrees.of(prevActualTargetAngle);
    outputs.gainSlot = slot;
    outputs.feedForward = omegaFF + translationFF;

    // At Tolerance Calculation
    Angle error = inputs.turretMotorPosition.minus(position);
    Drive swerve = Drive.getInstance();

    Angle tolerance =
        switch (state) {
          case SOTM -> swerve
                      .getTurretPose()
                      .getTranslation()
                      .getDistance(Field.HUB_CENTER.getTranslation())
                  > Settings.Superstructure.Turret.SOTM_TOLERANCE_THRESHOLD_METERS.get()
              ? Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_CLOSE_DEG.get())
              : Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_FAR_DEG.get());
          case FOTM -> Settings.Superstructure.Turret.FOTM_TOLERANCE;
          default -> Settings.Superstructure.Turret.TOLERANCE;
        };

    atTolerance = error.abs(Degrees) < tolerance.in(Degrees);
    lagging =
        error.abs(Degrees)
            >= Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
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
    return runOnce(
            () -> {
              zeroEncoders();
              seedTurret();
            })
        .withName("Zero Turret")
        .ignoringDisable(true);
  }
}
