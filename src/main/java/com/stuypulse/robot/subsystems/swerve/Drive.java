// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Mode;
import com.stuypulse.robot.generated.TunerConstants;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.LocalADStarAK;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends FullSubsystem implements Vision.VisionConsumer {
  private static final Drive instance;
  private static SwerveDriveSimulation driveSimulation = null;

  private Optional<Boolean> isBehindHub = Optional.empty();
  private Optional<Boolean> isOutsideAllianceZone = Optional.empty();
  private Optional<Boolean> isInOpponentZone = Optional.empty();
  private Optional<Boolean> isUnderTrench = Optional.empty();
  private Optional<Boolean> isBehindTower = Optional.empty();
  private Optional<Boolean> isBtwnOppHubAndWall = Optional.empty();

  static {
    switch (Settings.currentMode) {
      case REAL -> {
        instance =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});
      }

      case SIM -> {
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        instance =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
      }

        // For replay mode
      default -> {
        instance =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
      }
    }
  }

  public static Drive getInstance() {
    return instance;
  }

  public static SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  public boolean isUnderTrench() {
    if (isUnderTrench.isEmpty()) {
      Translation2d turretTranslation = Turret.getInstance().getTurretPose().getTranslation();

      boolean isBetweenRightTrenchesY =
          Field.AllianceRightTrench.rightEdge.getY() < turretTranslation.getY()
              && Field.AllianceRightTrench.leftEdge.getY() > turretTranslation.getY();

      boolean isBetweenLeftTrenchesY =
          Field.AllianceLeftTrench.rightEdge.getY() < turretTranslation.getY()
              && Field.AllianceLeftTrench.leftEdge.getY() > turretTranslation.getY();

      boolean isUnderAllianceTrenchX =
          Math.abs(turretTranslation.getX() - Field.AllianceRightTrench.rightEdge.getX())
              < Field.TRENCH_HOOD_TOLERANCE.in(Meters);

      boolean isUnderOpponentTrenchX =
          Math.abs(turretTranslation.getX() - Field.OpponentRightTrench.rightEdge.getX())
              < Field.TRENCH_HOOD_TOLERANCE.in(Meters);

      boolean isUnderTrench =
          (isBetweenRightTrenchesY || isBetweenLeftTrenchesY)
              && (isUnderAllianceTrenchX || isUnderOpponentTrenchX);

      this.isUnderTrench = Optional.of(isUnderTrench);
    }

    return isUnderTrench.get();
  }

  public boolean isInOpponentZone() {
    if (isInOpponentZone.isEmpty()) {
      Translation2d turretTranslation = Turret.getInstance().getTurretPose().getTranslation();
      isInOpponentZone = Optional.of(turretTranslation.getMeasureX().gt(Field.OPPONENT_ZONE_X));
    }
    return isInOpponentZone.get();
  }

  public boolean isBehindTower() {
    if (isBehindTower.isEmpty()) {
      boolean withinTowerX = getPose().getTranslation().getX() < Field.TOWER_FAR_CENTER.getX();
      boolean withinTowerY =
          Field.TOWER_FAR_RIGHT.getY()
                  < Turret.getInstance().getTurretPose().getTranslation().getY()
              && Turret.getInstance().getTurretPose().getTranslation().getY()
                  < Field.TOWER_FAR_LEFT.getY();
      isBehindTower = Optional.of(withinTowerX && withinTowerY);
    }

    return isBehindTower.get();
  }

  /**
   * Checks whether the robot turret pose is behind the hub or not in the neutral zone. For stopping
   * ferrying to prevent fuel hitting the hub.
   *
   * @return true if robot turret pose is behind the hub.
   */
  public boolean isBehindHub() {
    if (isBehindHub.isEmpty()) {
      // === TRIANGLE === (CUSTOM VERTEX)
      Translation2d turretTranslation = Turret.getInstance().getTurretPose().getTranslation();

      boolean behindHubX = Field.HUB_FAR_LEFT_CORNER.getX() < turretTranslation.getX();
      // && turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX; // With
      // this line the triangle will be cut to more like a trapezoid.

      Pose2d hubFarLeftCornerWithTolerance =
          new Pose2d(
              Field.HUB_FAR_LEFT_CORNER.getMeasureX(),
              Field.HUB_FAR_LEFT_CORNER.getMeasureY().plus(Field.BEHIND_HUB_TOLERANCE_Y),
              new Rotation2d());
      Pose2d hubFarRightCornerWithTolerance =
          new Pose2d(
              Field.HUB_FAR_RIGHT_CORNER.getMeasureX(),
              Field.HUB_FAR_RIGHT_CORNER.getMeasureY().minus(Field.BEHIND_HUB_TOLERANCE_Y),
              new Rotation2d());

      // Find point on triangle using the point-slope formula (of the line constructed by the hub
      // corner pose and ferry pose)
      // y = (slope)(robotX - hubCornerX) + (hubCornerY)
      // where the slope = (hubCornerY - vertexY)/(hubCornerX - vertexX)
      double leftY =
          ((hubFarLeftCornerWithTolerance.getY() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getY())
                      / (hubFarLeftCornerWithTolerance.getX()
                          - Field.BEHIND_HUB_TRIANGLE_VERTEX.getX())) // (Slope)
                  * (turretTranslation.getX() - hubFarLeftCornerWithTolerance.getX())
              + hubFarLeftCornerWithTolerance.getY(); // *(robotX - hubCornerX) + (hubCornerY)
      double rightY =
          ((hubFarRightCornerWithTolerance.getY() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getY())
                      / (hubFarRightCornerWithTolerance.getX()
                          - Field.BEHIND_HUB_TRIANGLE_VERTEX.getX())) // (Slope)
                  * (turretTranslation.getX() - hubFarRightCornerWithTolerance.getX())
              + hubFarRightCornerWithTolerance.getY(); // *(robotX - hubCornerX) + (hubCornerY)

      // Debug:
      // leftBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), leftY, new Rotation2d()));
      // rightBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), rightY, new
      // Rotation2d()));
      // vertexBehindHubPublisher.set(Field.BEHIND_HUB_TRIANGLE_VERTEX);

      boolean withinHubY =
          rightY < Turret.getInstance().getTurretPose().getY()
              && Turret.getInstance().getTurretPose().getY() < leftY;

      isBehindHub = Optional.of(behindHubX && withinHubY);

      // === TRIANGLE === (FROM FERRY ZONES):
      // Translation2d turretTranslation = getTurretPose().getTranslation();
      // boolean behindHubX = Field.hubFarLeftCorner.getX() < turretTranslation.getX();
      // 		// && turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX; //
      // With this line the triangle will be cut to more like a trapezoid.
      // // Find point on triangle using the point-slope formula (of the line constructed by the hub
      // corner pose and ferry pose)
      // // y = (slope)(robotX - hubCornerX) + (hubCornerY)
      // // where the slope = (hubCornerY - ferryY)/(hubCornerX - ferryX)
      // double leftY = ((Field.hubFarLeftCorner.getY() -
      // Field.leftFerryZone.getY())/(Field.hubFarLeftCorner.getX() - Field.leftFerryZone.getX()))
      // // (Slope)
      // 				* (turretTranslation.getX() - Field.hubFarLeftCorner.getX()) +
      // Field.hubFarLeftCorner.getY(); // *(robotX - hubCornerX) + (hubCornerY)
      // double rightY = ((Field.hubFarRightCorner.getY() -
      // Field.rightFerryZone.getY())/(Field.hubFarRightCorner.getX() -
      // Field.rightFerryZone.getX())) // (Slope)
      // 				* (turretTranslation.getX() - Field.hubFarRightCorner.getX()) +
      // Field.hubFarRightCorner.getY(); // *(robotX - hubCornerX) + (hubCornerY)
      // leftBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), leftY -
      // Field.hubToleranceY, new Rotation2d()));
      // rightBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), rightY +
      // Field.hubToleranceY, new Rotation2d()));
      // boolean withinHubY = rightY + Field.hubToleranceY < getTurretPose().getY()
      // 					&& getTurretPose().getY() < leftY - Field.hubToleranceY;
      // return behindHubX && withinHubY;
      // === RECTANGLE ===:
      // Translation2d turretTranslation = getTurretPose().getTranslation();
      // boolean behindHubX = Field.hubFarLeftCorner.getX() < turretTranslation.getX()
      // 		&& turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX;
      // boolean withinHubY = Field.hubFarRightCorner.getY() + Field.hubToleranceY <
      // getTurretPose().getY()
      // 		&& getTurretPose().getY() < Field.hubFarLeftCorner.getY() - Field.hubToleranceY;
      // return behindHubX && withinHubY;
    }

    return isBehindHub.get();
  }

  public boolean isOutsideAllianceZone() {
    if (isOutsideAllianceZone.isEmpty()) {
      isOutsideAllianceZone =
          Optional.of(
              getPose()
                  .getMeasureX()
                  .lt(
                      Field.AllianceRightTrench.rightEdge
                          .getMeasureX()
                          .plus(Field.TRENCH_HOOD_TOLERANCE)));
    }

    return isOutsideAllianceZone.get();
  }

  public boolean isBtwnOppHubAndWall() {
    if (!isBtwnOppHubAndWall.isEmpty()) {
      return isBtwnOppHubAndWall.get();
    }

    Translation2d turretTranslation = Turret.getInstance().getTurretPose().getTranslation();

    boolean btwnOppHubAndWallX =
        turretTranslation.getMeasureX().lt(Field.LENGTH)
            && turretTranslation.getMeasureX().gt(Field.OPPONENT_HUB_DS_X);
    boolean btwnOppHubAndWallY =
        turretTranslation.getY() < Field.HUB_FAR_LEFT_CORNER.getY()
            && turretTranslation.getY() > Field.HUB_FAR_RIGHT_CORNER.getY();

    isBtwnOppHubAndWall = Optional.of(btwnOppHubAndWallX && btwnOppHubAndWallY);

    return isBtwnOppHubAndWall.get();
  }

  public void clearMemoized() {
    isBehindHub = Optional.empty();
    isOutsideAllianceZone = Optional.empty();
    isInOpponentZone = Optional.empty();
    isUnderTrench = Optional.empty();
    isBehindTower = Optional.empty();
    isBtwnOppHubAndWall = Optional.empty();
  }

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  private static DriveTrainSimulationConfig mapleSimConfig = null;

  public static DriveTrainSimulationConfig getMapleSimConfig() {
    if (mapleSimConfig != null) return mapleSimConfig;

    return mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Inches.of(2),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    WHEEL_COF));
  }

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    try {
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          this::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Settings.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
