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

package com.stuypulse.robot.subsystems.vision;

import static com.stuypulse.robot.subsystems.vision.VisionConstants.*;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Camera;

import java.util.Arrays;
import java.util.stream.IntStream;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.VisionMode;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.subsystems.vision.VisionIO.MegaTagMode;
import com.stuypulse.robot.subsystems.vision.VisionIO.PoseObservationType;
import com.stuypulse.robot.subsystems.vision.VisionIO.VisionIOOutputs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static final Vision instance;

    static {
        Drive drive = Drive.getInstance();

        switch (Settings.currentMode) {
            case REAL -> {
                if (Settings.currentVisionMode == VisionMode.LIMELIGHT) {
                    instance = new Vision(
                            drive,
                            Arrays.stream(cameras)
                                .map((camera) -> new VisionIOLimelight(camera.name(), drive::getRotation))
                                .toArray(VisionIO[]::new));
                } else {
                    instance = new Vision(
                            drive,
                            Arrays.stream(cameras)
                                .map((camera) -> new VisionIOPhotonVision(camera.name(), camera.robotToCamera()))
                                .toArray(VisionIO[]::new));
                }
            }

            case SIM -> {
                SwerveDriveSimulation driveSimulation = Drive.getDriveSimulation();

                instance = new Vision(
                        drive,
                        Arrays.stream(cameras)
                            .map((camera) -> new VisionIOPhotonVisionSim(camera.name(), camera.robotToCamera(), driveSimulation::getSimulatedDriveTrainPose))
                            .toArray(VisionIO[]::new));
            }

            // For replay mode
            default -> {
                instance = new Vision(
                            drive,
                            IntStream.range(0, cameras.length)
                                .mapToObj((_i) -> (VisionIO) new VisionIO() {} )
                                .toArray(VisionIO[]::new));
            }
        }
    }

    public static Vision getInstance() {
        return instance;
    }

    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final VisionIOOutputs[] outputs;
    private final Alert[] disconnectedAlerts;
    private int maxTagCount;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.outputs = new VisionIOOutputs[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            outputs[i] = new VisionIOOutputs();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + cameras[i].name() + " is disconnected.",
                    AlertType.kWarning);
        }

        maxTagCount = 0;
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public int getMaxTagCount() {
        return maxTagCount;
    }

    public boolean isCameraDead(Camera camera) {
        return !inputs[camera.ordinal()].connected;
    }

    @Override
    public void periodic() {
        maxTagCount = 0;

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameras[i].name(), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                maxTagCount = Math.max(maxTagCount, observation.tagCount());
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                linearStdDev *= cameras[cameraIndex].stdDevFactor();
                angularStdDev *= cameras[cameraIndex].stdDevFactor();

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + cameras[cameraIndex].name() + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameras[cameraIndex].name() + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameras[cameraIndex].name() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameras[cameraIndex].name() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    public void periodicAfterScheduler() {
        for (int i = 0; i < io.length; i++) {
            Logger.recordOutput("Vision/" + cameras[i].name() + "/MegaTagMode", outputs[i].megaTagMode);
            Logger.recordOutput("Vision/" + cameras[i].name() + "/Pipeline", outputs[i].pipeline);

            io[i].applyOutputs(outputs[i]);
        }
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }

    public Command setMegaTagMode(MegaTagMode mode) {
        return runOnce(
                () -> {
                    for (VisionIOOutputs output : outputs) {
                        output.megaTagMode = mode;
                    }
                })
                .ignoringDisable(true);
    }

    public Command setPipeline(int pipeline) {
        return runOnce(
                () -> {
                    for (VisionIOOutputs output : outputs) {
                        output.pipeline = pipeline;
                    }
                })
                .ignoringDisable(true);
    }

    public Command setAprilTagWhitelist(double[] whitelist) {
        return runOnce(
                () -> {
                    for (VisionIOOutputs output : outputs) {
                        output.aprilTagIDWhitelist = whitelist;
                    }
                })
                .ignoringDisable(true);
    }
}
