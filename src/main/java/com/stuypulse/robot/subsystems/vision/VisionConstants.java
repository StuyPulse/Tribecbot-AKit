/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-right";
    public static String camera1Name = "limelight-left";
    public static String camera2Name = "limelight-back";

    public static String[] cameraNames = {camera0Name, camera1Name, camera2Name};

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
            new Transform3d(
                    Units.inchesToMeters(-9.149),
                    Units.inchesToMeters(15.080),
                    Units.inchesToMeters(8.088),
                    new Rotation3d(
                            Units.degreesToRadians(180),
                            Units.degreesToRadians(28.0),
                            Units.degreesToRadians(-80.203885)));
    public static Transform3d robotToCamera1 =
            new Transform3d(
                    Units.inchesToMeters(-2.490),
                    Units.inchesToMeters(-14.8620),
                    Units.inchesToMeters(5.676),
                    new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(14.955812),
                            Units.degreesToRadians(71.5)));
    public static Transform3d robotToCamera2 =
            new Transform3d(
                    Units.inchesToMeters(-10.676),
                    Units.inchesToMeters(-12.969),
                    Units.inchesToMeters(8.753),
                    new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(27.875),
                            Units.degreesToRadians(185.155825)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
