/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import static edu.wpi.first.units.Units.*;

public interface VisionConstants {
    public interface VisionSettings {
        int RESET_IMU_INDEX = 1;

        // Basic filtering thresholds
        double MAX_AMBIGUITY = 0.3;
        double MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
        double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
                Double.POSITIVE_INFINITY; // No rotation data available

        double BUZZ_DEBOUNCE = 0.25;
    }

    record CameraData(String name, Transform3d robotToCamera, double stdDevFactor) {}

    public enum Cameras {
        RIGHT(
                "limelight-right",
                new Transform3d(
                        Inches.of(-9.149),
                        Inches.of(15.080),
                        Inches.of(8.088),
                        new Rotation3d(
                                Degrees.of(180),
                                Degrees.of(28.0),
                                Degrees.of(-80.203885))),
                1.0),
        LEFT(
                "limelight-left",
                new Transform3d(
                        Inches.of(-2.490),
                        Inches.of(-14.8620),
                        Inches.of(5.676),
                        new Rotation3d(
                                Degrees.of(0),
                                Degrees.of(14.955812),
                                Degrees.of(71.5))),
                1.0),
        BACK(
                "limelight-back",
                new Transform3d(
                        Inches.of(-10.676),
                        Inches.of(-12.969),
                        Inches.of(8.753),
                        new Rotation3d(
                                Degrees.of(0),
                                Degrees.of(27.875),
                                Degrees.of(185.155825))),
                1.0);

        private final CameraData data;

        private Cameras(String name, Transform3d robotToCamera, double stdDevFactor) {
            this.data = new CameraData(name, robotToCamera, stdDevFactor);
        }

        public String getName() {
            return data.name();
        }

        public Transform3d getRobotToCamera() {
            return data.robotToCamera();
        }

        public double getStdDevFactor() {
            return data.stdDevFactor();
        }

        public CameraData getData() {
            return data;
        }
    }
}
