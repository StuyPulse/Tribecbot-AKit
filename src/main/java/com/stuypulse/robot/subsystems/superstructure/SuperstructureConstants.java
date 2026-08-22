/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface SuperstructureConstants {
    public interface InterpolationConstants {
        final double[][] DISTANCE_ANGLE_INTERPOLATION_VALUES = {
            {0.96, Math.toRadians(15.0)},
            {1.22, Math.toRadians(20.0)},
            {2.15, Math.toRadians(27.0)},
            {3.38, Math.toRadians(34.0)},
            {4.43, Math.toRadians(39.0)},
            {5.66, Math.toRadians(39.0)},
            {6.44, Math.toRadians(44.0)}
        };

        final double[][] DISTANCE_RPM_INTERPOLATION_VALUES = {
            {0.96, 2800.0},
            {1.22, 2600.0},
            {2.15, 2805.0},
            {3.38, 3075.0},
            {4.43, 3350.0},
            {5.66, 3650.0},
            {6.44, 3800.0},
            {8.23, 4500.0} // THIS POINT IS AN EXTRAPOLATION
        };

        final double[][] DISTANCE_TOF_INTERPOLATION_VALUES = {
            {0.96, 1.055},
            {1.22, 0.965},
            {2.15, 1.01},
            {3.38, 1.02},
            {4.43, 1.165},
            {5.50, 1.21},
            {6.44, 1.255},
            {6.6, 1.41},
            {8.23, 1.71} // THIS POINT IS AN EXTRAPOLATION
        };

        final double[][] FERRY_DISTANCE_RPM_INTERPOLATION = {
            {1.0, 2000.0},
            {5.16, 3300.0},
            {6.94, 3600.0},
            {7.87, 3800.0},
            {9.77, 4300.0}, // TODO: ADD DATA BACK IN COMP
            {10.694, 4700.0}, // STARTING FROM HERE THE DATA IS EXTRAPOLATED
            {11.516, 4900.0}
        };

        final double[][] FERRY_TOF_INTERPOLATION = {
            {5.16, 1.16},
            {6.94, 1.37},
            {7.87, 1.57},
            {9.77, 1.64},
            {10.694, 1.765}, // extrapolated
            {11.516, 1.838}, // extrapolated
            {12.416, 1.914}, // extrapolated
            {13.316, 1.988}, // extrapolated
            {14.216, 2.060}, // extrapolated
            {15.148, 2.131}, // extrapolated
            {16.54, 2.234} // extrapolated (field length)
        };
    }

    public interface SOTM {
        final int MAX_ITERATIONS = 10;
        final double TIME_TOLERANCE = 1e-3;

        final LoggedNetworkNumber UPDATE_DELAY =
                new LoggedNetworkNumber("/Tuning/Superstructure/SOTM/update delay", 0.05);
    }
}
