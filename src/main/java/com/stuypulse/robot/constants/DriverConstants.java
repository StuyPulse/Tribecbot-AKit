/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public interface DriverConstants {
    public interface DriverSettings {
        int INDEX = 0;
        Time BUZZ_TIME = Seconds.of(1.0);
        double BUZZ_INTENSITY = 1.0;
    }

    public interface DriverDriveSettings {
        double DEADBAND = 0.05;
        double RC = 0.05;
        int POWER = 2;
    }

    public interface DriverTurnSettings {
        double DEADBAND = 0.05;
        double RC = 0.05;
        int POWER = 2;
    }
}
