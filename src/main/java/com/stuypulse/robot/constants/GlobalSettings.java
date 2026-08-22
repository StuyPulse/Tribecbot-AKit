/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public interface GlobalSettings {
    final CANBus RIO = new CANBus("rio");
    final CANBus CANIVORE = new CANBus("CANIVORE");

    public interface EnabledSubsystems {
        final LoggedNetworkBoolean INTAKE =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Intake", true);
        final LoggedNetworkBoolean HOOD =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Hood", true);
        final LoggedNetworkBoolean SHOOTER =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Shooter", true);
        final LoggedNetworkBoolean TURRET =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Turret", true);
        final LoggedNetworkBoolean SPINDEXER =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Spindexer", true);
        final LoggedNetworkBoolean HANDOFF =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Handoff", true);
        final LoggedNetworkBoolean LEDs =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/LEDs", true);
        final LoggedNetworkBoolean VISION =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Vision", true);
    }

    final Time DT = Milliseconds.of(20);

    final Mode SIM_MODE = Mode.SIM;
    final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
    final VisionMode VISION_MODE = VisionMode.LIMELIGHT;

    enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    enum VisionMode {
        LIMELIGHT,
        PHOTON
    }
}
