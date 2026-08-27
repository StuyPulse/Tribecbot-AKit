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
    CANBus RIO = new CANBus("rio");
    CANBus CANIVORE = new CANBus("CANIVORE");

    public interface EnabledSubsystems {
        LoggedNetworkBoolean INTAKE =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Intake", true);
        LoggedNetworkBoolean HOOD =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Hood", true);
        LoggedNetworkBoolean SHOOTER =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Shooter", true);
        LoggedNetworkBoolean TURRET =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Turret", true);
        LoggedNetworkBoolean SPINDEXER =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Spindexer", true);
        LoggedNetworkBoolean HANDOFF =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Handoff", true);
        LoggedNetworkBoolean LEDs =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/LEDs", true);
        LoggedNetworkBoolean VISION =
                new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Vision", true);
    }

    Time DT = Milliseconds.of(20);

    Mode SIM_MODE = Mode.SIM;
    Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
    VisionMode VISION_MODE = VisionMode.LIMELIGHT;

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
