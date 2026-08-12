package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.CANBus;

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
  }

  final Time DT = Milliseconds.of(20);

  final Mode simMode = Mode.SIM;
  final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  final VisionMode currentVisionMode = VisionMode.LIMELIGHT;

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
