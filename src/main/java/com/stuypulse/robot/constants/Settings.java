package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Settings {

  public interface EnabledSubsystems {
    LoggedNetworkBoolean HANDOFF =
        new LoggedNetworkBoolean("/Tunable/Enabled Subsystems/Handoff", true);
  }

  // A Kit stuff

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // end of A kit stuff

  public interface Intake {
    Angle PIVOT_STOW_ANGLE = Degrees.of(71.0);
    Angle PIVOT_DEPLOY_ANGLE = Degrees.of(-10.0);
    Angle PIVOT_DIGEST_ANGLE = Degrees.of(30);

    Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(5.0);

    Angle PIVOT_MAX_ANGLE = Degrees.of(76.4);
    Angle PIVOT_MIN_ANGLE = Degrees.of(-10.0);

    Angle THRESHOLD_TO_START_ROLLERS = Degrees.of(10.0);

    Angle ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Degrees.of(15.0);
    Voltage HOMING_VOLTAGE = Volts.of(3.0);

    Voltage PUSHDOWN_VOLTAGE = Volts.of(-3.0);
    Current PUSHDOWN_CURRENT_TELEOP =
        Amps.of(
            -75.0); // new SmartNumber("Intake/Pushdown Current", -65.0); //TODO: GET ACTUAL TYTY
    Current PUSHDOWN_CURRENT_AUTON = Amps.of(-80.0);

    double PIVOT_GEAR_RATIO = 32.0 / 20.0 * 64.0 / 18.0 * 60.0 / 8.0;

    Current PIVOT_STALL_CURRENT = Amps.of(0); // TODO: set value
    double PIVOT_STALL_DEBOUNCE = 1.0; // TODO: VERIFY

    double ROLLER_STALL_DEBOUNCE = 0.05; // TODO: VERIFY
    Current ROLLER_STALL_CURRENT = Amps.of(50.0);
  }

  public interface Handoff {

    public final double GEAR_RATIO = 3.0 / 1.0;

    Current HANDOFF_STALL_CURRENT = Amps.of(30); // TODO: set value
    double HANDOFF_STALL_DEBOUNCE_SEC = 0.5; // TODO: VERIFY
  }
}
