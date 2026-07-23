package com.stuypulse.robot.constants;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Gains {
  public interface Intake {
    public interface Pivot {
      LoggedNetworkNumber kP = new LoggedNetworkNumber("Intake/Pivot/Gains/kP", 125.0);
      LoggedNetworkNumber kI = new LoggedNetworkNumber("Intake/Pivot/Gains/kI", 0.0);
      LoggedNetworkNumber kD = new LoggedNetworkNumber("Intake/Pivot/Gains/kD", 10.0);

      LoggedNetworkNumber kS = new LoggedNetworkNumber("Intake/Pivot/Gains/kS", 0.0);
      LoggedNetworkNumber kV = new LoggedNetworkNumber("Intake/Pivot/Gains/kV", 0.12);
      LoggedNetworkNumber kA = new LoggedNetworkNumber("Intake/Pivot/Gains/kA", 0.0);

      double kG = 0.5;
    }
  }
}
