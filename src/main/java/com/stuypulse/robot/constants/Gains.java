package com.stuypulse.robot.constants;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Gains {
  public interface Intake {
    public interface Pivot {
      LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kP", 125.0);
      LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kI", 0.0);
      LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kD", 10.0);

      LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kS", 0.0);
      LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kV", 0.12);
      LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Gains/kA", 0.0);

      double kG = 0.5;
    }
  }

  public interface Superstructure {
    public interface Shooter {
      // VTC PID
      LoggedNetworkNumber kP = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kP", 10.5);
      LoggedNetworkNumber kI = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kI", 0.0);
      LoggedNetworkNumber kD = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kD", 0.0);

      LoggedNetworkNumber kS = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kS", 2.47);
      LoggedNetworkNumber kV = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kV", 0.01775);
      LoggedNetworkNumber kA = new LoggedNetworkNumber("Superstructure/Shooter/Gains/kA", 0.0);
    }

    public interface Hood {
      double kP = 250.0;
      double kI = 0.0;
      double kD = 2.0;

      double kS = 0.25;
      double kV = 0.0;
      double kA = 0.0;
    }

    public interface Turret {
      public interface slot0 {
        double kP = 200.0;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.4775;
        double kV = 0.0;
        double kA = 0.0;
      }

      public interface slot1 {
        LoggedNetworkNumber kP =
            new LoggedNetworkNumber("Superstructure/Turret/Gains/kP", 150.0); // 80
        LoggedNetworkNumber kI = new LoggedNetworkNumber("Superstructure/Turret/Gains/kI", 0.0);
        LoggedNetworkNumber kD =
            new LoggedNetworkNumber("Superstructure/Turret/Gains/kD", 3.0); // 10

        LoggedNetworkNumber kS = new LoggedNetworkNumber("Superstructure/Turret/Gains/kS", 0.4775);
        LoggedNetworkNumber kV = new LoggedNetworkNumber("Superstructure/Turret/Gains/kV", 0.0);
        LoggedNetworkNumber kA = new LoggedNetworkNumber("Superstructure/Turret/Gains/kA", 0.0);
      }

      LoggedNetworkNumber kOmega =
          new LoggedNetworkNumber("Superstructure/Turret/Gains/kOmega", 3.43);
      LoggedNetworkNumber kTranslation =
          new LoggedNetworkNumber("Superstructure/Turret/Gains/kTranslation", 0.0);
    }
  }
}
