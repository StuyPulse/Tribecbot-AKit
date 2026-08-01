package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;

public class Ports {
  public CANBus RIO = new CANBus("rio");
  public static CANBus CANIVORE = new CANBus("CANIVORE");

  public interface Gamepad {
    int DRIVER = 0;
  }

  public interface LED {
    int LED_PORT = 1;
    int CANDLE_PORT = 61;
  }

  public interface Handoff {
    int MOTOR_LEAD = 43;
    int MOTOR_FOLLOW = 48;
  }

  public interface Superstructure {
    public interface Hood {
      int MOTOR = 45;
      int THROUGHBORE_ENCODER = 44;
    }

    public interface Shooter {
      int MOTOR_LEAD = 47;
      int MOTOR_FOLLOW = 46;
    }

    public interface Turret {
      int MOTOR = 40;
      int ENCODER17T = 42;
      int ENCODER18T = 41;
    }
  }

  public interface Intake {
    int PIVOT = 20;
    int ROLLER_LEADER = 21;
    int ROLLER_FOLLOWER = 22;
  }
}
