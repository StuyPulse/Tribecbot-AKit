package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDController extends SubsystemBase {
  private static final LEDController instance; // LED instance

  static {
    if (Robot.isReal() || Robot.isSimulation()) {
      instance = new LEDController(new LEDIOCANdle() {});
    } else {
      instance = new LEDController(new LEDIO() {});
    }
  }

  public static LEDController getInstance() { // getter
    return instance;
  }

  // IO fields
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs;
  private final LEDIOOutputs outputs;

  // CANdle

  private LEDController(LEDIO io) {
    this.io = io;
    this.inputs = new LEDIOInputsAutoLogged();
    this.outputs = new LEDIOOutputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  public void periodicAfterScheduler() {
    if (Settings.EnabledSubsystems.LEDs.get()) {
      io.applyOutputs(outputs); 
    } else {
      io.clearAllAnimations();
    }
  }
}
