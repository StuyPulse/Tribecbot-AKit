package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.leds.LEDConstants.*;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDPattern;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Camera;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LED extends FullSubsystem {
  private static final LED instance; // LED instance

  static {
    switch (GlobalSettings.currentMode) {
      case REAL -> instance = new LED(new LEDIOReal());

      case SIM -> instance = new LED(new LEDIOSim());

      default -> instance = new LED(new LEDIO() {});
    }
  }

  public static LED getInstance() { // getter
    return instance;
  }

  public enum LEDState {
    PASSING_TRENCH(LEDSettings.StateColors.PASSING_TRENCH),
    IS_BEHIND_HUB(LEDSettings.StateColors.IS_BEHIND_HUB),
    TURRET_WRAPPING(LEDSettings.StateColors.TURRET_WRAPPING),
    SHOOT_IN_PLACE(LEDSettings.StateColors.SHOOT_IN_PLACE),
    SOTM_ON(LEDSettings.StateColors.SOTM_ON),
    FOTM_ON(LEDSettings.StateColors.FOTM_ON),
    LEFT_CORNER(LEDSettings.StateColors.LEFT_CORNER),
    RIGHT_CORNER(LEDSettings.StateColors.RIGHT_CORNER),
    KB_DISTANCE(LEDSettings.StateColors.KB_DISTANCE),
    STOP_ROLLERS(LEDSettings.StateColors.STOP_ROLLERS),
    ROLLERS_REVERSE(LEDSettings.StateColors.ROLLERS_REVERSE),
    RESET_HEADING(LEDSettings.StateColors.RESET_HEADING),
    X_WHEELS(LEDSettings.StateColors.X_WHEELS),
    INTAKE_STOW(LEDSettings.StateColors.INTAKE_STOW),
    INTAKE_DEPLOYED(LEDSettings.StateColors.INTAKE_DEPLOYED),
    DISABLED_ALIGNED(LEDSettings.StateColors.DISABLED_ALIGNED),
    DISABLED(LEDSettings.StateColors.DISABLED),
    AUTON_COLOR_ONE(LEDSettings.StateColors.AUTON_ONE),
    AUTON_COLOR_TWO(LEDSettings.StateColors.AUTON_TWO);

    private final RGBWColor color;

    LEDState(RGBWColor color) {
      this.color = color;
    }

    public RGBWColor getColor() {
      return color;
    }
  }

  @AutoLogOutput(key = "States/LEDs/Current")
  private LEDState state;

  private Vision vision;

  // IO fields
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs;
  private final LEDIOOutputs outputs;

  // CANdle

  private LED(LEDIO io) {
    this.io = io;
    this.inputs = new LEDIOInputsAutoLogged();
    this.outputs = new LEDIOOutputs();

    this.state = LEDState.DISABLED;
    this.vision = Vision.getInstance();
  }

  public void changeState(LEDState updatedState) {
    this.state = updatedState;
  }

  public LEDState getState() {
    return this.state;
  }

  private void applyState() {
    outputs.patterns.clear();
    outputs.patterns.add(
        new LEDPattern(LEDSettings.STRIP_START, LEDSettings.LED_LENGTH, state.getColor()));

    // add limelight dead strips
    if (vision.isCameraDead(Camera.LEFT)) {
      outputs.patterns.add(
          new LEDPattern(
              LEDSettings.LEFT_DEAD_START,
              LEDSettings.LEFT_DEAD_END,
              LEDSettings.StateColors.LLDEAD));
    }
    if (vision.isCameraDead(Camera.RIGHT)) {
      outputs.patterns.add(
          new LEDPattern(
              LEDSettings.RIGHT_DEAD_START,
              LEDSettings.RIGHT_DEAD_END,
              LEDSettings.StateColors.LLDEAD));
    }
    if (vision.isCameraDead(Camera.BACK)) {
      outputs.patterns.add(
          new LEDPattern(
              LEDSettings.BACK_DEAD_START,
              LEDSettings.BACK_DEAD_END,
              LEDSettings.StateColors.LLDEAD));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    io.periodic();
  }

  @Override
  public void periodicAfterScheduler() {
    if (!GlobalSettings.EnabledSubsystems.LEDs.get()) {
      changeState(LEDState.DISABLED);
    }
    applyState();

    io.applyOutputs(outputs);

    for (LEDPattern pattern : outputs.patterns) {
        Logger.recordOutput("LEDs/Pattern/" + pattern.start() + "-" + pattern.end(), pattern.color().toString());
    }
  }

  public Command setState(LEDState state) {
    return run(() -> changeState(state));
  }
}
