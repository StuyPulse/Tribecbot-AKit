package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.leds.LEDConstants.Settings.StateColors;
import com.stuypulse.robot.subsystems.leds.LEDConstants.Settings;

import com.ctre.phoenix6.signals.RGBWColor;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDPattern;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Camera;
import com.stuypulse.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LED extends FullSubsystem {
  private static final LED instance; // LED instance

  static {
    switch (GlobalSettings.currentMode) {
      case REAL -> instance = new LED(new LEDIOCANdle());

      case SIM -> instance = new LED(new LEDIOSim());

      default -> instance = new LED(new LEDIO() {
      });
    }
  }

  public static LED getInstance() { // getter
    return instance;
  }

  public enum LEDState {
    PASSING_TRENCH(StateColors.PASSING_TRENCH),
    IS_BEHIND_HUB(StateColors.IS_BEHIND_HUB),
    TURRET_WRAPPING(StateColors.TURRET_WRAPPING),
    SHOOT_IN_PLACE(StateColors.SHOOT_IN_PLACE),
    SOTM_ON(StateColors.SOTM_ON),
    FOTM_ON(StateColors.FOTM_ON),
    LEFT_CORNER(StateColors.LEFT_CORNER),
    RIGHT_CORNER(StateColors.RIGHT_CORNER),
    KB_DISTANCE(StateColors.KB_DISTANCE),
    STOP_ROLLERS(StateColors.STOP_ROLLERS),
    RESET(StateColors.RESET_HEADING),
    X_WHEELS(StateColors.X_WHEELS),
    INTAKE_STOW(StateColors.INTAKE_STOW),
    INTAKE_DEPLOYED(StateColors.INTAKE_DEPLOYED),
    DISABLED_ALIGNED(StateColors.DISABLED_ALIGNED),
    DISABLED(StateColors.DISABLED),
    AUTON_COLOR_ONE(StateColors.AUTON_ONE),
    AUTON_COLOR_TWO(StateColors.AUTON_TWO);

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
        new LEDPattern(Settings.STRIP_START, Settings.LED_LENGTH, state.getColor()));

    // add limelight dead strips
    if (vision.isCameraDead(Camera.LEFT)) {
      outputs.patterns.add(
          new LEDPattern(
              Settings.LEFT_DEAD_START, Settings.LEFT_DEAD_END, StateColors.LLDEAD));
    }
    if (vision.isCameraDead(Camera.RIGHT)) {
      outputs.patterns.add(
          new LEDPattern(
              Settings.RIGHT_DEAD_START, Settings.RIGHT_DEAD_END, StateColors.LLDEAD));
    }
    if (vision.isCameraDead(Camera.BACK)) {
      outputs.patterns.add(
          new LEDPattern(
              Settings.BACK_DEAD_START, Settings.BACK_DEAD_END, StateColors.LLDEAD));
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
  }
}
