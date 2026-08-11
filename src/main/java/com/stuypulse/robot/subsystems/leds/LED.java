package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDPattern;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private static final LED instance; // LED instance

    static {
        switch (Settings.currentMode) {
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
        PASSING_TRENCH(LEDConstants.PASSING_TRENCH),
        IS_BEHIND_HUB(LEDConstants.IS_BEHIND_HUB),
        TURRET_WRAPPING(LEDConstants.TURRET_WRAPPING),
        SHOOT_IN_PLACE(LEDConstants.SHOOT_IN_PLACE),
        SOTM_ON(LEDConstants.SOTM_ON),
        FOTM_ON(LEDConstants.FOTM_ON),
        LEFT_CORNER(LEDConstants.LEFT_CORNER),
        RIGHT_CORNER(LEDConstants.RIGHT_CORNER),
        KB_DISTANCE(LEDConstants.KB_DISTANCE),
        STOP_ROLLERS(LEDConstants.STOP_ROLLERS),
        RESET(LEDConstants.RESET_HEADING),
        X_WHEELS(LEDConstants.X_WHEELS),
        INTAKE_STOW(LEDConstants.INTAKE_STOW),
        INTAKE_DEPLOYED(LEDConstants.INTAKE_DEPLOYED),
        DISABLED_ALIGNED(LEDConstants.DISABLED_ALIGNED),
        DISABLED(LEDConstants.DISABLED),
        AUTON_COLOR_ONE(LEDConstants.AUTON_ONE),
        AUTON_COLOR_TWO(LEDConstants.AUTON_TWO);

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
        outputs.patterns.add(new LEDPattern(LEDConstants.STRIP_START, LEDConstants.LED_LENGTH, state.getColor()));

        // add limelight dead strips
        if (vision.isCameraDead(Camera.LEFT)) {
            outputs.patterns.add(
                new LEDPattern(LEDConstants.LEFT_DEAD_START, LEDConstants.LEFT_DEAD_END, LEDConstants.LLDEAD)
            );
        }
        if (vision.isCameraDead(Camera.RIGHT)) {
            outputs.patterns.add(
                new LEDPattern(LEDConstants.RIGHT_DEAD_START, LEDConstants.RIGHT_DEAD_END, LEDConstants.LLDEAD)
            );
        }
        if (vision.isCameraDead(Camera.BACK)) {
            outputs.patterns.add(
                new LEDPattern(LEDConstants.BACK_DEAD_START, LEDConstants.BACK_DEAD_END, LEDConstants.LLDEAD)
            );
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);

        io.periodic();
    }

    public void periodicAfterScheduler() {
        if (!Settings.EnabledSubsystems.LEDs.get()) {
            changeState(LEDState.DISABLED);
        }
        applyState();

        io.applyOutputs(outputs);
    }
}
