package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.RGBWColor;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;

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

        public ControlRequest getPattern() {
            return LEDConstants.solidColorRequest.withColor(color);
        }
    }

    @AutoLogOutput(key = "States/LEDs/Current")
    private LEDState state;
    @AutoLogOutput(key = "States/LEDs/Previous")
    private LEDState previousState;

    private boolean isLeftLimelightDead;
    private boolean isRightLimelightDead;
    private boolean isBackLimelightDead;

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
        this.previousState = LEDState.DISABLED;
    }

    public void changeState(LEDState updatedState) {
        this.state = updatedState;
    }

    public LEDState getState() {
        return this.state;
    }

    private void applyState() {
        if (previousState == state) {
            return;
        }

        outputs.pattern = state.getPattern();
        previousState = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

    public void periodicAfterScheduler() {
        if (!Settings.EnabledSubsystems.LEDs.get()) {
            changeState(LEDState.DISABLED);
        }
        applyState();

        outputs.leftLimelightDead = isLeftLimelightDead;
        outputs.rightLimelightDead = isRightLimelightDead;
        outputs.backLimelightDead = isBackLimelightDead;

        io.applyOutputs(outputs);
    }
}
