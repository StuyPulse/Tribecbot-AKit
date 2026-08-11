package com.stuypulse.robot.subsystems.leds;

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

            default -> instance = new LED(new LEDIO() {} );
        }
    }

    public static LED getInstance() { // getter
        return instance;
    }

    public enum LEDState {
        PASSING_TRENCH,
        IS_BEHIND_HUB,
        TURRET_WRAPPING,
        SHOOT_IN_PLACE,
        SOTM_ON,
        FOTM_ON,
        LEFT_CORNER,
        RIGHT_CORNER,
        KB_DISTANCE,
        STOP_ROLLERS,
        RESET,
        X_WHEELS,
        INTAKE_STOW,
        INTAKE_DEPLOYED,
        DISABLED_ALIGNED,
        DISABLED,
        AUTON_COLOR_ONE,
        AUTON_COLOR_TWO
    }

    @AutoLogOutput(key = "States/LEDs")
    private LEDState state;
    private LEDState previousState;

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
