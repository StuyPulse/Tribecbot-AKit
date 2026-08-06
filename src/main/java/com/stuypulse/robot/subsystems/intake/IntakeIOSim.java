package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

public class IntakeIOSim extends IntakeIOTalonFXBase<TalonFXSimulation> {
    private final IntakeHardware<TalonFXSimulation> hardware;
    
    public IntakeIOSim(IntakeHardware<TalonFXSimulation> hardware) {
        super(hardware);
        this.hardware = hardware;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        hardware.pivotSim.update(Settings.DT);
        hardware.pivotMotor.refresh();

        hardware.rollerSim .update(Settings.DT);
        hardware.rollerLeaderMotor.refresh();
        hardware.rollerFollowerMotor.refresh();

        super.updateInputs(inputs);
    }
}
