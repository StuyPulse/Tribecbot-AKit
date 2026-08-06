package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX extends IntakeIOTalonFXBase<TalonFX> {
    public IntakeIOTalonFX(IntakeHardware<TalonFX> hardware) {
        super(hardware);
    }
}
