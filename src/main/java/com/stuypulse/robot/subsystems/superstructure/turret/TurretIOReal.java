/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;

import edu.wpi.first.units.measure.*;

public class TurretIOReal implements TurretIO {
    private final TalonFX turretMotor;

    private final CANcoder encoder17t;
    private final CANcoder encoder18t;

    private final PositionVoltage positionController;

    private final StatusSignal<Angle> turretMotorPosition;
    private final StatusSignal<Current> turretMotorSupplyCurrent;
    private final StatusSignal<Current> turretMotorStatorCurrent;
    private final StatusSignal<Temperature> turretMotorTemperature;
    private final StatusSignal<Voltage> turretMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> turretMotorVelocity;

    private final StatusSignal<Angle> encoder17tPosition;
    private final StatusSignal<Angle> encoder18tPosition;

    private Angle encoder17tMagnetOffset;
    private Angle encoder18tMagnetOffset;

    public TurretIOReal() {
        turretMotor = new TalonFX(TurretDeviceIds.MOTOR, GlobalSettings.RIO);
        TurretMotorConfigs.TURRET_CONFIG.configure(turretMotor);

        turretMotor.getClosedLoopError().setUpdateFrequency(Hertz.of(50));

        positionController = new PositionVoltage(0).withEnableFOC(true);

        encoder17t = new CANcoder(TurretDeviceIds.ENCODER_17T, GlobalSettings.RIO);
        encoder18t = new CANcoder(TurretDeviceIds.ENCODER_18T, GlobalSettings.RIO);

        TurretEncoderConfigs.encoder17tConfig.configure(encoder17t);
        TurretEncoderConfigs.encoder18tConfig.configure(encoder18t);

        turretMotorPosition = turretMotor.getPosition();
        turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();
        turretMotorStatorCurrent = turretMotor.getStatorCurrent();
        turretMotorTemperature = turretMotor.getDeviceTemp();
        turretMotorAppliedVoltage = turretMotor.getMotorVoltage();
        turretMotorVelocity = turretMotor.getVelocity();

        encoder17tPosition = encoder17t.getAbsolutePosition();
        encoder18tPosition = encoder18t.getAbsolutePosition();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                turretMotorPosition,
                turretMotorSupplyCurrent,
                turretMotorStatorCurrent,
                turretMotorTemperature,
                turretMotorAppliedVoltage,
                turretMotorVelocity,
                encoder17tPosition,
                encoder18tPosition);
        inputs.turretMotorPosition = turretMotorPosition.getValue();
        inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValue();
        inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValue();
        inputs.turretMotorTemperature = turretMotorTemperature.getValue();
        inputs.turretMotorAppliedVoltage = turretMotorAppliedVoltage.getValue();
        inputs.turretMotorVelocity = turretMotorVelocity.getValue();

        inputs.encoder17tPosition = encoder17tPosition.getValue();
        inputs.encoder18tPosition = encoder18tPosition.getValue();

        inputs.encoder17tMagnetOffset = encoder17tMagnetOffset;
        inputs.encoder18tMagnetOffset = encoder18tMagnetOffset;
    }

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
        switch (outputs.turretMode) {
            case POSITION ->
                    turretMotor.setControl(
                            positionController
                                    .withPosition(outputs.turretPosition)
                                    .withSlot(outputs.gainSlot)
                                    .withFeedForward(outputs.feedForward));

            case STOP -> turretMotor.stopMotor();
        }
    }

    @Override
    public void seedTurretPosition(Angle position) {
        turretMotor.setPosition(position);
    }

    @Override
    public void zeroEncoders() {
        MagnetSensorConfigs config17t = new MagnetSensorConfigs();
        MagnetSensorConfigs config18t = new MagnetSensorConfigs();

        encoder17t.getConfigurator().refresh(config17t);
        encoder18t.getConfigurator().refresh(config18t);

        Angle newOffset17t = config17t.getMagnetOffsetMeasure().minus(encoder17t.getAbsolutePosition().getValue());
        Angle newOffset18t = config18t.getMagnetOffsetMeasure().minus(encoder18t.getAbsolutePosition().getValue());

        config17t.withMagnetOffset(newOffset17t);
        config18t.withMagnetOffset(newOffset18t);

        encoder17t.getConfigurator().apply(config17t);
        encoder18t.getConfigurator().apply(config18t);

        encoder17tMagnetOffset = newOffset17t;
        encoder18tMagnetOffset = newOffset18t;
    }
}