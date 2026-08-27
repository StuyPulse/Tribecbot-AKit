/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SpindexerIOSim implements SpindexerIO {
    private final SystemSim<FlywheelSim> spindexerSystem;

    private final TalonFXSimulation spindexerLeaderMotor;
    private final TalonFXSimulation spindexerFollowerMotor;

    private final DutyCycleOut spindexerController;
    private final Follower followerController;

    private final StatusSignal<Angle> spindexerLeaderPosition;
    private final StatusSignal<Current> spindexerLeaderSupplyCurrent;
    private final StatusSignal<Current> spindexerLeaderStatorCurrent;
    private final StatusSignal<Temperature> spindexerLeaderTemperature;
    private final StatusSignal<Voltage> spindexerLeaderAppliedVoltage;
    private final StatusSignal<AngularVelocity> spindexerLeaderVelocity;

    private final StatusSignal<Angle> spindexerFollowerPosition;
    private final StatusSignal<Current> spindexerFollowerSupplyCurrent;
    private final StatusSignal<Temperature> spindexerFollowerTemperature;
    private final StatusSignal<Current> spindexerFollowerStatorCurrent;
    private final StatusSignal<Voltage> spindexerFollowerAppliedVoltage;
    private final StatusSignal<AngularVelocity> spindexerFollowerVelocity;

    public SpindexerIOSim() {
        spindexerSystem =
                SystemSim.of(
                        new FlywheelSim(
                                LinearSystemId.createFlywheelSystem(
                                        DCMotor.getKrakenX44Foc(2), SpindexerSettings.SPINDEXER_MOI.in(KilogramSquareMeters), SpindexerSettings.GEAR_RATIO),
                                DCMotor.getKrakenX44Foc(2)));

        spindexerLeaderMotor = new TalonFXSimulation(SpindexerDeviceIds.LEADER, SpindexerSettings.GEAR_RATIO, spindexerSystem);
        spindexerFollowerMotor =
                new TalonFXSimulation(SpindexerDeviceIds.FOLLOWER, SpindexerSettings.GEAR_RATIO, spindexerSystem);

        spindexerLeaderMotor.configure(SpindexerMotorConfigs.SPINDEXER_LEAD_CONFIG);
        spindexerFollowerMotor.configure(SpindexerMotorConfigs.SPINDEXER_FOLLOW_CONFIG);

        spindexerFollowerMotor.linkToReference(spindexerLeaderMotor);

        spindexerController = new DutyCycleOut(0);
        followerController =
                new Follower(spindexerLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned);

        spindexerFollowerMotor.setControl(followerController);

        spindexerLeaderPosition = spindexerLeaderMotor.getPosition();
        spindexerLeaderSupplyCurrent = spindexerLeaderMotor.getSupplyCurrent();
        spindexerLeaderStatorCurrent = spindexerLeaderMotor.getStatorCurrent();
        spindexerLeaderTemperature = spindexerLeaderMotor.getDeviceTemp();
        spindexerLeaderAppliedVoltage = spindexerLeaderMotor.getMotorVoltage();
        spindexerLeaderVelocity = spindexerLeaderMotor.getVelocity();

        spindexerFollowerPosition = spindexerFollowerMotor.getPosition();
        spindexerFollowerSupplyCurrent = spindexerFollowerMotor.getSupplyCurrent();
        spindexerFollowerStatorCurrent = spindexerFollowerMotor.getStatorCurrent();
        spindexerFollowerTemperature = spindexerFollowerMotor.getDeviceTemp();
        spindexerFollowerAppliedVoltage = spindexerFollowerMotor.getMotorVoltage();
        spindexerFollowerVelocity = spindexerFollowerMotor.getVelocity();
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        spindexerSystem.update(GlobalSettings.DT);
        spindexerLeaderMotor.refresh();
        spindexerFollowerMotor.refresh();

        BaseStatusSignal.refreshAll(
                spindexerLeaderPosition,
                spindexerLeaderSupplyCurrent,
                spindexerLeaderStatorCurrent,
                spindexerLeaderTemperature,
                spindexerLeaderAppliedVoltage,
                spindexerLeaderVelocity,
                spindexerFollowerPosition,
                spindexerFollowerSupplyCurrent,
                spindexerFollowerStatorCurrent,
                spindexerFollowerTemperature,
                spindexerFollowerAppliedVoltage,
                spindexerFollowerVelocity);

        inputs.spindexerLeaderMotorPosition = spindexerLeaderPosition.getValue();
        inputs.spindexerLeaderMotorSupplyCurrent = spindexerLeaderSupplyCurrent.getValue();
        inputs.spindexerLeaderMotorStatorCurrent = spindexerLeaderStatorCurrent.getValue();
        inputs.spindexerLeaderMotorTemperature = spindexerLeaderTemperature.getValue();
        inputs.spindexerLeaderMotorAppliedVoltage = spindexerLeaderAppliedVoltage.getValue();
        inputs.spindexerLeaderMotorVelocity = spindexerLeaderVelocity.getValue();

        inputs.spindexerFollowerMotorPosition = spindexerFollowerPosition.getValue();
        inputs.spindexerFollowerMotorSupplyCurrent = spindexerFollowerSupplyCurrent.getValue();
        inputs.spindexerFollowerMotorStatorCurrent = spindexerFollowerStatorCurrent.getValue();
        inputs.spindexerFollowerMotorTemperature = spindexerFollowerTemperature.getValue();
        inputs.spindexerFollowerMotorAppliedVoltage = spindexerFollowerAppliedVoltage.getValue();
        inputs.spindexerFollowerMotorVelocity = spindexerFollowerVelocity.getValue();
    }

    @Override
    public void applyOutputs(SpindexerIOOutputs outputs) {
        switch (outputs.spindexerMode) {
            case DUTY_CYCLE ->
                    spindexerLeaderMotor.setControl(
                            spindexerController.withOutput(outputs.spindexerLeaderDutyCycle));

            case STOP -> {
                spindexerLeaderMotor.stopMotor();
                spindexerFollowerMotor.stopMotor();

                spindexerFollowerMotor.setControl(followerController);
            }
        }
    }
}
