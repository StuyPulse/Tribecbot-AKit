/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HandoffIOSim implements HandoffIO {

    private final SystemSim<FlywheelSim> handoffSystem;

    private final TalonFXSimulation motorLead;
    private final TalonFXSimulation motorFollow;

    private final DutyCycleOut controller;
    private final Follower follower;

    private final StatusSignal<Current> motorLeadSupplyCurrent;
    private final StatusSignal<Current> motorLeadStatorCurrent;
    private final StatusSignal<Temperature> motorLeadTemperature;
    private final StatusSignal<AngularVelocity> motorLeadVelocity;
    private final StatusSignal<Voltage> motorLeadAppliedVoltage;

    private final StatusSignal<Current> motorFollowSupplyCurrent;
    private final StatusSignal<Current> motorFollowStatorCurrent;
    private final StatusSignal<Temperature> motorFollowTemperature;
    private final StatusSignal<AngularVelocity> motorFollowVelocity;
    private final StatusSignal<Voltage> motorFollowAppliedVoltage;

    public HandoffIOSim() {

        handoffSystem =
                SystemSim.of(
                        new FlywheelSim(
                                LinearSystemId.createFlywheelSystem(
                                        DCMotor.getKrakenX60Foc(2), 0.01, 1),
                                DCMotor.getKrakenX60Foc(2),
                                1));

        motorLead = new TalonFXSimulation(HandoffDeviceIds.MOTOR_LEAD, 1, handoffSystem);
        motorFollow = new TalonFXSimulation(HandoffDeviceIds.MOTOR_FOLLOW, 1, handoffSystem);

        motorLead.configure(HandoffMotorConfigs.HANDOFF_CONFIG);
        motorFollow.configure(HandoffMotorConfigs.HANDOFF_CONFIG);

        motorFollow.linkToReference(motorLead);

        controller = new DutyCycleOut(0).withEnableFOC(true);
        follower = new Follower(motorLead.getDeviceID(), MotorAlignmentValue.Opposed);

        motorFollow.setControl(follower);

        motorLeadSupplyCurrent = motorLead.getSupplyCurrent();
        motorLeadStatorCurrent = motorLead.getStatorCurrent();
        motorLeadTemperature = motorLead.getDeviceTemp();
        motorLeadVelocity = motorLead.getVelocity();
        motorLeadAppliedVoltage = motorLead.getMotorVoltage();

        motorFollowSupplyCurrent = motorLead.getSupplyCurrent();
        motorFollowStatorCurrent = motorLead.getStatorCurrent();
        motorFollowTemperature = motorLead.getDeviceTemp();
        motorFollowVelocity = motorLead.getVelocity();
        motorFollowAppliedVoltage = motorLead.getMotorVoltage();
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        handoffSystem.update(GlobalSettings.DT);
        motorLead.refresh();
        motorFollow.refresh();

        BaseStatusSignal.refreshAll(
                motorLeadSupplyCurrent,
                motorLeadStatorCurrent,
                motorLeadTemperature,
                motorLeadVelocity,
                motorLeadAppliedVoltage,
                motorFollowSupplyCurrent,
                motorFollowStatorCurrent,
                motorFollowTemperature,
                motorFollowVelocity,
                motorFollowAppliedVoltage);

        inputs.motorLeadSupplyCurrent = motorLeadSupplyCurrent.getValue();
        inputs.motorLeadStatorCurrent = motorLeadStatorCurrent.getValue();
        inputs.motorLeadTemperature = motorLeadTemperature.getValue();
        inputs.motorLeadVelocity = motorLeadVelocity.getValue();
        inputs.motorLeadAppliedVoltage = motorLeadAppliedVoltage.getValue();

        inputs.motorFollowSupplyCurrent = motorFollowSupplyCurrent.getValue();
        inputs.motorFollowStatorCurrent = motorFollowStatorCurrent.getValue();
        inputs.motorFollowTemperature = motorFollowTemperature.getValue();
        inputs.motorFollowVelocity = motorFollowVelocity.getValue();
        inputs.motorFollowAppliedVoltage = motorFollowAppliedVoltage.getValue();
    }

    @Override
    public void applyOutputs(HandoffIOOutputs outputs) {
        switch (outputs.handoffMode) {
            case DUTY_CYCLE ->
                    motorLead.setControl(controller.withOutput(outputs.handoffDutyCycle));
            case STOP -> {
                motorLead.stopMotor();
                motorFollow.stopMotor();

                motorFollow.setControl(follower);
            }
        }
    }
}
