package com.stuypulse.robot.subsystems.handoff;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HandoffIOTalonFX implements HandoffIO {
    private final TalonFX motorLead;
    private final TalonFX motorFollow;

    private final DutyCycleOut controller;
    private final Follower follower;
    private Optional<Double> voltageOverride;

    private final StatusSignal<Current> motorLeadSupplyCurrent;
    private final StatusSignal<Current> motorLeadStatorCurrent;
    private final StatusSignal<AngularVelocity> motorLeadVelocity;
    private final StatusSignal<Voltage> motorLeadAppliedVoltage;

    private final StatusSignal<Current> motorFollowSupplyCurrent;
    private final StatusSignal<Current> motorFollowStatorCurrent;
    private final StatusSignal<AngularVelocity> motorFollowVelocity;
    private final StatusSignal<Voltage> motorFollowAppliedVoltage;

    public HandoffIOTalonFX() {
        motorLead = new TalonFX(Ports.Handoff.MOTOR_LEAD);
        motorFollow = new TalonFX(Ports.Handoff.MOTOR_FOLLOW);

        Motors.Handoff.HANDOFF_CONFIG.configure(motorLead);
        Motors.Handoff.HANDOFF_CONFIG.configure(motorFollow);

        controller = new DutyCycleOut(getTargetDutyCycle());
        follower = new Follower(Ports.Handoff.MOTOR_LEAD, MotorAlignmentValue.Opposed);
        
        motorLeadSupplyCurrent = motorLead.getSupplyCurrent();
        motorLeadStatorCurrent = motorLead.getStatorCurrent();
        motorLeadVelocity = motorLead.getVelocity();
        motorLeadAppliedVoltage = motorLead.getMotorVoltage();

        motorFollowSupplyCurrent = motorLead.getSupplyCurrent();
        motorFollowStatorCurrent = motorLead.getStatorCurrent();
        motorFollowVelocity = motorLead.getVelocity();
        motorFollowAppliedVoltage = motorLead.getMotorVoltage();
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorLeadSupplyCurrent,
            motorLeadStatorCurrent,
            motorLeadVelocity,
            motorLeadAppliedVoltage,
            motorFollowSupplyCurrent,
            motorFollowStatorCurrent,
            motorFollowVelocity,
            motorFollowAppliedVoltage);

        inputs.motorLeadSupplyCurrent = motorLeadSupplyCurrent.getValue();
        inputs.motorLeadStatorCurrent = motorLeadStatorCurrent.getValue();
        inputs.motorLeadVelocity = motorLeadVelocity.getValue();
        inputs.motorLeadAppliedVoltage = motorLeadAppliedVoltage.getValue();
        
        inputs.motorFollowSupplyCurrent = motorFollowSupplyCurrent.getValue();
        inputs.motorFollowStatorCurrent = motorFollowStatorCurrent.getValue();
        inputs.motorFollowVelocity = motorFollowVelocity.getValue();
        inputs.motorFollowAppliedVoltage = motorFollowAppliedVoltage.getValue();
    }

    @Override
    public void applyOutputs(HandoffIOOutputs outputs) {
        motorLead.setControl(controller.withOutput(outputs.handoffDutyCycle));
    }

}
