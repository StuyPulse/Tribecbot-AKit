package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.MotorConfig;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.MotorIds;
import edu.wpi.first.units.measure.*;

public class HandoffIOTalonFX implements HandoffIO {
  private final TalonFX motorLead;
  private final TalonFX motorFollow;

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

  public HandoffIOTalonFX() {
    motorLead = new TalonFX(MotorIds.MOTOR_LEAD);
    motorFollow = new TalonFX(MotorIds.MOTOR_FOLLOW);

    MotorConfig.HANDOFF_CONFIG.configure(motorLead);
    MotorConfig.HANDOFF_CONFIG.configure(motorFollow);

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
      case DUTY_CYCLE -> motorLead.setControl(controller.withOutput(outputs.handoffDutyCycle));
      case STOP -> {
        motorLead.stopMotor();
        motorFollow.stopMotor();

        motorFollow.setControl(follower);
      }
    }
  }
}
