package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX spindexerLeaderMotor;
  private final TalonFX spindexerFollowerMotor;

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

  public SpindexerIOTalonFX() {
    spindexerLeaderMotor = new TalonFX(Ports.Handoff.MOTOR_LEAD);
    spindexerFollowerMotor = new TalonFX(Ports.Handoff.MOTOR_FOLLOW);

    Motors.Spindexer.SPINDEXER_CONFIG.configure(spindexerLeaderMotor);
    Motors.Spindexer.SPINDEXER_CONFIG.configure(spindexerFollowerMotor);

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
      case DUTY_CYCLE -> spindexerLeaderMotor.setControl(
          spindexerController.withOutput(outputs.spindexerLeaderDutyCycle));

      case STOP -> {
        spindexerLeaderMotor.stopMotor();
        spindexerFollowerMotor.stopMotor();

        spindexerFollowerMotor.setControl(followerController);
      }
    }
  }
}
