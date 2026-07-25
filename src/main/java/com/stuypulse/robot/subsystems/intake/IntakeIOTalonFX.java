package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivotMotor;
  private final TalonFX rollerLeaderMotor;
  private final TalonFX rollerFollowerMotor;

  private final DutyCycleOut rollerLeaderController;
  private final Follower rollerFollowerController;
  private final PositionTorqueCurrentFOC pivotController;
  private final TorqueCurrentFOC pivotPushdownController;
  private final VoltageOut pivotVoltageController;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final StatusSignal<Angle> rollerLeaderPosition;
  private final StatusSignal<Current> rollerLeaderSupplyCurrent;
  private final StatusSignal<Current> rollerLeaderStatorCurrent;
  private final StatusSignal<Temperature> rollerLeaderTemperature;
  private final StatusSignal<Voltage> rollerLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerLeaderVelocity;

  private final StatusSignal<Angle> rollerFollowerPosition;
  private final StatusSignal<Current> rollerFollowerSupplyCurrent;
  private final StatusSignal<Current> rollerFollowerStatorCurrent;
  private final StatusSignal<Temperature> rollerFollowerTemperature;
  private final StatusSignal<Voltage> rollerFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerFollowerVelocity;

  public IntakeIOTalonFX() {
    pivotMotor = new TalonFX(com.stuypulse.robot.constants.Ports.Intake.PIVOT);
    rollerLeaderMotor = new TalonFX(Ports.Intake.ROLLER_LEADER);
    rollerFollowerMotor = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);

    Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);
    Motors.Intake.ROLLER_CONFIG.configure(rollerLeaderMotor);
    Motors.Intake.ROLLER_CONFIG.configure(rollerFollowerMotor);

    rollerLeaderController = new DutyCycleOut(0);
    rollerFollowerController =
        new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    pivotController = new PositionTorqueCurrentFOC(0);
    pivotPushdownController = new TorqueCurrentFOC(0);
    pivotVoltageController = new VoltageOut(0);

    rollerFollowerMotor.setControl(rollerFollowerController);
    pivotMotor.setPosition(Settings.Intake.PIVOT_MAX_ANGLE);

    pivotPosition = pivotMotor.getPosition();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    pivotStatorCurrent = pivotMotor.getStatorCurrent();
    pivotTemperature = pivotMotor.getDeviceTemp();
    pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    pivotVelocity = pivotMotor.getVelocity();

    rollerLeaderPosition = rollerLeaderMotor.getPosition();
    rollerLeaderSupplyCurrent = rollerLeaderMotor.getSupplyCurrent();
    rollerLeaderStatorCurrent = rollerLeaderMotor.getStatorCurrent();
    rollerLeaderTemperature = rollerLeaderMotor.getDeviceTemp();
    rollerLeaderAppliedVoltage = rollerLeaderMotor.getMotorVoltage();
    rollerLeaderVelocity = rollerLeaderMotor.getVelocity();

    rollerFollowerPosition = rollerFollowerMotor.getPosition();
    rollerFollowerSupplyCurrent = rollerFollowerMotor.getSupplyCurrent();
    rollerFollowerStatorCurrent = rollerFollowerMotor.getStatorCurrent();
    rollerFollowerTemperature = rollerFollowerMotor.getDeviceTemp();
    rollerFollowerAppliedVoltage = rollerFollowerMotor.getMotorVoltage();
    rollerFollowerVelocity = rollerFollowerMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        pivotTemperature,
        pivotAppliedVoltage,
        pivotVelocity,
        rollerLeaderPosition,
        rollerLeaderSupplyCurrent,
        rollerLeaderStatorCurrent,
        rollerLeaderTemperature,
        rollerLeaderAppliedVoltage,
        rollerLeaderVelocity,
        rollerFollowerPosition,
        rollerFollowerSupplyCurrent,
        rollerFollowerStatorCurrent,
        rollerFollowerTemperature,
        rollerFollowerAppliedVoltage,
        rollerFollowerVelocity);

    inputs.pivotMotorPosition = pivotPosition.getValue();
    inputs.pivotMotorSupplyCurrent = pivotSupplyCurrent.getValue();
    inputs.pivotMotorStatorCurrent = pivotStatorCurrent.getValue();
    inputs.pivotMotorTemperature = pivotTemperature.getValue();
    inputs.pivotMotorAppliedVoltage = pivotAppliedVoltage.getValue();
    inputs.pivotMotorVelocity = pivotVelocity.getValue();

    inputs.rollerLeaderMotorPosition = rollerLeaderPosition.getValue();
    inputs.rollerLeaderMotorSupplyCurrent = rollerLeaderSupplyCurrent.getValue();
    inputs.rollerLeaderMotorStatorCurrent = rollerLeaderStatorCurrent.getValue();
    inputs.rollerLeaderMotorTemperature = rollerLeaderTemperature.getValue();
    inputs.rollerLeaderMotorAppliedVoltage = rollerLeaderAppliedVoltage.getValue();
    inputs.rollerLeaderMotorVelocity = rollerLeaderVelocity.getValue();

    inputs.rollerFollowerMotorPosition = rollerFollowerPosition.getValue();
    inputs.rollerFollowerMotorSupplyCurrent = rollerFollowerSupplyCurrent.getValue();
    inputs.rollerFollowerMotorStatorCurrent = rollerFollowerStatorCurrent.getValue();
    inputs.rollerFollowerMotorTemperature = rollerFollowerTemperature.getValue();
    inputs.rollerFollowerMotorAppliedVoltage = rollerFollowerAppliedVoltage.getValue();
    inputs.rollerFollowerMotorVelocity = rollerFollowerVelocity.getValue();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (!Settings.EnabledSubsystems.INTAKE.get()) {
      pivotMotor.stopMotor();
      rollerLeaderMotor.stopMotor();
      rollerFollowerMotor.stopMotor();

      rollerFollowerMotor.setControl(rollerFollowerController);

      return;
    }

    switch (outputs.pivotOutputMode) {
      case POSITION:
        pivotMotor.setControl(pivotController.withPosition(outputs.pivotPosition));
        break;
      case TORQUE_CURRENT:
        pivotMotor.setControl(pivotPushdownController.withOutput(outputs.pivotTorqueCurrent));
        break;
      case VOLTAGE:
        pivotMotor.setControl(pivotVoltageController.withOutput(outputs.pivotVoltage));
        break;
    }

    rollerLeaderMotor.setControl(rollerLeaderController.withOutput(outputs.rollerDutyCycle));
  }

  @Override
  public void seedPivotPosition(Angle position) {
    pivotMotor.setPosition(position);
  }
}
