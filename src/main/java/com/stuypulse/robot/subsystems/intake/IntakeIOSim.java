package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SystemSim<SingleJointedArmSim> pivotSim;
  private final SystemSim<DCMotorSim> rollerSim;

  private final TalonFXSimulation pivotMotor;
  private final TalonFXSimulation rollerLeaderMotor;
  private final TalonFXSimulation rollerFollowerMotor;

  private final DutyCycleOut rollerLeaderController;
  private final Follower rollerFollowerController;
  private final PositionVoltage pivotPositionController;
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

  public IntakeIOSim() {
    this.pivotSim =
        SystemSim.of(
            new SingleJointedArmSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1),
                    Settings.Intake.PIVOT_MOI,
                    Settings.Intake.PIVOT_GEAR_RATIO),
                DCMotor.getKrakenX60Foc(1),
                Settings.Intake.PIVOT_GEAR_RATIO,
                Settings.Intake.ARM_LENGTH_METERS,
                Settings.Intake.PIVOT_MIN_ANGLE.in(Radians),
                Settings.Intake.PIVOT_MAX_ANGLE.in(Radians),
                true,
                Settings.Intake.PIVOT_MAX_ANGLE.in(Radians)));

    this.rollerSim =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2),
                    0.01, // arbitrary
                    1.0),
                DCMotor.getKrakenX60Foc(2)));

    this.pivotMotor =
        new TalonFXSimulation(Ports.Intake.PIVOT, Settings.Intake.PIVOT_GEAR_RATIO, pivotSim);
    this.rollerLeaderMotor = new TalonFXSimulation(Ports.Intake.ROLLER_LEADER, 1.0, rollerSim);
    this.rollerFollowerMotor = new TalonFXSimulation(Ports.Intake.ROLLER_FOLLOWER, 1.0, rollerSim);

    Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);
    Motors.Intake.ROLLER_CONFIG.configure(rollerLeaderMotor);
    Motors.Intake.ROLLER_CONFIG.configure(rollerFollowerMotor);

    this.rollerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    this.rollerFollowerController =
        new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    this.pivotPositionController = new PositionVoltage(0).withEnableFOC(true);
    this.pivotPushdownController = new TorqueCurrentFOC(0);
    this.pivotVoltageController = new VoltageOut(0).withEnableFOC(true);

    rollerFollowerMotor.setControl(rollerFollowerController);
    pivotMotor.setControl(pivotPositionController);

    this.pivotPosition = pivotMotor.getPosition();
    this.pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    this.pivotStatorCurrent = pivotMotor.getStatorCurrent();
    this.pivotTemperature = pivotMotor.getDeviceTemp();
    this.pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    this.pivotVelocity = pivotMotor.getVelocity();

    this.rollerLeaderPosition = rollerLeaderMotor.getPosition();
    this.rollerLeaderSupplyCurrent = rollerLeaderMotor.getSupplyCurrent();
    this.rollerLeaderStatorCurrent = rollerLeaderMotor.getStatorCurrent();
    this.rollerLeaderTemperature = rollerLeaderMotor.getDeviceTemp();
    this.rollerLeaderAppliedVoltage = rollerLeaderMotor.getMotorVoltage();
    this.rollerLeaderVelocity = rollerLeaderMotor.getVelocity();

    this.rollerFollowerPosition = rollerFollowerMotor.getPosition();
    this.rollerFollowerSupplyCurrent = rollerFollowerMotor.getSupplyCurrent();
    this.rollerFollowerStatorCurrent = rollerFollowerMotor.getStatorCurrent();
    this.rollerFollowerTemperature = rollerFollowerMotor.getDeviceTemp();
    this.rollerFollowerAppliedVoltage = rollerFollowerMotor.getMotorVoltage();
    this.rollerFollowerVelocity = rollerFollowerMotor.getVelocity();
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

    this.pivotSim.update(Settings.DT);
    this.pivotMotor.refresh();

    this.rollerSim.update(Settings.DT);
    this.rollerLeaderMotor.refresh();
    this.rollerFollowerMotor.refresh();
  }

  // @Override
  // public void runPivotPosition(Angle position) {
  //   pivotMotor.setControl(pivotPositionController.withPosition(position));
  // }

  // @Override
  // public void runPivotTorqueCurrent(Current torqueCurrent) {
  //   pivotMotor.setControl(pivotPushdownController.withOutput(torqueCurrent));
  // }

  // @Override
  // public void runPivotVoltage(Voltage voltage) {
  //   pivotMotor.setControl(pivotVoltageController.withOutput(voltage));
  // }

  // @Override
  // public void runRollersDutyCycle(double dutyCycle) {
  //   rollerLeaderMotor.setControl(rollerLeaderController.withOutput(dutyCycle));
  // }

  // @Override
  // public void stopPivot() {
  //   pivotMotor.setControl(pivotVoltageController.withOutput(0));
  // }

  // @Override
  // public void stopRollers() {
  //   rollerLeaderMotor.stopMotor();
  //   rollerFollowerMotor.stopMotor();

  //   rollerFollowerMotor.setControl(rollerFollowerController);
  // }

  @Override
  public void seedPivotPosition(Angle position) {
    pivotMotor.setPosition(position);
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.pivotMode) {
      case POSITION -> pivotMotor.setControl(
          pivotPositionController.withPosition(outputs.pivotTargetPosition));
      case TORQUE_CURRENT -> pivotMotor.setControl(
          pivotPushdownController.withOutput(outputs.pivotTargetTorqueCurrent));
      case VOLTAGE -> pivotMotor.setControl(
          pivotVoltageController.withOutput(outputs.pivotTargetVoltage));
      case STOP -> pivotMotor.stopMotor();
    }

    switch (outputs.rollerMode) {
      case DUTY_CYCLE -> rollerLeaderMotor.setControl(
          rollerLeaderController.withOutput(outputs.rollerTargetDutyCycle));
      case STOP -> {
        rollerLeaderMotor.stopMotor();
        rollerFollowerMotor.stopMotor();
        rollerFollowerMotor.setControl(rollerFollowerController);
      }
    }
  }
}
