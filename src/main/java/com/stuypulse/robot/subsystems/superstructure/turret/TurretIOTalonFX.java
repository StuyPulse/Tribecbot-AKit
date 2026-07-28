package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;

  private final PositionVoltage positionController;

  private final StatusSignal<Angle> turretMotorPosition;
  private final StatusSignal<Current> turretMotorSupplyCurrent;
  private final StatusSignal<Current> turretMotorStatorCurrent;
  private final StatusSignal<Temperature> turretMotorTemperature;
  private final StatusSignal<Voltage> turretMotorAppliedVoltage;
  private final StatusSignal<AngularVelocity> turretMotorVelocity;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);

    positionController = new PositionVoltage(0).withEnableFOC(true);

    turretMotorPosition = turretMotor.getPosition();
    turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();
    turretMotorStatorCurrent = turretMotor.getStatorCurrent();
    turretMotorTemperature = turretMotor.getDeviceTemp();
    turretMotorAppliedVoltage = turretMotor.getMotorVoltage();
    turretMotorVelocity = turretMotor.getVelocity();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretMotorPosition,
        turretMotorSupplyCurrent,
        turretMotorStatorCurrent,
        turretMotorTemperature,
        turretMotorAppliedVoltage,
        turretMotorVelocity);
    inputs.turretMotorPosition = turretMotorPosition.getValue();
    inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValue();
    inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValue();
    inputs.turretMotorTemperature = turretMotorTemperature.getValue();
    inputs.turrentMotorAppliedVoltage = turretMotorAppliedVoltage.getValue();
    inputs.turretMotorVelocity = turretMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    turretMotor.setControl(positionController.withPosition(outputs.turretPosition));
  }
}
