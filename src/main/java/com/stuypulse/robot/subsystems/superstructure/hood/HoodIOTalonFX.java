package com.stuypulse.robot.subsystems.superstructure.hood;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.Angles;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.MotorIds;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.MotorConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;

  private final PositionVoltage positionController;
  private final VoltageOut homingController;

  private final StatusSignal<Angle> hoodMotorPosition;
  private final StatusSignal<Current> hoodMotorSupplyCurrent;
  private final StatusSignal<Current> hoodMotorStatorCurrent;
  private final StatusSignal<Temperature> hoodMotorTemperature;
  private final StatusSignal<Voltage> hoodMotorAppliedVoltage;
  private final StatusSignal<AngularVelocity> hoodMotorVelocity;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(MotorIds.MOTOR, GlobalSettings.CANIVORE);

    MotorConfig.HOOD_CONFIG.configure(hoodMotor);    

    seedHoodPosition(Angles.STOW);

    positionController = new PositionVoltage(0).withEnableFOC(true);
    homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);

    hoodMotorPosition = hoodMotor.getPosition();
    hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodMotorStatorCurrent = hoodMotor.getStatorCurrent();
    hoodMotorTemperature = hoodMotor.getDeviceTemp();
    hoodMotorAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodMotorVelocity = hoodMotor.getVelocity();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodMotorPosition,
        hoodMotorSupplyCurrent,
        hoodMotorStatorCurrent,
        hoodMotorTemperature,
        hoodMotorAppliedVoltage,
        hoodMotorVelocity);

    inputs.hoodMotorPosition = hoodMotorPosition.getValue();
    inputs.hoodMotorSupplyCurrent = hoodMotorSupplyCurrent.getValue();
    inputs.hoodMotorStatorCurrent = hoodMotorStatorCurrent.getValue();
    inputs.hoodMotorTemperature = hoodMotorTemperature.getValue();
    inputs.hoodMotorAppliedVoltage = hoodMotorAppliedVoltage.getValue();
    inputs.hoodMotorVelocity = hoodMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.outputMode) {
      case POSITION -> hoodMotor.setControl(positionController.withPosition(outputs.position));

      case VOLTAGE -> hoodMotor.setControl(homingController.withOutput(outputs.voltage));

      case STOP -> hoodMotor.stopMotor();
    }
  }

  @Override
  public void seedHoodPosition(Angle position) {
    hoodMotor.setPosition(position);
  }
}
