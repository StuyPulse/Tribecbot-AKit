package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {
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

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);

    turretMotor.getClosedLoopError().setUpdateFrequency(Hertz.of(50));

    encoder17t = new CANcoder(Ports.Superstructure.Turret.ENCODER17T, Ports.RIO);
    encoder18t = new CANcoder(Ports.Superstructure.Turret.ENCODER18T, Ports.RIO);

    Motors.Superstructure.Turret.TURRET_CONFIG.configure(turretMotor);
    Motors.Superstructure.Turret.ENCODER_17T_CONFIG.configure(encoder17t);
    Motors.Superstructure.Turret.ENCODER_18T_CONFIG.configure(encoder18t);

    positionController = new PositionVoltage(0).withEnableFOC(true);

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
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.turretMode) {
      case POSITION -> turretMotor.setControl(
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
  public void refreshMagnetSensorConfigurations(
      MagnetSensorConfigs encoder17tConfigs, MagnetSensorConfigs encoder18tConfigs) {
    encoder17t.getConfigurator().refresh(encoder17tConfigs);
    encoder18t.getConfigurator().refresh(encoder18tConfigs);
  }

  @Override
  public void configureEncoders() {
    Motors.Superstructure.Turret.ENCODER_17T_CONFIG.configure(encoder17t);
    Motors.Superstructure.Turret.ENCODER_18T_CONFIG.configure(encoder18t);
  }
}
