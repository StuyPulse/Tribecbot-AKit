package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final SystemSim<DCMotorSim> sim;
  private final PositionVoltage controller;
  private final TalonFXSimulation simMotor;

  private StatusSignal<Current> turretSimMotorSupplyCurrent;
  private StatusSignal<Current> turretSimMotorStatorCurrent;
  private StatusSignal<Temperature> turretSimMotorTemperature;
  private StatusSignal<Angle> turretSimMotorPosition;
  private StatusSignal<Voltage> turretSimMotorAppliedVoltage;
  private StatusSignal<AngularVelocity> turretSimMotorVelocity;

  public TurretIOSim() {
    sim =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 2.8),
                DCMotor.getKrakenX60(1),
                1.0,
                2.8));

    controller = new PositionVoltage(0).withEnableFOC(true);

    simMotor =
        new TalonFXSimulation(
            Ports.Superstructure.Turret.MOTOR,
            Settings.Superstructure.Turret.GEAR_RATIO_MOTOR_TO_MECH,
            sim);

    turretSimMotorPosition = simMotor.getPosition();
    turretSimMotorSupplyCurrent = simMotor.getSupplyCurrent();
    turretSimMotorStatorCurrent = simMotor.getStatorCurrent();
    turretSimMotorTemperature = simMotor.getDeviceTemp();
    turretSimMotorAppliedVoltage = simMotor.getMotorVoltage();
    turretSimMotorVelocity = simMotor.getVelocity();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretSimMotorPosition,
        turretSimMotorSupplyCurrent,
        turretSimMotorStatorCurrent,
        turretSimMotorTemperature,
        turretSimMotorAppliedVoltage,
        turretSimMotorVelocity);
    inputs.turretMotorPosition = turretSimMotorPosition.getValue();
    inputs.turretMotorSupplyCurrent = turretSimMotorSupplyCurrent.getValue();
    inputs.turretMotorStatorCurrent = turretSimMotorStatorCurrent.getValue();
    inputs.turretMotorTemperature = turretSimMotorTemperature.getValue();
    inputs.turretMotorAppliedVoltage = turretSimMotorAppliedVoltage.getValue();
    inputs.turretMotorVelocity = turretSimMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    simMotor.setControl(controller.withPosition(outputs.turretPosition));
  }
}
