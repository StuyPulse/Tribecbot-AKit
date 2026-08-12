package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final SystemSim<DCMotorSim> turretSim;
  private final PositionVoltage controller;
  private final TalonFXSimulation turretMotor;

  private StatusSignal<Current> turretSimMotorSupplyCurrent;
  private StatusSignal<Current> turretSimMotorStatorCurrent;
  private StatusSignal<Temperature> turretSimMotorTemperature;
  private StatusSignal<Angle> turretSimMotorPosition;
  private StatusSignal<Voltage> turretSimMotorAppliedVoltage;
  private StatusSignal<AngularVelocity> turretSimMotorVelocity;

  public TurretIOSim() {
    turretSim =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 2.8),
                DCMotor.getKrakenX60(1),
                1.0,
                2.8));

    controller = new PositionVoltage(0).withEnableFOC(true);

    turretMotor =
        new TalonFXSimulation(
            TurretDeviceIds.MOTOR, TurretSettings.GEAR_RATIO_MOTOR_TO_MECH, turretSim);
    TurretMotorConfigs.TURRET_CONFIG.configure(turretMotor);

    turretSimMotorPosition = turretMotor.getPosition();
    turretSimMotorSupplyCurrent = turretMotor.getSupplyCurrent();
    turretSimMotorStatorCurrent = turretMotor.getStatorCurrent();
    turretSimMotorTemperature = turretMotor.getDeviceTemp();
    turretSimMotorAppliedVoltage = turretMotor.getMotorVoltage();
    turretSimMotorVelocity = turretMotor.getVelocity();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretSim.update(GlobalSettings.DT);
    turretMotor.refresh();

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
    switch (outputs.turretMode) {
      case POSITION -> turretMotor.setControl(
          controller
              .withPosition(outputs.turretPosition)
              .withSlot(outputs.gainSlot)
              .withFeedForward(outputs.feedForward));

      case STOP -> turretMotor.stopMotor();
    }
  }
}
