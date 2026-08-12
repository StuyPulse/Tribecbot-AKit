package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class HoodIOSim implements HoodIO {

  private static SystemSim<ElevatorSim> hoodSim;

  private final TalonFXSimulation hoodMotor;

  private final PositionVoltage positionController;
  private final VoltageOut homingController;

  private final StatusSignal<Angle> hoodMotorPosition;
  private final StatusSignal<Current> hoodMotorSupplyCurrent;
  private final StatusSignal<Current> hoodMotorStatorCurrnet;
  private final StatusSignal<Temperature> hoodMotorTermperature;
  private final StatusSignal<Voltage> hoodMotorAppliedVoltage;
  private final StatusSignal<AngularVelocity> hoodMotorVelocity;

  public HoodIOSim() {
    hoodSim =
        SystemSim.of(
            new ElevatorSim(
                LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 1.0, HoodSettings.DRUM_RADIUS.in(Meters), 1.0),
                DCMotor.getKrakenX60(1),
                HoodSettings.MIN_HEIGHT.in(Meters),
                HoodSettings.MAX_HEIGHT.in(Meters),
                true,
                HoodSettings.MIN_HEIGHT.in(Meters),
                0.001,
                0.001),
            Meters.of(HoodSettings.DRUM_RADIUS.in(Meters)));

    hoodMotor = new TalonFXSimulation(HoodDeviceIds.MOTOR, 1.0, hoodSim);
    positionController = new PositionVoltage(0).withEnableFOC(true);
    homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);

    HoodMotorConfigs.HOOD_CONFIG.configure(hoodMotor);

    hoodMotorPosition = hoodMotor.getPosition();
    hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodMotorStatorCurrnet = hoodMotor.getStatorCurrent();
    hoodMotorTermperature = hoodMotor.getDeviceTemp();
    hoodMotorAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodMotorVelocity = hoodMotor.getVelocity();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodMotor.refresh();

    BaseStatusSignal.refreshAll(
        hoodMotorPosition,
        hoodMotorSupplyCurrent,
        hoodMotorStatorCurrnet,
        hoodMotorTermperature,
        hoodMotorAppliedVoltage,
        hoodMotorVelocity);
    inputs.hoodMotorPosition = hoodMotorPosition.getValue();
    inputs.hoodMotorSupplyCurrent = hoodMotorSupplyCurrent.getValue();
    inputs.hoodMotorStatorCurrent = hoodMotorStatorCurrnet.getValue();
    inputs.hoodMotorTemperature = hoodMotorTermperature.getValue();
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
