package com.stuypulse.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.Motors;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class HoodIOSim implements HoodIO {

  private static SystemSim<ElevatorSim> hoodSim;
  private static final double HOOD_ARM_LENGTH_METERS = 0.3;

  private static final double MIN_HEIGHT =
      HOOD_ARM_LENGTH_METERS * Math.sin(Settings.Superstructure.Hood.Angles.MIN.in(Radians));
  private static final double MAX_HEIGHT =
      HOOD_ARM_LENGTH_METERS * Math.sin(Settings.Superstructure.Hood.Angles.MAX.in(Radians));

  private static final double DRUM_RADIUS = 0.01;

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
                LinearSystemId.createElevatorSystem(
                    DCMotor.getKrakenX60Foc(1), 1.0, DRUM_RADIUS, 1.0),
                DCMotor.getKrakenX60Foc(1),
                MIN_HEIGHT,
                MAX_HEIGHT,
                true,
                MIN_HEIGHT,
                0.001,
                0.001),
            Meters.of(DRUM_RADIUS));

    hoodMotor = new TalonFXSimulation(Ports.Superstructure.Hood.MOTOR, 1.0, hoodSim);
    positionController = new PositionVoltage(0).withEnableFOC(true);
    homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);

    hoodMotor.configure(Motors.Superstructure.Hood.HOOD_CONFIG);

    hoodMotorPosition = hoodMotor.getPosition();
    hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodMotorStatorCurrnet = hoodMotor.getStatorCurrent();
    hoodMotorTermperature = hoodMotor.getDeviceTemp();
    hoodMotorAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodMotorVelocity = hoodMotor.getVelocity();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.update(Settings.DT);
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
