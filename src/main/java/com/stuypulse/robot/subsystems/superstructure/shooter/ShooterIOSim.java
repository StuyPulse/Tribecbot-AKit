package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final SystemSim<FlywheelSim> flywheelSim;

  private final TalonFXSimulation shooterLeaderSim;
  private final TalonFXSimulation shooterFollowerSim;

  private final VelocityTorqueCurrentFOC shooterLeaderController;
  private final Follower shooterFollowerController;

  private final StatusSignal<Angle> shooterLeaderSimPosition;
  private final StatusSignal<Current> shooterLeaderSimSupplyCurrent;
  private final StatusSignal<Current> shooterLeaderSimStatorCurrent;
  private final StatusSignal<Temperature> shooterLeaderSimTemperature;
  private final StatusSignal<Voltage> shooterLeaderSimAppliedVoltage;
  private final StatusSignal<AngularVelocity> shooterLeaderSimVelocity;

  private final StatusSignal<Angle> shooterFollowerSimPosition;
  private final StatusSignal<Current> shooterFollowerSimSupplyCurrent;
  private final StatusSignal<Current> shooterFollowerSimStatorCurrent;
  private final StatusSignal<Temperature> shooterFollowerSimTemperature;
  private final StatusSignal<Voltage> shooterFollowerSimAppliedVoltage;
  private final StatusSignal<AngularVelocity> shooterFollowerSimVelocity;

  public ShooterIOSim() {
    flywheelSim =
        SystemSim.of(
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX44(2), 0.05, Settings.Superstructure.Shooter.GEAR_RATIO),
                DCMotor.getKrakenX44(2),
                Settings.Superstructure.Shooter.GEAR_RATIO));

    shooterLeaderSim =
        new TalonFXSimulation(
            Ports.Superstructure.Shooter.MOTOR_LEAD,
            Settings.Superstructure.Shooter.GEAR_RATIO,
            flywheelSim);
    shooterFollowerSim =
        new TalonFXSimulation(
            Ports.Superstructure.Shooter.MOTOR_FOLLOW,
            Settings.Superstructure.Shooter.GEAR_RATIO,
            flywheelSim);

    shooterLeaderController = new VelocityTorqueCurrentFOC(0);
    shooterFollowerController =
        new Follower(shooterLeaderSim.getDeviceID(), MotorAlignmentValue.Opposed);

    shooterFollowerSim.setControl(shooterFollowerController);

    shooterLeaderSimPosition = shooterLeaderSim.getPosition();
    shooterLeaderSimSupplyCurrent = shooterLeaderSim.getSupplyCurrent();
    shooterLeaderSimStatorCurrent = shooterLeaderSim.getStatorCurrent();
    shooterLeaderSimTemperature = shooterLeaderSim.getDeviceTemp();
    shooterLeaderSimAppliedVoltage = shooterLeaderSim.getMotorVoltage();
    shooterLeaderSimVelocity = shooterLeaderSim.getVelocity();

    shooterFollowerSimPosition = shooterFollowerSim.getPosition();
    shooterFollowerSimSupplyCurrent = shooterFollowerSim.getSupplyCurrent();
    shooterFollowerSimStatorCurrent = shooterFollowerSim.getStatorCurrent();
    shooterFollowerSimTemperature = shooterFollowerSim.getDeviceTemp();
    shooterFollowerSimAppliedVoltage = shooterFollowerSim.getMotorVoltage();
    shooterFollowerSimVelocity = shooterFollowerSim.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterLeaderSim.refresh();
    shooterFollowerSim.refresh();

    BaseStatusSignal.refreshAll(
        shooterLeaderSimPosition,
        shooterLeaderSimSupplyCurrent,
        shooterLeaderSimStatorCurrent,
        shooterLeaderSimTemperature,
        shooterLeaderSimAppliedVoltage,
        shooterLeaderSimVelocity,
        shooterFollowerSimPosition,
        shooterFollowerSimSupplyCurrent,
        shooterFollowerSimStatorCurrent,
        shooterFollowerSimTemperature,
        shooterFollowerSimAppliedVoltage,
        shooterFollowerSimVelocity);

    inputs.shooterLeaderMotorPosition = shooterLeaderSimPosition.getValue();
    inputs.shooterLeaderMotorSupplyCurrent = shooterLeaderSimSupplyCurrent.getValue();
    inputs.shooterLeaderMotorStatorCurrent = shooterLeaderSimStatorCurrent.getValue();
    inputs.shooterLeaderMotorTemperature = shooterLeaderSimTemperature.getValue();
    inputs.shooterLeaderMotorAppliedVoltage = shooterLeaderSimAppliedVoltage.getValue();
    inputs.shooterLeaderMotorVelocity = shooterLeaderSimVelocity.getValue();

    inputs.shooterFollowerMotorPosition = shooterFollowerSimPosition.getValue();
    inputs.shooterFollowerMotorSupplyCurrent = shooterFollowerSimSupplyCurrent.getValue();
    inputs.shooterFollowerMotorStatorCurrent = shooterFollowerSimStatorCurrent.getValue();
    inputs.shooterFollowerMotorTemperature = shooterFollowerSimTemperature.getValue();
    inputs.shooterFollowerMotorAppliedVoltage = shooterFollowerSimAppliedVoltage.getValue();
    inputs.shooterFollowerMotorVelocity = shooterFollowerSimVelocity.getValue();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    shooterLeaderSim.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
  }
}
