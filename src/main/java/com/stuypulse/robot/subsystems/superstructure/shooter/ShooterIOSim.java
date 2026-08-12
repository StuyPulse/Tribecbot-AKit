package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.MotorConfig;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.ShooterSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.MotorIds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

  private final TalonFXSimulation shooterLeaderMotor;
  private final TalonFXSimulation shooterFollowerMotor;

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
                    DCMotor.getKrakenX44(2),
                    0.05,
                    ShooterSettings.GEAR_RATIO),
                DCMotor.getKrakenX44(2),
                ShooterSettings.GEAR_RATIO));

    shooterLeaderMotor =
        new TalonFXSimulation(
            MotorIds.MOTOR_LEAD,
            ShooterSettings.GEAR_RATIO,
            flywheelSim);
    shooterFollowerMotor =
        new TalonFXSimulation(
            MotorIds.MOTOR_FOLLOW,
            ShooterSettings.GEAR_RATIO,
            flywheelSim);

    MotorConfig.SHOOTER_CONFIG.configure(shooterLeaderMotor);
    MotorConfig.SHOOTER_CONFIG.configure(shooterFollowerMotor);

    shooterLeaderController = new VelocityTorqueCurrentFOC(0);
    shooterFollowerController =
        new Follower(shooterLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);

    shooterFollowerMotor.setControl(shooterFollowerController);

    shooterLeaderSimPosition = shooterLeaderMotor.getPosition();
    shooterLeaderSimSupplyCurrent = shooterLeaderMotor.getSupplyCurrent();
    shooterLeaderSimStatorCurrent = shooterLeaderMotor.getStatorCurrent();
    shooterLeaderSimTemperature = shooterLeaderMotor.getDeviceTemp();
    shooterLeaderSimAppliedVoltage = shooterLeaderMotor.getMotorVoltage();
    shooterLeaderSimVelocity = shooterLeaderMotor.getVelocity();

    shooterFollowerSimPosition = shooterFollowerMotor.getPosition();
    shooterFollowerSimSupplyCurrent = shooterFollowerMotor.getSupplyCurrent();
    shooterFollowerSimStatorCurrent = shooterFollowerMotor.getStatorCurrent();
    shooterFollowerSimTemperature = shooterFollowerMotor.getDeviceTemp();
    shooterFollowerSimAppliedVoltage = shooterFollowerMotor.getMotorVoltage();
    shooterFollowerSimVelocity = shooterFollowerMotor.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(GlobalSettings.DT);
    shooterLeaderMotor.refresh();
    shooterFollowerMotor.refresh();

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
    shooterLeaderMotor.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
  }
}
