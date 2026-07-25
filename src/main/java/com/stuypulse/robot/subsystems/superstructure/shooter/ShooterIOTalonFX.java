package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;

  private final VelocityTorqueCurrentFOC shooterLeaderController;
  private final Follower shooterFollowerController;

  private final StatusSignal<Angle> shooterLeaderPosition;
  private final StatusSignal<Current> shooterLeaderSupplyCurrent;
  private final StatusSignal<Current> shooterLeaderStatorCurrent;
  private final StatusSignal<Temperature> shooterLeaderTemperature;
  private final StatusSignal<Voltage> shooterLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> shooterLeaderVelocity;

  private final StatusSignal<Angle> shooterFollowerPosition;
  private final StatusSignal<Current> shooterFollowerSupplyCurrent;
  private final StatusSignal<Current> shooterFollowerStatorCurrent;
  private final StatusSignal<Temperature> shooterFollowerTemperature;
  private final StatusSignal<Voltage> shooterFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> shooterFollowerVelocity;

  public ShooterIOTalonFX() {
    shooterLeader = new TalonFX(Ports.Superstructure.Shooter.MOTOR_LEAD);
    shooterFollower = new TalonFX(Ports.Superstructure.Shooter.MOTOR_FOLLOW);

    Motors.Superstructure.Shooter.SHOOTER_CONFIG.configure(shooterLeader);
    Motors.Superstructure.Shooter.SHOOTER_CONFIG.configure(shooterFollower);

    shooterLeaderController = new VelocityTorqueCurrentFOC(0);
    shooterFollowerController =
        new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    shooterFollower.setControl(shooterFollowerController);

    shooterLeaderPosition = shooterLeader.getPosition();
    shooterLeaderSupplyCurrent = shooterLeader.getSupplyCurrent();
    shooterLeaderStatorCurrent = shooterLeader.getStatorCurrent();
    shooterLeaderTemperature = shooterLeader.getDeviceTemp();
    shooterLeaderAppliedVoltage = shooterLeader.getMotorVoltage();
    shooterLeaderVelocity = shooterLeader.getVelocity();

    shooterFollowerPosition = shooterFollower.getPosition();
    shooterFollowerSupplyCurrent = shooterFollower.getSupplyCurrent();
    shooterFollowerStatorCurrent = shooterFollower.getStatorCurrent();
    shooterFollowerTemperature = shooterFollower.getDeviceTemp();
    shooterFollowerAppliedVoltage = shooterFollower.getMotorVoltage();
    shooterFollowerVelocity = shooterFollower.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterLeaderPosition,
        shooterLeaderSupplyCurrent,
        shooterLeaderStatorCurrent,
        shooterLeaderTemperature,
        shooterLeaderAppliedVoltage,
        shooterLeaderVelocity,
        shooterFollowerPosition,
        shooterFollowerSupplyCurrent,
        shooterFollowerStatorCurrent,
        shooterFollowerTemperature,
        shooterFollowerAppliedVoltage,
        shooterFollowerVelocity);

    inputs.shooterLeaderMotorPosition = shooterLeaderPosition.getValue();
    inputs.shooterLeaderMotorSupplyCurrent = shooterLeaderSupplyCurrent.getValue();
    inputs.shooterLeaderMotorStatorCurrent = shooterLeaderStatorCurrent.getValue();
    inputs.shooterLeaderMotorTemperature = shooterLeaderTemperature.getValue();
    inputs.shooterLeaderMotorAppliedVoltage = shooterLeaderAppliedVoltage.getValue();
    inputs.shooterLeaderMotorVelocity = shooterLeaderVelocity.getValue();

    inputs.shooterFollowerMotorPosition = shooterFollowerPosition.getValue();
    inputs.shooterFollowerMotorSupplyCurrent = shooterFollowerSupplyCurrent.getValue();
    inputs.shooterFollowerMotorStatorCurrent = shooterFollowerStatorCurrent.getValue();
    inputs.shooterFollowerMotorTemperature = shooterFollowerTemperature.getValue();
    inputs.shooterFollowerMotorAppliedVoltage = shooterFollowerAppliedVoltage.getValue();
    inputs.shooterFollowerMotorVelocity = shooterFollowerVelocity.getValue();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    shooterLeader.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
  }
}
