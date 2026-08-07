package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandoffIOSim implements HandoffIO {

  private final DCMotorSim motorLeaderSim;
  private final DCMotorSim motorFollowerSim;

  HandoffIOSim() {

    motorLeaderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 0.01),
            DCMotor.getKrakenX60(1));

    motorFollowerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 0.01),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(HandoffIOInputs inputs) {
    inputs.motorLeadSupplyCurrent = Amps.of(motorLeaderSim.getCurrentDrawAmps());
    inputs.motorLeadStatorCurrent = Amps.of(motorLeaderSim.getCurrentDrawAmps());
    inputs.motorLeadVelocity = DegreesPerSecond.of(motorLeaderSim.getAngularVelocityRadPerSec());
    inputs.motorLeadAppliedVoltage = Volts.of(motorLeaderSim.getInputVoltage());

    inputs.motorFollowSupplyCurrent = Amps.of(motorFollowerSim.getCurrentDrawAmps());
    inputs.motorFollowStatorCurrent = Amps.of(motorFollowerSim.getCurrentDrawAmps());
    inputs.motorFollowVelocity =
        DegreesPerSecond.of(motorFollowerSim.getAngularVelocityRadPerSec());
    inputs.motorFollowAppliedVoltage = Volts.of(motorFollowerSim.getInputVoltage());
  }

  @Override
  public void applyOutputs(HandoffIOOutputs outputs) {
    motorLeaderSim.setInputVoltage(12 * outputs.handoffDutyCycle);
    motorLeaderSim.update(0.02);

    motorFollowerSim.setInputVoltage(12 * outputs.handoffDutyCycle);
    motorFollowerSim.update(0.02);
  }
}
