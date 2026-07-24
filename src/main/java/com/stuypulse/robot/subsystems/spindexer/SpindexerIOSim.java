package com.stuypulse.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpindexerIOSim implements SpindexerIO {
  private final DCMotorSim spindexerLeaderSim;
  private final DCMotorSim spindexerFollowerSim;

  public SpindexerIOSim() {
    this.spindexerLeaderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1),
            DCMotor.getKrakenX60(1));

    this.spindexerFollowerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spindexerLeaderMotorAppliedVoltage = Volts.of(spindexerLeaderSim.getInputVoltage());
    inputs.spindexerLeaderMotorPosition = spindexerLeaderSim.getAngularPosition();
    inputs.spindexerLeaderMotorStatorCurrent = Amps.of(spindexerLeaderSim.getCurrentDrawAmps());
    inputs.spindexerLeaderMotorSupplyCurrent = Amps.of(spindexerLeaderSim.getCurrentDrawAmps());
    inputs.spindexerLeaderMotorVelocity = spindexerLeaderSim.getAngularVelocity();

    inputs.spindexerFollowerMotorAppliedVoltage = Volts.of(spindexerFollowerSim.getInputVoltage());
    inputs.spindexerFollowerMotorPosition = spindexerFollowerSim.getAngularPosition();
    inputs.spindexerFollowerMotorStatorCurrent = Amps.of(spindexerFollowerSim.getCurrentDrawAmps());
    inputs.spindexerFollowerMotorSupplyCurrent = Amps.of(spindexerFollowerSim.getCurrentDrawAmps());
    inputs.spindexerFollowerMotorVelocity = spindexerFollowerSim.getAngularVelocity();
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    // TODO:Enabled Subsystems Check
    // TODO:Possibly battery sim?
    spindexerLeaderSim.setInputVoltage(12 * outputs.spindexerLeaderDutyCycle);
    spindexerLeaderSim.update(0.02);

    spindexerFollowerSim.setInputVoltage(12 * outputs.spindexerLeaderDutyCycle);
    spindexerLeaderSim.update(0.02);
  }
}
