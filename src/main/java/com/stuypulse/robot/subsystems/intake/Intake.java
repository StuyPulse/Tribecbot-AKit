package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.util.DualDebouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final Intake instance;

  static {
    if (Settings.currentMode == Settings.Mode.SIM) {
      instance = new Intake(new IntakeIOSim());
    } else {
      instance = new Intake(new IntakeIOTalonFX());
    }
  }

  public static Intake getInstance() {
    return instance;
  }

  private final DualDebouncer pivotDebouncer;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final IntakeIOOutputs outputs;

  private Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.outputs = new IntakeIOOutputs();

    this.pivotDebouncer = new DualDebouncer(0.5, 0.1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Intake", outputs.pivotOutputMode);
    io.applyOutputs(outputs);
  }

  private boolean isPivotBelowPushdownThreshold() {
    return pivotDebouncer.calculate(
        inputs.pivotMotorPosition.lte(Settings.Intake.ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE));
  }

  private void stopMotors() {
    outputs.pivotOutputMode = IntakeIO.PivotIOOutputMode.VOLTAGE;
    outputs.pivotVoltage = Volts.of(0);
    outputs.rollerDutyCycle = 0.0;
  }

  private void runPivotPosition(Angle position) {
    outputs.pivotOutputMode = IntakeIO.PivotIOOutputMode.POSITION;
    outputs.pivotPosition = position;
  }

  private void runPivotTorqueCurrent(Current torqueCurrent) {
    outputs.pivotOutputMode = IntakeIO.PivotIOOutputMode.TORQUE_CURRENT;
    outputs.pivotTorqueCurrent = torqueCurrent;
  }

  private void runPivotVoltage(Voltage voltage) {
    outputs.pivotOutputMode = IntakeIO.PivotIOOutputMode.VOLTAGE;
    outputs.pivotVoltage = voltage;
  }

  private void runRollersDutyCycle(double dutyCycle) {
    outputs.rollerDutyCycle = dutyCycle;
  }

  public Command runIntake() {
    if (!Settings.EnabledSubsystems.INTAKE.get()) {
      return run(this::stopMotors);
    }

    return run(
        () -> {
          if (inputs.pivotMotorPosition.lte(Settings.Intake.THRESHOLD_TO_START_ROLLERS)) {
            runRollersDutyCycle(1.0);
          } else {
            runRollersDutyCycle(0.0);
          }

          if (isPivotBelowPushdownThreshold()) {
            Current pushdownCurrent =
                DriverStation.isTeleop()
                    ? Settings.Intake.PUSHDOWN_CURRENT_TELEOP
                    : Settings.Intake.PUSHDOWN_CURRENT_AUTON;

            runPivotTorqueCurrent(pushdownCurrent);
          } else {
            runPivotPosition(Settings.Intake.PIVOT_DEPLOY_ANGLE);
          }
        });
  }

  public Command runStow() {
    if (!Settings.EnabledSubsystems.INTAKE.get()) {
      return run(this::stopMotors);
    }

    return run(
        () -> {
          runRollersDutyCycle(0.0);
          runPivotPosition(Settings.Intake.PIVOT_STOW_ANGLE);
        });
  }

  public Command runHoming() {
    if (!Settings.EnabledSubsystems.INTAKE.get()) {
      return run(this::stopMotors);
    }

    return run(() -> {
          runRollersDutyCycle(0.0);
          runPivotVoltage(Settings.Intake.HOMING_VOLTAGE);
        })
        .until(() -> inputs.pivotMotorStatorCurrent.gte(Settings.Intake.PIVOT_STALL_CURRENT))
        .andThen(() -> io.seedPivotPosition(Settings.Intake.PIVOT_MIN_ANGLE));
  }
}
