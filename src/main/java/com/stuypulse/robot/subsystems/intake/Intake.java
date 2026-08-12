package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.util.DualDebouncer;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {
  private static final Intake instance;

  static {
    switch (GlobalSettings.currentMode) {
      case REAL -> instance = new Intake(new IntakeIOTalonFX());

      case SIM -> instance = new Intake(new IntakeIOSim());

      default -> instance = new Intake(new IntakeIO() {});
    }
  }

  public static Intake getInstance() {
    return instance;
  }

  @AutoLogOutput(key = "States/Intake/Pivot")
  private PivotState pivotState;

  @AutoLogOutput(key = "States/Intake/Rollers")
  private RollerState rollerState;

  private final DualDebouncer pivotPositionDebouncer;
  private final Debouncer pivotStallingDebouncer;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final IntakeIOOutputs outputs;

  private Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.outputs = new IntakeIOOutputs();
    this.pivotState = PivotState.STOW;
    this.rollerState = RollerState.STOP;

    this.pivotPositionDebouncer = new DualDebouncer(0.5, 0.1);
    this.pivotStallingDebouncer =
        new Debouncer(IntakeSettings.PIVOT_STALL_DEBOUNCE.in(Seconds), DebounceType.kBoth);
  }

  public enum PivotState {
    DEPLOY,
    HOMING,
    DIGEST,
    STOW;
  }

  public enum RollerState {
    INTAKE,
    OUTTAKE,
    STOP;
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public RollerState getRollerState() {
    return rollerState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (!GlobalSettings.EnabledSubsystems.INTAKE.get()) {
      stopPivot();
      stopRollers();

      return;
    }

    switch (pivotState) {
      case DEPLOY -> {
        if (isPivotBelowPushdownThreshold()) {
          Current pushdownCurrent =
              DriverStation.isTeleop()
                  ? IntakeSettings.PUSHDOWN_CURRENT_TELEOP
                  : IntakeSettings.PUSHDOWN_CURRENT_AUTON;

          runPivotTorqueCurrent(pushdownCurrent);
        } else {
          runPivotPosition(IntakeSettings.PIVOT_DEPLOY_ANGLE);
        }
      }

      case HOMING -> {
        if (pivotStalling()) {
          io.seedPivotPosition(IntakeSettings.PIVOT_MIN_ANGLE);
          setPivotState(PivotState.DEPLOY);
        } else {
          runPivotVoltage(IntakeSettings.HOMING_VOLTAGE);
        }
      }
      case DIGEST -> runPivotPosition(IntakeSettings.PIVOT_DIGEST_ANGLE);
      case STOW -> runPivotPosition(IntakeSettings.PIVOT_STOW_ANGLE);
    }

    if (pivotState == PivotState.DEPLOY
        && inputs.pivotMotorPosition.lte(IntakeSettings.THRESHOLD_TO_START_ROLLERS)) {
      switch (rollerState) {
        case INTAKE -> runRollersDutyCycle(1.0);
        case OUTTAKE -> runRollersDutyCycle(-1.0);
        case STOP -> stopRollers();
      }
    } else {
      stopRollers();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private boolean isPivotBelowPushdownThreshold() {
    return pivotPositionDebouncer.calculate(
        inputs.pivotMotorPosition.lte(IntakeSettings.ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE));
  }

  private boolean pivotStalling() {
    return pivotStallingDebouncer.calculate(
        inputs.pivotMotorStatorCurrent.abs(Amps) > IntakeSettings.PIVOT_STALL_CURRENT.in(Amps));
  }

  private void setPivotState(PivotState state) {
    this.pivotState = state;
  }

  private void setRollerState(RollerState state) {
    this.rollerState = state;
  }

  private void stopPivot() {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.STOP;
  }

  private void stopRollers() {
    outputs.rollerMode = IntakeIO.RollerIOOutputMode.STOP;
  }

  private void runPivotPosition(Angle position) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.POSITION;
    outputs.pivotTargetPosition = position;
  }

  private void runPivotTorqueCurrent(Current torqueCurrent) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.TORQUE_CURRENT;
    outputs.pivotTargetTorqueCurrent = torqueCurrent;
  }

  private void runPivotVoltage(Voltage voltage) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.VOLTAGE;
    outputs.pivotTargetVoltage = voltage;
  }

  private void runRollersDutyCycle(double dutyCycle) {
    outputs.rollerMode = IntakeIO.RollerIOOutputMode.DUTY_CYCLE;
    outputs.rollerTargetDutyCycle = dutyCycle;
  }

  public Command deploy() {
    return runOnce(
            () -> {
              setPivotState(PivotState.DEPLOY);
              setRollerState(RollerState.INTAKE);
            })
        .withName("Intake Deploy");
  }

  public Command stow() {
    return runOnce(
            () -> {
              setPivotState(PivotState.STOW);
              setRollerState(RollerState.STOP);
            })
        .withName("Intake Stow");
  }

  public Command home() {
    return runOnce(() -> setPivotState(PivotState.HOMING)).withName("Intake Home");
  }

  public Command digest() {
    return runOnce(
            () -> {
              setPivotState(PivotState.DIGEST);
              setRollerState(RollerState.INTAKE);
            })
        .withName("Intake Digest");
  }

  public Command autoDigest() {
    return digest()
        .andThen(Commands.waitSeconds(0.5))
        .andThen(deploy())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(digest().andThen(Commands.waitSeconds(0.5)).andThen(deploy()))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(digest().andThen(Commands.waitSeconds(0.5)).andThen(deploy()))
        .withName("Intake Auto Digest");
  }

  public Command teleopDigest() {
    return digest()
        .andThen(Commands.waitSeconds(0.5))
        .andThen(deploy())
        .andThen(Commands.waitSeconds(0.5))
        .withName("Intake Teleop Digest");
  }

  public Command outtake() {
    return runOnce(() -> setRollerState(RollerState.OUTTAKE)).withName("Intake Outtake");
  }

  public Command runRollers() {
    return runOnce(() -> setRollerState(RollerState.INTAKE)).withName("Intake Run Rollers");
  }

  public Command stopRollersCommand() {
    return runOnce(() -> setRollerState(RollerState.STOP)).withName("Intake Stop Rollers");
  }

  public Command seedPivotDeployed() {
    return runOnce(
            () -> {
              io.seedPivotPosition(IntakeSettings.PIVOT_DEPLOY_ANGLE);
              setPivotState(PivotState.DEPLOY);
            })
        .ignoringDisable(true)
        .withName("Intake Seed Pivot Deployed");
  }

  public Command seedPivotStowed() {
    return runOnce(
            () -> {
              io.seedPivotPosition(IntakeSettings.PIVOT_STOW_ANGLE);
              setPivotState(PivotState.STOW);
            })
        .ignoringDisable(true)
        .withName("Intake Seed Pivot Stowed");
  }
}
