package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.leds.LEDConstants.Settings;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.OperationMode;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.leds.LED;
import com.stuypulse.robot.subsystems.leds.LED.LEDState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDDefaultCommand extends InstantCommand {
  private final LED leds;
  private final Drive swerve;
  private final Handoff handoff;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Hood hood;
  private final Shooter shooter;
  private final Turret turret;
  private final Superstructure superstructure;
  private final Vision vision;

  public LEDDefaultCommand() {
    this.leds = LED.getInstance();
    this.swerve = Drive.getInstance();
    this.handoff = Handoff.getInstance();
    this.intake = Intake.getInstance();
    this.spindexer = Spindexer.getInstance();
    this.hood = Hood.getInstance();
    this.shooter = Shooter.getInstance();
    this.turret = Turret.getInstance();
    this.superstructure = Superstructure.getInstance();
    this.vision = Vision.getInstance();

    addRequirements(leds);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initialize() {
    if (Robot.getOperationMode() == OperationMode.DISABLED) {
      if (vision.getMaxTagCount() >= Settings.DESIRED_TAGS_WHEN_DISABLED) {
        leds.changeState(LEDState.DISABLED_ALIGNED);
      } else {
        leds.changeState(LEDState.DISABLED);
      }

      return;
    }

    if (swerve.isUnderTrench()) {
      leds.changeState(LEDState.PASSING_TRENCH);
    } else if (turret.isWrapping()) {
      leds.changeState(LEDState.TURRET_WRAPPING);
    } else if (superstructure.getState() == SuperstructureState.LEFT_CORNER) {
      leds.changeState(LEDState.LEFT_CORNER);
    } else if (superstructure.getState() == SuperstructureState.RIGHT_CORNER) {
      leds.changeState(LEDState.RIGHT_CORNER);
    } else if (superstructure.getState() == SuperstructureState.KB) {
      leds.changeState(LEDState.KB_DISTANCE);
    } else if (superstructure.getState() == SuperstructureState.SOTM) {
      leds.changeState(LEDState.SOTM_ON);
    } else if (superstructure.getState() == SuperstructureState.FOTM) {
      leds.changeState(LEDState.FOTM_ON);
    } else if (intake.getPivotState() == PivotState.STOW) {
      leds.changeState(LEDState.INTAKE_STOW);
    } else if (intake.getPivotState() == PivotState.DEPLOY) {
      leds.changeState(LEDState.INTAKE_DEPLOYED);
    }
  }
}
