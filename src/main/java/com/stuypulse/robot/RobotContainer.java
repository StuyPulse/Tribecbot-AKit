/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.DriveCommands;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LED;
import com.stuypulse.robot.subsystems.leds.LED.LEDState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Gamepads
  public static final CommandXboxController driver =
      new CommandXboxController(DriverConstants.Driver.DRIVER_INDEX);

  // Subsystem
  private final Handoff handoff = Handoff.getInstance();
  private final Spindexer spindexer = Spindexer.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Hood hood = Hood.getInstance();
  private final Turret turret = Turret.getInstance();
  private final Drive swerve = Drive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Vision vision = Vision.getInstance();
  private final Superstructure superstructure = Superstructure.getInstance();
  private final LED LEDs = LED.getInstance();

  // Autons
  private static LoggedDashboardChooser<Command> autonChooser =
      new LoggedDashboardChooser<>("Autonomous");

  // Robot container

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(DriveCommands.joystickDrive(driver));
    LEDs.setDefaultCommand(new LEDDefaultCommand());
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    // Shoot in place (TR)
    driver
        .y()
        .whileTrue(
            superstructure
                .cacheState(driver)
                .andThen(new WaitUntilCommand(superstructure::isReadyToShoot))
                .andThen(
                    Commands.parallel(
                        intake.deploy(),
                        new RunCommand(() -> handoff.setState(HandoffState.FORWARD), handoff),
                        new RunCommand(
                            () -> spindexer.setState(SpindexerState.FORWARD), spindexer))))
        .onFalse(spindexer.stopSpindexer().alongWith(handoff.stopHandoffCommand()));

    driver.povLeft().whileTrue(intake.teleopDigest().repeatedly()).onFalse(intake.deploy());

    // Intake Stow
    driver.leftTrigger().onTrue(intake.stow());

    // Intake Deploy
    driver
        .rightTrigger()
        .onTrue(intake.deploy());

    // Reset Heading
    driver
        .povUp()
        .onTrue(DriveCommands.resetHeading())
        .onTrue(vision.resetIMU())
        .onTrue(LEDs.setState(LEDState.RESET_HEADING))
        .onFalse(vision.setIMUMode(0));

    // Stop Rollers
    driver
        .leftBumper()
        .onTrue(LEDs.setState(LEDState.STOP_ROLLERS).withTimeout(2.0)) // This is done in the binding instead of the default command 
                                                                            //so that it doesn't persist/get overriden by the deploy state  
        .onTrue(intake.deploy().andThen(intake.stopRollersCommand()));

    // Outtake
    driver
        .rightBumper()
        .whileTrue(intake.outtake())
        .onFalse(intake.runRollers());

    // SOTM (BR)
    driver
        .start()
        .onTrue(
            new WaitUntilCommand(() -> spindexer.getState() == SpindexerState.FORWARD)
                .andThen(new WaitCommand(0.75).andThen(intake.deploy())))
        .whileTrue(
            new RepeatCommand(
                DriveCommands.buzzController(driver)
                    .onlyWhile(
                        () ->
                            !vision.hasData()
                                && superstructure.getState() == SuperstructureState.SOTM)))
        .onTrue(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    superstructure.stow(), spindexer.stopSpindexer(), handoff.stopHandoffCommand()),
                new ParallelCommandGroup(
                    superstructure
                        .SOTM()
                        .alongWith(new WaitUntilCommand(() -> superstructure.isReadyToShoot()))
                        .andThen(handoff.runHandoffForward())
                        .andThen(spindexer.runSpindexerForward()),
                    DriveCommands.driveSOTM(driver)),
                () ->
                    superstructure.getState() == SuperstructureState.SOTM
                        && swerve.canShootIntoHub()));

    // FOTM (BL)
    driver
        .back()
        .onTrue(intake.runRollers())
        .onTrue(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    superstructure.stow(), spindexer.stopSpindexer(), handoff.stopHandoffCommand()),
                new ParallelCommandGroup(
                    superstructure
                        .FOTM()
                        .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                        .andThen(handoff.runHandoffForward())
                        .andThen(spindexer.runSpindexerForward()),
                    DriveCommands.driveFOTM(driver)),
                () -> superstructure.getState() == SuperstructureState.FOTM));

    driver.povDown()
        .whileTrue(DriveCommands.xMode())
        .whileTrue(LEDs.setState(LEDState.X_WHEELS));    

    // Reset (TL)
    driver
        .povRight()
        .onTrue(
            superstructure
                .stow()
                .alongWith(handoff.stopHandoffCommand())
                .alongWith(spindexer.stopSpindexer()))
        .onTrue(LEDs.setState(LEDState.RESET_HEADING).withTimeout(2.0));

    // Manual Left Corner Scoring
    driver
        .x()
        .onTrue(
            new ParallelCommandGroup(
                intake.runRollers(),
                superstructure
                    .leftCorner()
                    .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                    .andThen(handoff.runHandoffForward())
                    .andThen(spindexer.runSpindexerForward()),
                // new SwerveResetPoseLeftCorner(),
                DriveCommands.xMode()))
        .onFalse(
            superstructure
                .stow()
                .alongWith(spindexer.stopSpindexer())
                .alongWith(handoff.stopHandoffCommand()));

    // Manual Right Corner Scoring
    driver
        .b()
        .whileTrue(DriveCommands.xMode())
        .onTrue(intake.runRollers())
        // .onTrue(new SwerveResetPoseRightCorner())
        .whileTrue(
            superstructure
                .rightCorner()
                .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(handoff.runHandoffForward())
                .alongWith(
                    new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD)
                        .andThen(spindexer.runSpindexerForward())))
        .onFalse(
            superstructure
                .stow()
                .alongWith(spindexer.stopSpindexer())
                .alongWith(handoff.stopHandoffCommand()));

    // Manual KB Distance Scoring
    driver
        .a()
        .whileTrue(DriveCommands.xMode())
        .onTrue(intake.runRollers())
        .onTrue(DriveCommands.resetPoseKBShot())
        .whileTrue(
            superstructure
                .kb()
                .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(handoff.runHandoffForward())
                .alongWith(
                    new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD)
                        .andThen(spindexer.runSpindexerForward())))
        .onFalse(
            superstructure
                .stow()
                .alongWith(spindexer.stopSpindexer())
                .alongWith(handoff.stopHandoffCommand()));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    // autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
  }

  public Command getAutonomousCommand() {
    return autonChooser.get();
  }

  public void clearMemoized() {
    superstructure.clearMemoized();
    swerve.clearMemoized();
    InterpolationCalculator.clearMemoized();
  }
}
