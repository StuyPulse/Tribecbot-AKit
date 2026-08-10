/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Gamepads
  public static final CommandXboxController driver =
      new CommandXboxController(Ports.Gamepad.DRIVER);

  // Subsystem
  private final Handoff handoff = Handoff.getInstance();
  private final Spindexer spindexer = Spindexer.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Hood hood = Hood.getInstance();
  private final Turret turret = Turret.getInstance();
  private final Drive drive = Drive.getInstance();
  private final Vision vision = Vision.getInstance();
  private final Superstructure superstructure = Superstructure.getInstance();

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

  private void configureDefaultCommands() {}

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {}

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    // autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
  }

  public Command getAutonomousCommand() {
    return autonChooser.get();
  }

  public void periodicAfterScheduler() {
    handoff.periodicAfterScheduler();
    spindexer.periodicAfterScheduler();
    shooter.periodicAfterScheduler();
    hood.periodicAfterScheduler();
    turret.periodicAfterScheduler();
    vision.periodicAfterScheduler();
    superstructure.periodicAfterScheduler();
  }
}
