/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // Gamepads
    public final CommandXboxController driver = new CommandXboxController(Ports.Gamepad.DRIVER);
    
    // Subsystem

    // Autons
    private static LoggedDashboardChooser<Command> autonChooser = new LoggedDashboardChooser<>("Autonomous");

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
}