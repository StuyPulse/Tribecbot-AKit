/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.autons.regular;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.DriveCommands;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.Drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.Set;

public class Depot extends SequentialCommandGroup {

    public Depot(PathPlannerPath... paths) {

        addCommands(
                DriveCommands.resetPose(paths[0].getStartingHolonomicPose().get()),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),
                Superstructure.getInstance().SOTM(),
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                Handoff.getInstance()
                        .runHandoffForward()
                        .alongWith(Spindexer.getInstance().runSpindexerForward()),
                new ParallelCommandGroup(
                        Drive.getInstance().followPathCommand(paths[0]),
                        new WaitCommand(4.5)
                                .andThen(
                                        Handoff.getInstance()
                                                .stopHandoffCommand()
                                                .alongWith(
                                                        Spindexer.getInstance().stopSpindexer())),
                        new WaitCommand(0.5).andThen(Intake.getInstance().deploy())),
                new WaitCommand(0.5),
                Handoff.getInstance()
                        .runHandoffForward()
                        .alongWith(Spindexer.getInstance().runSpindexerForward()));
    }
}
