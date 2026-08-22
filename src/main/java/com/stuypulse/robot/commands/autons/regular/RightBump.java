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

public class RightBump extends SequentialCommandGroup {

    public RightBump(PathPlannerPath... paths) {

        addCommands(
                DriveCommands.resetPose(paths[0].getStartingHolonomicPose().get()),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),
                Superstructure.getInstance().SOTM(),
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                new ParallelCommandGroup(
                        Handoff.getInstance().runHandoffForward(),
                        Spindexer.getInstance().runSpindexerForward(),
                        Drive.getInstance().followPathCommand(paths[0]),
                        new WaitCommand(0.5)
                                .andThen(Intake.getInstance().deploy())
                                .andThen(new WaitCommand(1.0))),

                // NZ Trip 1
                new ParallelCommandGroup(
                        Drive.getInstance().followPathCommand(paths[1]),
                        Intake.getInstance().deploy(),
                        Handoff.getInstance().stopHandoffCommand(),
                        Spindexer.getInstance().stopSpindexer()),
                new WaitCommand(0.5),

                // SOTM To Depot
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                Handoff.getInstance()
                        .runHandoffForward()
                        .alongWith(Spindexer.getInstance().runSpindexerForward()),
                new ParallelCommandGroup(
                        Drive.getInstance().followPathCommand(paths[2]),
                        new WaitCommand(6.0)
                                .andThen(
                                        Handoff.getInstance()
                                                .stopHandoffCommand()
                                                .alongWith(
                                                        Spindexer.getInstance().stopSpindexer()))),
                new WaitCommand(0.5),

                // Off Depot
                new ParallelCommandGroup(
                        Handoff.getInstance()
                                .runHandoffForward()
                                .alongWith(Spindexer.getInstance().runSpindexerForward()),
                        new WaitCommand(3.0)
                                .andThen(Intake.getInstance().autoDigest().repeatedly())));
    }
}
