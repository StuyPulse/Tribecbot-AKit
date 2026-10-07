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

public class RightFollow extends SequentialCommandGroup {

    public RightFollow(PathPlannerPath... paths) {

        addCommands(
                DriveCommands.resetPose(paths[0].getStartingHolonomicPose().get()),

                // Preloads
                Superstructure.getInstance().SOTM(),
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                new ParallelCommandGroup(
                        Handoff.getInstance().runHandoffForward(),
                        Spindexer.getInstance().runSpindexerForward(),
                        Commands.defer(
                                () -> new WaitCommand(RobotContainer.getWaitTimeOne() + 1.0),
                                Set.of()),
                        new WaitCommand(1.0).andThen(Intake.getInstance().deploy())),

                // To NZ
                new ParallelCommandGroup(
                        Handoff.getInstance().stopHandoffCommand(),
                        Spindexer.getInstance().stopSpindexer(),
                        Drive.getInstance().followPathCommand(paths[0])),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeTwo()), Set.of()),

                // SOTM To Corner
                new ParallelCommandGroup(
                        Drive.getInstance().followPathCommand(paths[1]),
                        new WaitCommand(3.0)
                                .andThen(
                                        new WaitUntilCommand(
                                                        () ->
                                                                Superstructure.getInstance()
                                                                        .atTolerance())
                                                .andThen(
                                                        new ParallelCommandGroup(
                                                                Handoff.getInstance()
                                                                        .runHandoffForward(),
                                                                Spindexer.getInstance()
                                                                        .runSpindexerForward())))),
                Intake.getInstance().autoDigest().repeatedly());
    }
}
