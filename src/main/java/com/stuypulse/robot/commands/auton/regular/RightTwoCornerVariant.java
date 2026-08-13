/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.DriveCommands;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.Drive;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

public class RightTwoCornerVariant extends SequentialCommandGroup {
    
    public RightTwoCornerVariant(PathPlannerPath... paths) {

        addCommands(

            DriveCommands.resetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            // NZ Trip 1
            Drive.getInstance().followPathCommand(paths[0]).alongWith(
                new WaitCommand(0.2).andThen(Intake.getInstance().deploy())
            ),

            // Trip 1 To Score
            Drive.getInstance().followPathCommand(paths[1]).alongWith(
                Superstructure.getInstance().autoInterpolation()
            ),
            Superstructure.getInstance().SOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                Drive.getInstance().followPathCommand(paths[3]),
                Handoff.getInstance().runHandoffForward(),
                Spindexer.getInstance().runSpindexerForward(),
                new WaitCommand(0.5)
                    .andThen(Intake.getInstance().autoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(5.0)),
                new WaitCommand(1.0).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(4.5))
            ),
            Superstructure.getInstance().autoInterpolation().alongWith(Intake.getInstance().deploy()),

            // NZ Trip 2
            new ParallelCommandGroup(
                Drive.getInstance().followPathCommand(paths[2]),
                Handoff.getInstance().stopHandoffCommand(),
                Spindexer.getInstance().stopSpindexer()
            ),

            Superstructure.getInstance().SOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                Drive.getInstance().followPathCommand(paths[3]),
                Handoff.getInstance().runHandoffForward(),
                Spindexer.getInstance().runSpindexerForward(),
                new WaitCommand(0.5)
                    .andThen(Intake.getInstance().autoDigest().withTimeout(15.0)),
                new WaitCommand(15.0)
            ) 
        
        );

    }

}
