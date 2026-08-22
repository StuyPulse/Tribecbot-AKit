/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.autons.DoNothingAuton;
import com.stuypulse.robot.commands.autons.regular.TwoCorner;
import com.stuypulse.robot.commands.autons.regular.TwoCornerShallow;

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
import com.stuypulse.robot.util.PathUtil.AutonConfig;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
    private static LoggedNetworkNumber waitTimeOne =
            new LoggedNetworkNumber("/Tuning/Auton/Wait Time 1", 0.0);
    private static LoggedNetworkNumber waitTimeTwo =
            new LoggedNetworkNumber("/Tuning/Auton/Wait Time 2", 0.0);

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
        driver.y()
                .whileTrue(
                        superstructure
                                .cacheState(driver)
                                .andThen(new WaitUntilCommand(superstructure::isReadyToShoot))
                                .andThen(
                                        Commands.parallel(
                                                intake.deploy(),
                                                new RunCommand(
                                                        () ->
                                                                handoff.setState(
                                                                        HandoffState.FORWARD),
                                                        handoff),
                                                new RunCommand(
                                                        () ->
                                                                spindexer.setState(
                                                                        SpindexerState.FORWARD),
                                                        spindexer))))
                .onFalse(spindexer.stopSpindexer().alongWith(handoff.stopHandoffCommand()));

        driver.povLeft().whileTrue(intake.teleopDigest().repeatedly()).onFalse(intake.deploy());

        // Intake Stow
        driver.leftTrigger().onTrue(intake.stow());

        // Intake Deploy
        driver.rightTrigger().onTrue(intake.deploy());

        // Reset Heading
        driver.povUp()
                .onTrue(DriveCommands.resetHeading())
                .onTrue(vision.resetIMU())
                .onTrue(LEDs.setState(LEDState.RESET_HEADING))
                .onFalse(vision.setIMUMode(0));

        // Stop Rollers
        driver.leftBumper()
                .onTrue(
                        LEDs.setState(LEDState.STOP_ROLLERS)
                                .withTimeout(
                                        2.0)) // This is done in the binding instead of the default
                // command
                // so that it doesn't persist/get overriden by the deploy state
                .onTrue(intake.deploy().andThen(intake.stopRollersCommand()));

        // Outtake
        driver.rightBumper().whileTrue(intake.outtake()).onFalse(intake.runRollers());

        // SOTM (BR)
        driver.start()
                .onTrue(
                        new WaitUntilCommand(() -> spindexer.getState() == SpindexerState.FORWARD)
                                .andThen(new WaitCommand(0.75).andThen(intake.deploy())))
                .whileTrue(
                        new RepeatCommand(
                                DriveCommands.buzzController(driver)
                                        .onlyWhile(
                                                () ->
                                                        !vision.hasData()
                                                                && superstructure.getState()
                                                                        == SuperstructureState
                                                                                .SOTM)))
                .onTrue(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        superstructure.stow(),
                                        spindexer.stopSpindexer(),
                                        handoff.stopHandoffCommand()),
                                new ParallelCommandGroup(
                                        superstructure
                                                .SOTM()
                                                .alongWith(
                                                        new WaitUntilCommand(
                                                                () ->
                                                                        superstructure
                                                                                .isReadyToShoot()))
                                                .andThen(handoff.runHandoffForward())
                                                .andThen(spindexer.runSpindexerForward()),
                                        DriveCommands.driveSOTM(driver)),
                                () ->
                                        superstructure.getState() == SuperstructureState.SOTM
                                                && swerve.canShootIntoHub()));

        // FOTM (BL)
        driver.back()
                .onTrue(intake.runRollers())
                .onTrue(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        superstructure.stow(),
                                        spindexer.stopSpindexer(),
                                        handoff.stopHandoffCommand()),
                                new ParallelCommandGroup(
                                        superstructure
                                                .FOTM()
                                                .alongWith(
                                                        new WaitUntilCommand(
                                                                () -> superstructure.atTolerance()))
                                                .andThen(handoff.runHandoffForward())
                                                .andThen(spindexer.runSpindexerForward()),
                                        DriveCommands.driveFOTM(driver)),
                                () -> superstructure.getState() == SuperstructureState.FOTM));

        driver.povDown()
                .whileTrue(DriveCommands.xMode())
                .whileTrue(LEDs.setState(LEDState.X_WHEELS));

        // Reset (TL)
        driver.povRight()
                .onTrue(
                        superstructure
                                .stow()
                                .alongWith(handoff.stopHandoffCommand())
                                .alongWith(spindexer.stopSpindexer()))
                .onTrue(LEDs.setState(LEDState.RESET_HEADING).withTimeout(2.0));

        // Manual Left Corner Scoring
        driver.x()
                .onTrue(
                        new ParallelCommandGroup(
                                intake.runRollers(),
                                superstructure
                                        .leftCorner()
                                        .alongWith(
                                                new WaitUntilCommand(
                                                        () -> superstructure.atTolerance()))
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
        driver.b()
                .whileTrue(DriveCommands.xMode())
                .onTrue(intake.runRollers())
                // .onTrue(new SwerveResetPoseRightCorner())
                .whileTrue(
                        superstructure
                                .rightCorner()
                                .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                                .andThen(handoff.runHandoffForward())
                                .alongWith(
                                        new WaitUntilCommand(
                                                        () ->
                                                                handoff.getState()
                                                                        == HandoffState.FORWARD)
                                                .andThen(spindexer.runSpindexerForward())))
                .onFalse(
                        superstructure
                                .stow()
                                .alongWith(spindexer.stopSpindexer())
                                .alongWith(handoff.stopHandoffCommand()));

        // Manual KB Distance Scoring
        driver.a()
                .whileTrue(DriveCommands.xMode())
                .onTrue(intake.runRollers())
                .onTrue(DriveCommands.resetPoseKBShot())
                .whileTrue(
                        superstructure
                                .kb()
                                .alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                                .andThen(handoff.runHandoffForward())
                                .alongWith(
                                        new WaitUntilCommand(
                                                        () ->
                                                                handoff.getState()
                                                                        == HandoffState.FORWARD)
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
        autonChooser.addDefaultOption("Do Nothing", new DoNothingAuton());

        // DEPOT
        // AutonConfig DEPOT_ONLY = new AutonConfig("Depot Only", Depot::new,
        // "Center Hub To Depot");
        // DEPOT_ONLY.register(autonChooser);

        // AutonConfig LEFT_BUMP = new AutonConfig("Left Bump", Bump::new,
        // "Left Bump To Score (Start)", "Left Bump To Score", "Left Bump Score To Depot");
        // LEFT_BUMP.register(autonChooser);

        // AutonConfig RIGHT_BUMP = new AutonConfig("Right Bump", RightBump::new,
        // "Right Bump To Score (Start)", "Right Bump To Score", "Right Bump Score To Depot");
        // RIGHT_BUMP.register(autonChooser);

        // // TWO CYCLES (TRENCH)
        // AutonConfig LEFT_TWO_CYCLE = new AutonConfig("Left Two Cycle", LeftTwoCycle::new,
        // "Left Trench To NZ", "Left NZ To Score", "Left Score To Score", "Left Score To Corner",
        // "Left Score To NZ (F)");
        // LEFT_TWO_CYCLE.register(autonChooser);

        // AutonConfig RIGHT_TWO_CYCLE = new AutonConfig("Right Two Cycle", RightTwoCycle::new,
        // "Right Trench To NZ", "Right NZ To Score", "Right Score To Score", "Right Score To
        // Corner", "Right Score To NZ (F)");
        // RIGHT_TWO_CYCLE.register(autonChooser);

        // AutonConfig BC_TEST = new AutonConfig("BC Test", TestBC::new, "Right Corner to Dot");
        // BC_TEST.register(autonChooser);

        // // TWO CYCLES (CORNER)
        // AutonConfig L_CN_FN = new AutonConfig("Left Corner-Near Far-Near", TwoCorner::new,
        // "Left Corner Bite Anti Collision", "Left NZ To Score Anti Collision", "Left Bite Score To
        // Score", "Left Score To Corner", "Left Score To NZ (F)");
        // L_CN_FN.register(autonChooser);

        // AutonConfig R_CN_FN = new AutonConfig("Right Corner-Near Far-Near", RightTwoCorner::new,
        // "Right Corner Bite Anti Collision", "Right NZ To Score Anti Collision", "Right Bite Score
        // To Score", "Right Score To Corner", "Right Score To NZ (F)");
        // R_CN_FN.register(autonChooser);

        // AutonConfig L_FNS_FN = new AutonConfig("Left Far-Near Shallow Far-Near",
        // TwoCornerShallow::new,
        // "Left To Shallow", "Left Shallow To Score", "Left Bite Score To Score", "Left Score To
        // Corner", "Left Score To NZ (F)");
        // L_FNS_FN.register(autonChooser);

        // AutonConfig R_FNS_FN = new AutonConfig("Right Far-Near Shallow Far-Near",
        // TwoCornerShallow::new,
        // "Right To Shallow", "Right Shallow To Score", "Right Bite Score To Score", "Right Score
        // To Corner", "Right Score To NZ (F)");
        // R_FNS_FN.register(autonChooser);

        // AutonConfig L_CN_NF = new AutonConfig("Left Corner-Near Near-Far",
        // LeftTwoCornerVariant::new,
        // "Left Corner Bite Anti Collision", "Left NZ To Score Anti Collision", "Left Score To
        // Score", "Left Score To Corner", "Left Score To NZ (F)");
        // L_CN_NF.register(autonChooser);

        // AutonConfig R_CN_NF = new AutonConfig("Right Corner-Near Near-Far",
        // RightTwoCornerVariant::new,
        // "Right Corner Bite Anti Collision", "Right NZ To Score Anti Collision", "Right Score To
        // Score", "Right Score To Corner", "Right Score To NZ (F)");
        // R_CN_NF.register(autonChooser);

        // AutonConfig L_CNL_D = new AutonConfig("Left Corner-Near Long Dot", ShallowSwipeDot::new,
        //     "BC Left To Shallow", "BC Left Shallow To Score", "Left Score To Corner", "Left
        // Corner To Dot Straight"
        // );
        // L_CNL_D.register(autonChooser);

        // AutonConfig R_CNL_D = new AutonConfig("Right Corner-Near Long Dot", ShallowSwipeDot::new,
        //     "BC Right To Shallow", "BC Right Shallow To Score", "Right Score To Corner", "Right
        // Corner To Dot Straight"
        // );
        // R_CNL_D.register(autonChooser);

        // RAN AT BATTLE CRY
        // AutonConfig R_CN_NFS = new AutonConfig("Right Corner-Near Near-Far-Short",
        // TwoCornerShallow::new,
        // "Right Corner Bite Anti Collision", "Right NZ To Score Anti Collision", "BC Right Score
        // To Score NY", "Right Score To Corner", "Right Score To NZ (F)");
        // R_CN_NFS.register(autonChooser);

        // AutonConfig L_CN_NFS = new AutonConfig("Left Corner-Near Near-Far-Short",
        // TwoCornerShallow::new,
        // "Left Corner Bite Anti Collision", "Left NZ To Score Anti Collision", "BC Left Score To
        // Score NY", "Left Score To Corner", "Left Score To NZ (F)");
        // L_CN_NFS.register(autonChooser);

        // might be a duplicate of Right Far Near Shallow Far Near - if no changes to that were made
        AutonConfig Right_Champs =
                new AutonConfig(
                        "Right Champs",
                        TwoCorner::new,
                        "Champs Right To Shallow",
                        "Champs Right Shallow To Score Wide",
                        "Champs Right Bite Score To Score",
                        "Champs Right Score To Corner");
        Right_Champs.register(autonChooser);

        AutonConfig Left_Champs =
                new AutonConfig(
                        "Left Champs",
                        TwoCorner::new,
                        "Champs Left To Shallow",
                        "Champs Left Shallow To Score Wide",
                        "Champs Left Bite Score To Score",
                        "Champs Left Score To Corner");
        Left_Champs.register(autonChooser);

        AutonConfig Right_NY =
                new AutonConfig(
                        "Right NY",
                        TwoCorner::new,
                        "NY Right Trench To NZ",
                        "NY Right NZ To Score",
                        "NY Right Score To Score",
                        "Right Score To Corner");
        Right_NY.register(autonChooser);

        AutonConfig Left_NY =
                new AutonConfig(
                        "Left NY",
                        TwoCorner::new,
                        "NY Left Trench To NZ",
                        "NY Left NZ To Score",
                        "NY Left Score To Score",
                        "Left Score To Corner");
        Left_NY.register(autonChooser);

        AutonConfig Right_Champs_NY =
                new AutonConfig(
                        "Right Champs NY",
                        TwoCorner::new,
                        "316 Champs Right To Shallow",
                        "316 Champs Right Shallow To Score Wide",
                        "316 NY Right Score To Score",
                        "Right Score To Corner");
        Right_Champs_NY.register(autonChooser);

        AutonConfig Left_Champs_NY =
                new AutonConfig(
                        "Left Champs NY",
                        TwoCorner::new,
                        "316 Champs Left To Shallow",
                        "316 Champs Left Shallow To Score Wide",
                        "316 NY Left Score To Score",
                        "Left Score To Corner");
        Left_Champs_NY.register(autonChooser);

        // BC Score To Score NY is a shorened version of NY and w the slow down
        AutonConfig Right_BC =
                new AutonConfig(
                        "Right BC",
                        TwoCornerShallow::new,
                        "Right Corner Bite Anti Collision",
                        "Right NZ To Score Anti Collision",
                        "BC Right Score To Score NY",
                        "Right Score To Corner");
        Right_BC.register(autonChooser);

        AutonConfig Left_BC =
                new AutonConfig(
                        "Left BC",
                        TwoCornerShallow::new,
                        "Left Corner Bite Anti Collision",
                        "Left NZ To Score Anti Collision",
                        "BC Left Score To Score NY",
                        "Left Score To Corner");
        Left_BC.register(autonChooser);

        // AutonConfig Exp_Right_Champs = new AutonConfig("Exp Right Champs", MasterAuton::new,
        // "Champs Right To Shallow", "Champs Right Shallow To Score", "Champs Right Score To
        // Corner", "Champs Right Bite Score To Score");
        // Right_Champs.register(autonChooser);

        // AutonConfig Exp_Left_Champs = new AutonConfig("Exp Left Champs", MasterAuton::new,
        // "Champs Left To Shallow", "Champs Left Shallow To Score", "Champs Left Score To Corner",
        // "Champs Left Bite Score To Score");
        // Left_Champs.register(autonChooser);

        // BC RIGHT
        // AutonConfig R_CN_FN_D = new AutonConfig("Right Corner-Near Far-Near Dot",
        // TwoCornerBC::new,
        // "BC Right Corner Bite", "BC Right NZ To Score", "Right Score To Corner", "BC Right Bite
        // Score To Score", "Right Corner To Dot");
        // R_CN_FN_D.register(autonChooser);

        // AutonConfig R_CN_NF_D = new AutonConfig("Right Corner-Near Near-Far Dot",
        // TwoCornerBC::new,
        // "BC Right Corner Bite", "BC Right NZ To Score", "Right Score To Corner", "BC Right Score
        // To Score", "Right Corner To Dot");
        // R_CN_NF_D.register(autonChooser);

        // AutonConfig R_CN_FN_CD = new AutonConfig("Right Corner-Near Far-Near Center Dot",
        // CenterTwoCornerBC::new,
        // "BC Right Corner Bite", "BC Right NZ To Score", "Right Score To Corner", "BC Right Bite
        // Score To Score", "Right Corner To Center Dot pt1", "Right Corner To Center Dot pt2");
        // R_CN_FN_CD.register(autonChooser);

        // AutonConfig R_CN_NF_CD = new AutonConfig("Right Corner-Near Near-Far Center Dot",
        // CenterTwoCornerBC::new,
        // "BC Right Corner Bite", "BC Right NZ To Score", "Right Score To Corner", "BC Right Score
        // To Score", "Right Corner To Center Dot pt1", "Right Corner To Center Dot pt2");
        // R_CN_NF_CD.register(autonChooser);

        // BC LEFT
        // AutonConfig L_CN_FN_D = new AutonConfig("Left Corner-Near Far-Near Dot",
        // TwoCornerBC::new,
        // "BC Left Corner Bite", "BC Left NZ To Score", "Left Score To Corner", "BC Left Bite Score
        // To Score", "Left Corner To Dot");
        // L_CN_FN_D.register(autonChooser);

        // AutonConfig L_CN_NF_D = new AutonConfig("Left Corner-Near Near-Far Dot",
        // TwoCornerBC::new,
        // "BC Left Corner Bite", "BC Left NZ To Score", "Left Score To Corner", "BC Left Score To
        // Score", "Left Corner To Dot");
        // L_CN_NF_D.register(autonChooser);

        // AutonConfig L_CN_FN_CD = new AutonConfig("Left Corner-Near Far-Near Center Dot",
        // CenterTwoCornerBC::new,
        // "BC Left Corner Bite", "BC Left NZ To Score", "Left Score To Corner", "BC Left Bite Score
        // To Score", "Left Corner To Center Dot pt1", "Left Corner To Center Dot pt2");
        // L_CN_FN_CD.register(autonChooser);

        // AutonConfig L_CN_NF_CD = new AutonConfig("Left Corner-Near Near-Far Center Dot",
        // CenterTwoCornerBC::new,
        // "BC Left Corner Bite", "BC Left NZ To Score", "Left Score To Corner", "BC Left Score To
        // Score", "Left Corner To Center Dot pt1", "Left Corner To Center Dot pt2");
        // L_CN_NF_CD.register(autonChooser);

        // FOLLOWS
        // AutonConfig LEFT_FOLLOW = new AutonConfig("Left Follow", LeftFollow::new,
        // "Left Follow To Bump", "Left Follow To Score", "Left Corner To Depot");
        // LEFT_FOLLOW.register(autonChooser);

        // AutonConfig RIGHT_FOLLOW = new AutonConfig("Right Follow", RightFollow::new,
        // "Right Follow To Bump", "Right Follow To Score");
        // RIGHT_FOLLOW.register(autonChooser);
        // AutonConfig EMPTY_TEST = new AutonConfig("Empty Test", EmptyTest::new,
        //     "Right Trench Score To Corner");
        // EMPTY_TEST.register(autonChooser);

        // AutonConfig PATH_FIND_TEST = new AutonConfig("Path Find Test", PathfindTest::new,
        //  "Straight One", "Straight Two");
        // PATH_FIND_TEST.registe+r(autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.get();
    }

    public void clearMemoized() {
        superstructure.clearMemoized();
        swerve.clearMemoized();
        InterpolationCalculator.clearMemoized();
    }

    public static double getWaitTimeOne() {
        return waitTimeOne.get();
    }

    public static double getWaitTimeTwo() {
        return waitTimeTwo.get();
    }
}
