/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;

import static edu.wpi.first.units.Units.Radians;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Superstructure.AngleInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.FerryRPMInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.FerryTOFInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.RPMInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.TOFInterpolation;
import com.stuypulse.robot.subsystems.swerve.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import java.util.Optional;

public class InterpolationCalculator {

  public static InterpolatingDoubleTreeMap distanceAngleInterpolator;
  public static InterpolatingDoubleTreeMap distanceRPMInterpolator;
  public static InterpolatingDoubleTreeMap distanceTOFInterpolator;

  public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;
  public static InterpolatingDoubleTreeMap ferryingDistanceTOFInterpolator;

  private static Optional<InterpolatedShotInfo> cachedInterpolatedShotInfo = Optional.empty();
  private static Optional<InterpolatedFerryInfo> cachedInterpolatedFerryInfo = Optional.empty();

  public static void clearMemoized() {
    cachedInterpolatedShotInfo = Optional.empty();
    cachedInterpolatedFerryInfo = Optional.empty();
  }

  public static double getInterpolatedShotRPM() {
    if (cachedInterpolatedShotInfo.isEmpty()) {
      cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo());
    }
    return cachedInterpolatedShotInfo.get().targetRPM();
  }

  public static Angle getInterpolatedShotAngle() {
    if (cachedInterpolatedShotInfo.isEmpty()) {
      cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo());
    }
    return cachedInterpolatedShotInfo.get().targetHoodAngle();
  }

  public static double getInterpolatedFerryRPM() {
    if (cachedInterpolatedFerryInfo.isEmpty()) {
      cachedInterpolatedFerryInfo = Optional.of(interpolateFerryingInfo());
    }
    return cachedInterpolatedFerryInfo.get().targetRPM();
  }

  public static Angle getInterpolatedFerryAngle() {
    if (cachedInterpolatedFerryInfo.isEmpty()) {
      cachedInterpolatedFerryInfo = Optional.of(interpolateFerryingInfo());
    }
    return cachedInterpolatedFerryInfo.get().targetHoodAngle();
  }

  public record InterpolatedShotInfo(
      Angle targetHoodAngle, double targetRPM, double flightTimeSeconds) {}

  public record InterpolatedFerryInfo(
      Angle targetHoodAngle, double targetRPM, double flightTimeSeconds) {}

  static {
    distanceAngleInterpolator = new InterpolatingDoubleTreeMap();
    for (double[] pair : AngleInterpolation.distanceAngleInterpolationValues) {
      distanceAngleInterpolator.put(pair[0], pair[1]);
    }

    distanceRPMInterpolator = new InterpolatingDoubleTreeMap();
    for (double[] pair : RPMInterpolation.distanceRPMInterpolationValues) {
      distanceRPMInterpolator.put(pair[0], pair[1]);
    }

    distanceTOFInterpolator = new InterpolatingDoubleTreeMap();
    for (double[] pair : TOFInterpolation.distanceTOFInterpolationValues) {
      distanceTOFInterpolator.put(pair[0], pair[1]);
    }

    ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
    for (double[] pair : FerryRPMInterpolation.ferryDistanceRPMInterpolation) {
      ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
    }

    ferryingDistanceTOFInterpolator = new InterpolatingDoubleTreeMap();
    for (double[] pair : FerryTOFInterpolation.FerryTOFInterpolationInterpolation) {
      ferryingDistanceTOFInterpolator.put(pair[0], pair[1]);
    }
  }

  public static InterpolatedShotInfo interpolateShotInfo() {
    Drive drive = Drive.getInstance();

    return interpolateShotInfo(drive.getTurretPose(), Field.HUB_CENTER);
  }

  public static InterpolatedShotInfo interpolateShotInfo(Pose2d turretPose, Pose2d targetPose) {
    Translation2d hubPose = targetPose.getTranslation();
    Translation2d currentPose = turretPose.getTranslation();

    double distanceMeters = currentPose.getDistance(hubPose);

    Angle targetAngle = Radians.of(distanceAngleInterpolator.get(distanceMeters));
    double targetRPM = distanceRPMInterpolator.get(distanceMeters);
    double flightTime = distanceTOFInterpolator.get(distanceMeters);

    return new InterpolatedShotInfo(targetAngle, targetRPM, flightTime);
  }

  public static InterpolatedFerryInfo interpolateFerryingInfo() {
    Drive drive = Drive.getInstance();

    Pose2d turretPose = drive.getTurretPose();
    Pose2d ferryPose = Field.getFerryZonePose(turretPose.getTranslation());

    return interpolateFerryingInfo(turretPose, ferryPose);
  }

  public static InterpolatedFerryInfo interpolateFerryingInfo(
      Pose2d turretPose, Pose2d targetPose) {
    Translation2d currentPose = turretPose.getTranslation();
    Translation2d ferryPose = targetPose.getTranslation();

    double distanceMeters = currentPose.getDistance(ferryPose);

    Angle targetAngle = Settings.Superstructure.Hood.Angles.FERRY_ANGLE;
    double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);
    double flightTime = ferryingDistanceTOFInterpolator.get(distanceMeters);

    return new InterpolatedFerryInfo(targetAngle, targetRPM, flightTime);
  }
}
