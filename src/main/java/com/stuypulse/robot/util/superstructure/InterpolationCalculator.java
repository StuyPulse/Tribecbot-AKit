/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants.*;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.HoodAngles;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    public static AngularVelocity getInterpolatedShotRPM() {
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

    public static AngularVelocity getInterpolatedFerryRPM() {
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
            Angle targetHoodAngle, AngularVelocity targetRPM, double flightTimeSeconds) {
    }

    public record InterpolatedFerryInfo(
            Angle targetHoodAngle, AngularVelocity targetRPM, double flightTimeSeconds) {
    }

    static {
        distanceAngleInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : InterpolationConstants.DISTANCE_ANGLE_INTERPOLATION_VALUES) {
            distanceAngleInterpolator.put(pair[0], pair[1]);
        }

        distanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : InterpolationConstants.DISTANCE_RPM_INTERPOLATION_VALUES) {
            distanceRPMInterpolator.put(pair[0], pair[1]);
        }

        distanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : InterpolationConstants.DISTANCE_TOF_INTERPOLATION_VALUES) {
            distanceTOFInterpolator.put(pair[0], pair[1]);
        }

        ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : InterpolationConstants.FERRY_DISTANCE_RPM_INTERPOLATION) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }

        ferryingDistanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : InterpolationConstants.FERRY_TOF_INTERPOLATION) {
            ferryingDistanceTOFInterpolator.put(pair[0], pair[1]);
        }
    }

    public static InterpolatedShotInfo interpolateShotInfo() {
        return interpolateShotInfo(Turret.getInstance().getTurretPose(), Field.HUB_CENTER);
    }

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d turretPose, Pose2d targetPose) {
        Translation2d hubPose = targetPose.getTranslation();
        Translation2d currentPose = turretPose.getTranslation();

        double distanceMeters = currentPose.getDistance(hubPose);

        Angle targetAngle = Radians.of(distanceAngleInterpolator.get(distanceMeters));
        AngularVelocity targetRPM = RPM.of(distanceRPMInterpolator.get(distanceMeters));
        double flightTime = distanceTOFInterpolator.get(distanceMeters);

        return new InterpolatedShotInfo(targetAngle, targetRPM, flightTime);
    }

    public static InterpolatedFerryInfo interpolateFerryingInfo() {
        Pose2d turretPose = Turret.getInstance().getTurretPose();
        Pose2d ferryPose = Field.getFerryZonePose(turretPose.getTranslation());

        return interpolateFerryingInfo(turretPose, ferryPose);
    }

    public static InterpolatedFerryInfo interpolateFerryingInfo(
            Pose2d turretPose, Pose2d targetPose) {
        Translation2d currentPose = turretPose.getTranslation();
        Translation2d ferryPose = targetPose.getTranslation();

        double distanceMeters = currentPose.getDistance(ferryPose);

        Angle targetAngle = HoodAngles.FERRY_ANGLE;
        AngularVelocity targetRPM = RPM.of(ferryingDistanceRPMInterpolator.get(distanceMeters));
        double flightTime = ferryingDistanceTOFInterpolator.get(distanceMeters);

        return new InterpolatedFerryInfo(targetAngle, targetRPM, flightTime);
    }
}
