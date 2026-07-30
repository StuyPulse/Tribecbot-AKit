package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Field {
  public static final Field2d FIELD2D = new Field2d();

  public static Distance WIDTH = Inches.of(317.00);
  public static Distance LENGTH = Inches.of(651.200);

  public static final Pose2d HUB_CENTER =
      new Pose2d(Inches.of(182.11), WIDTH.div(2), new Rotation2d());

  public static final Pose2d INNER_LEFT_FERRY_ZONE =
      new Pose2d(Inches.of(31.5), WIDTH.minus(Inches.of(82.5)), new Rotation2d());

  public static final Pose2d INNER_RIGHT_FERRY_ZONE =
      new Pose2d(Inches.of(20.75), Inches.of(76).plus(Inches.of(48)), new Rotation2d());

  public static final Pose2d OUTER_LEFT_FERRY_ZONE =
      new Pose2d(Inches.of(31.5), WIDTH.minus(Inches.of(34.5)), new Rotation2d());

  public static final Pose2d OUTER_RIGHT_FERRY_ZONE =
      new Pose2d(Inches.of(20.75), Inches.of(76), new Rotation2d());

  public static final Distance FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE = Inches.of(75);

  public static Pose2d getFerryZonePose(Translation2d robot) {
    Distance fieldMidY = WIDTH.div(2);

    if (robot.getMeasureY().gt(fieldMidY)) {
      if (robot.getMeasureY().gt(WIDTH.minus(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE))) {
        return INNER_LEFT_FERRY_ZONE;
      } else {
        return OUTER_LEFT_FERRY_ZONE;
      }
    } else {
      if (robot.getMeasureY().lt(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE)) {
        return INNER_RIGHT_FERRY_ZONE;
      } else {
        return OUTER_RIGHT_FERRY_ZONE;
      }
    }
  }

  public static Pose2d transformToOppositeAlliance(Pose2d pose) {
    Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
    return new Pose2d(
        rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)), rotated.getRotation());
  }

  public static final double[] ALL_TAGS = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32
  };
}
