package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Field {
    public static final Field2d FIELD2D = new Field2d();

    public static Distance WIDTH = Inches.of(317.00);
    public static Distance LENGTH = Inches.of(651.200);
}
