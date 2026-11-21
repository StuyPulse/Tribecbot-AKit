/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDConstants {
    public interface LEDSettings {
        int STRIP_START = 8; // CANdle already
        int LED_LENGTH = STRIP_START + 21; // has 8 LEDs

        SolidColor solidColorRequest =
                new SolidColor(0, LED_LENGTH - 1).withColor(new RGBWColor(Color.kRed));

        public static RGBWColor rgbwConverter(Color color) {
            return new RGBWColor(color);
        }

        public interface StateColors {
            RGBWColor PASSING_TRENCH = rgbwConverter(Color.kRed);
            RGBWColor IS_BEHIND_HUB = rgbwConverter(Color.kRed);

            // RGBWColor CLIMB_ALIGNING = rgbwConverter(Color.kYellow);
            // RGBWColor CLIMB_ALIGNED = rgbwConverter(Color.kGreen);
            // RGBWColor CLIMBING = rgbwConverter(Color.kRed);

            RGBWColor TURRET_WRAPPING = rgbwConverter(Color.kRed);
            // RGBWColor LEFT_WARNING = rgbwConverter(Color.kBlack); // TBD
            // RGBWColor RIGHT_WARNING = rgbwConverter(Color.kBlack); // TBD

            RGBWColor SHOOT_IN_PLACE = rgbwConverter(Color.kPurple);

            RGBWColor SOTM_ON = rgbwConverter(Color.kGreen);
            RGBWColor FOTM_ON = rgbwConverter(Color.kDarkBlue);
            RGBWColor LEFT_CORNER = rgbwConverter(Color.kPurple);
            RGBWColor RIGHT_CORNER = rgbwConverter(Color.kBlue);

            RGBWColor KB_DISTANCE = rgbwConverter(Color.kPink);

            RGBWColor STOP_ROLLERS = rgbwConverter(Color.kYellow);
            RGBWColor ROLLERS_REVERSE = rgbwConverter(Color.kYellowGreen);

            RGBWColor RESET_HEADING = rgbwConverter(Color.kYellow);
            RGBWColor X_WHEELS = rgbwConverter(Color.kRed);

            RGBWColor INTAKE_STOW = rgbwConverter(Color.kBrown); // broken
            RGBWColor INTAKE_DEPLOYED = rgbwConverter(Color.kPurple); // broken

            RGBWColor DISABLED_ALIGNED = rgbwConverter(Color.kGreen);
            RGBWColor DISABLED = rgbwConverter(Color.kRed);

            RGBWColor AUTON_ONE = rgbwConverter(Color.kBlue);
            RGBWColor AUTON_TWO = rgbwConverter(Color.kOrange);

            RGBWColor LLDEAD = rgbwConverter(Color.kWhite);

            // SolidColor RIGHT_DEAD_STRIP =
            // new SolidColor(LEDConstants.LED_LENGTH - 6, LEDConstants.LED_LENGTH - 2);
            // SolidColor BACK_DEAD_STRIP =
            // new SolidColor(LEDConstants.LED_LENGTH - 13, LEDConstants.LED_LENGTH - 9);
            // SolidColor LEFT_DEAD_STRIP =
            // new SolidColor(LEDConstants.LED_LENGTH - 20, LEDConstants.LED_LENGTH - 16);
            // SolidColor CANDLE_DEAD_STRIP = new SolidColor(0, 7);

            // RGBWColor.gradient(GradientType.kDiscontinuous, Color.kRed,
            // Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        }

        int RIGHT_DEAD_START = LED_LENGTH - 6;
        int RIGHT_DEAD_END = LED_LENGTH - 2;

        int BACK_DEAD_START = LED_LENGTH - 13;
        int BACK_DEAD_END = LED_LENGTH - 9;

        int LEFT_DEAD_START = LED_LENGTH - 20;
        int LEFT_DEAD_END = LED_LENGTH - 16;

        int CANDLE_START = 0;
        int CANDLE_END = 7;

        int DESIRED_TAGS_WHEN_DISABLED = 2;

        double APRIL_TAG_DISTANCE_THRESHOLD =
                Units.feetToMeters(
                        2); // TODO: update because comparing Translation2d, so make sure it is 2
        // feet
    }

    public interface LEDDeviceIds {
        int LED_PORT = 1;
        int CANDLE_PORT = 61;
    }
}
