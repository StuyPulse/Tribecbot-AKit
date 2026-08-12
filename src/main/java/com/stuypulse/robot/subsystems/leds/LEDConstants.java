package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDConstants {
  public interface LEDSettings {
    final int STRIP_START = 8; // CANdle already
    final int LED_LENGTH = STRIP_START + 21; // has 8 LEDs

    final SolidColor solidColorRequest =
        new SolidColor(0, LED_LENGTH - 1).withColor(new RGBWColor(Color.kRed));

    public static RGBWColor rgbwConverter(Color color) {
      return new RGBWColor(color);
    }

    public interface StateColors {
      final RGBWColor PASSING_TRENCH = rgbwConverter(Color.kRed);
      final RGBWColor IS_BEHIND_HUB = rgbwConverter(Color.kRed);

      // RGBWColor CLIMB_ALIGNING = rgbwConverter(Color.kYellow);
      // RGBWColor CLIMB_ALIGNED = rgbwConverter(Color.kGreen);
      // RGBWColor CLIMBING = rgbwConverter(Color.kRed);

      final RGBWColor TURRET_WRAPPING = rgbwConverter(Color.kRed);
      // RGBWColor LEFT_WARNING = rgbwConverter(Color.kBlack); // TBD
      // RGBWColor RIGHT_WARNING = rgbwConverter(Color.kBlack); // TBD

      final RGBWColor SHOOT_IN_PLACE = rgbwConverter(Color.kPurple);

      final RGBWColor SOTM_ON = rgbwConverter(Color.kGreen);
      final RGBWColor FOTM_ON = rgbwConverter(Color.kDarkBlue);
      final RGBWColor LEFT_CORNER = rgbwConverter(Color.kPurple);
      final RGBWColor RIGHT_CORNER = rgbwConverter(Color.kBlue);

      final RGBWColor KB_DISTANCE = rgbwConverter(Color.kPink);

      // RGBWColor REVERSE = rgbwConverter(Color.kWhite);
      final RGBWColor STOP_ROLLERS = rgbwConverter(Color.kYellow);

      final RGBWColor RESET_HEADING = rgbwConverter(Color.kYellow);
      final RGBWColor X_WHEELS = rgbwConverter(Color.kRed);

      final RGBWColor INTAKE_STOW = rgbwConverter(Color.kBrown); // broken
      final RGBWColor INTAKE_DEPLOYED = rgbwConverter(Color.kPurple); // broken

      final RGBWColor DISABLED_ALIGNED = rgbwConverter(Color.kGreen);
      final RGBWColor DISABLED = rgbwConverter(Color.kRed);

      final RGBWColor AUTON_ONE = rgbwConverter(Color.kBlue);
      final RGBWColor AUTON_TWO = rgbwConverter(Color.kOrange);

      final RGBWColor LLDEAD = rgbwConverter(Color.kWhite);

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

    final int RIGHT_DEAD_START = LED_LENGTH - 6;
    final int RIGHT_DEAD_END = LED_LENGTH - 2;

    final int BACK_DEAD_START = LED_LENGTH - 13;
    final int BACK_DEAD_END = LED_LENGTH - 9;

    final int LEFT_DEAD_START = LED_LENGTH - 20;
    final int LEFT_DEAD_END = LED_LENGTH - 16;

    final int CANDLE_START = 0;
    final int CANDLE_END = 7;

    final int DESIRED_TAGS_WHEN_DISABLED = 2;

    final double APRIL_TAG_DISTANCE_THRESHOLD =
        Units.feetToMeters(
            2); // TODO: update because comparing Translation2d, so make sure it is 2 feet
  }

  public interface LEDDeviceIds {
    final int LED_PORT = 1;
    final int CANDLE_PORT = 61;
  }
}
