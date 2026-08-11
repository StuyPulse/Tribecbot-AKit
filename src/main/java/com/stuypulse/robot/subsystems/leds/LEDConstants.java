package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDConstants {
  public SolidColor solidColorRequest =
      new SolidColor(0, LEDConstants.LED_LENGTH - 1).withColor(new RGBWColor(Color.kRed));

  public static RGBWColor rgbwConverter(Color color) {
    return new RGBWColor(color);
  }

  public final int STRIP_START = 8;               // CANdle already 
  public final int LED_LENGTH = STRIP_START + 21; // has 8 LEDs
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

  // RGBWColor REVERSE = rgbwConverter(Color.kWhite);
  RGBWColor STOP_ROLLERS = rgbwConverter(Color.kYellow);

  RGBWColor RESET_HEADING = rgbwConverter(Color.kYellow);
  RGBWColor X_WHEELS = rgbwConverter(Color.kRed);

  RGBWColor INTAKE_STOW = rgbwConverter(Color.kBrown); // broken
  RGBWColor INTAKE_DEPLOYED = rgbwConverter(Color.kPurple); // broken

  RGBWColor DISABLED_ALIGNED = rgbwConverter(Color.kGreen);
  RGBWColor DISABLED = rgbwConverter(Color.kRed);

  RGBWColor AUTON_ONE = rgbwConverter(Color.kBlue);
  RGBWColor AUTON_TWO = rgbwConverter(Color.kOrange);

  RGBWColor LLDEAD = rgbwConverter(Color.kWhite);

  SolidColor RIGHT_DEAD_STRIP =
      new SolidColor(LEDConstants.LED_LENGTH - 6, LEDConstants.LED_LENGTH - 2);
  SolidColor BACK_DEAD_STRIP =
      new SolidColor(LEDConstants.LED_LENGTH - 13, LEDConstants.LED_LENGTH - 9);
  SolidColor LEFT_DEAD_STRIP =
      new SolidColor(LEDConstants.LED_LENGTH - 20, LEDConstants.LED_LENGTH - 16);
  SolidColor CANDLE_DEAD_STRIP = new SolidColor(0, 7);

  // RGBWColor.gradient(GradientType.kDiscontinuous, Color.kRed,
  // Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));

  public final int DESIRED_TAGS_WHEN_DISABLED = 2;

  public double APRIL_TAG_DISTANCE_THRESHOLD =
      Units.feetToMeters(
          2); // TODO: update because comparing Translation2d, so make sure it is 2 feet
}
