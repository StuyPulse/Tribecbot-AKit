package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.leds.LEDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDDefaultCommand extends InstantCommand {

  private final LEDController leds;
  // Boolean Fields
  public static boolean isLeftLLDead;
  public static boolean isBackLLDead;
  public static boolean isRightLLDead;

  public boolean leftDeadAnimationCleared;
  public boolean backDeadAnimationCleared;
  public boolean rightDeadAnimationCleared;

  public LEDDefaultCommand() {
    this.leds = LEDController.getInstance();
    // might need to change idk
    isLeftLLDead = false;
    isBackLLDead = false;
    isRightLLDead = false;

    leftDeadAnimationCleared = false;
    backDeadAnimationCleared = false;
    rightDeadAnimationCleared = false;

    addRequirements(leds);
  }
}
