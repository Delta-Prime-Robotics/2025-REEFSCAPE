// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * The Haptics subsystem controls the rumble functionality of a game controller.
 * It provides methods to set the rumble on the left, right, or both sides of
 * the controller.
 */
public class Haptics extends SubsystemBase {
  private GenericHID m_controller;
  private RumbleType kHeavy = RumbleType.kLeftRumble;
  private RumbleType kMedium = RumbleType.kBothRumble;
  private RumbleType kLight = RumbleType.kRightRumble;

  /**
   * Creates a new Haptics subsystem.
   * 
   * @param m_gamepad The XboxController to be used for haptic feedback.
   */
  public Haptics(XboxController m_gamepad) {
    m_controller = new GenericHID(m_gamepad.getPort());
  }

  /**
   * Sets the rumble on the controller.
   * 
   * @param buzz The intensity of the rumble (0.0 to 1.0).
   * @param type The type of buzz, Heavy, Medium, or Light
   */
  private void setBuzz(double buzz, RumbleType type) {
    m_controller.setRumble(type, buzz);
  }

  /**
   * Stops the rumble on the specified side of the controller.
   * 
   * @param where The side of the controller to stop rumbling.
   */
  private void stopBuzzing(RumbleType where) {
    m_controller.setRumble(where, 0);
  }

  public Command buzzLeft(double buzz) {
    return runEnd(
        () -> setBuzz(buzz, kHeavy),
        () -> stopBuzzing(kHeavy));
  }

  public Command buzzRight(double buzz) {
    return runEnd(
        () -> setBuzz(buzz, kLight),
        () -> stopBuzzing(kLight));
  }

  public Command buzzBoth(double buzz) {
    return startEnd(
        () -> setBuzz(buzz, kMedium),
        () -> stopBuzzing(kMedium));
  }

  /*This isn't working yet,  */
  public Command buzzVarBoth(double lightBuzz, double heavyBuzz) {
    return runOnce(()-> setBuzz(heavyBuzz, kHeavy))
      .alongWith(runOnce(()-> setBuzz(lightBuzz, kLight)))
      .finallyDo(()-> stopBuzzing(kMedium));
  }

  public Command buzzFor(double buzz, RumbleType type, double time) {
    return runEnd(
        () -> m_controller.setRumble(type, buzz),
        () -> stopBuzzing(type)).withTimeout(time);
  }

  public Command onOffBuzzRepeat(double buzz, RumbleType type, double buzzTime, double totalTime) {
    double kLess = buzzTime / 3;
    return new RepeatCommand(
        buzzFor(buzz, type, buzzTime))
        .andThen(Commands.waitSeconds(buzzTime - kLess))
        .withTimeout(totalTime)
        .finallyDo(() -> stopBuzzing(type));
  }

  public Command coralBuzz() {
    double kTime = 0.15;
    int kTotalTime = 1;
    double kStrength = 0.7;
    return onOffBuzzRepeat(kStrength, kLight, kTime, kTotalTime);
  }

  public Command algaeBuzz() {
    double kTime = 0.15;
    int kTotalTime = 1;
    double kStrength = 0.95;
    return onOffBuzzRepeat(kStrength, kHeavy, kTime, kTotalTime);
  }

  public Command babyModeBuzz() {
    return buzzFor(1, kMedium, 0.5).andThen(buzzVarBoth(0.75, 0.35));
  }
}
