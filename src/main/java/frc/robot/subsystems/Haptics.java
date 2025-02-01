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
 * It provides methods to set the rumble on the left, right, or both sides of the controller.
 */
public class Haptics extends SubsystemBase {
  private GenericHID m_controller;
  private RumbleType kLeft = RumbleType.kLeftRumble;
  private RumbleType kRight = RumbleType.kRightRumble;
  private RumbleType kBoth = RumbleType.kBothRumble;

 /**
   * Creates a new Haptics subsystem.
   * 
   * @param m_driverGamepad The XboxController to be used for haptic feedback.
  */
  public Haptics(XboxController m_driverGamepad) {
    m_controller =  new GenericHID(m_driverGamepad.getPort());
  }

  /** Sets the rumble on the left side of the controller.
   * @param buzz The intensity of the rumble (0.0 to 1.0).*/
  private void setLeft(double buzz) {
    m_controller.setRumble(kLeft, buzz);
  }

   /**Sets the rumble on the right side of the controller.
   * @param buzz The intensity of the rumble (0.0 to 1.0).*/
  private void setRight(double buzz) {
    m_controller.setRumble(kRight, buzz);
  }

  /** Sets the rumble on both sides of the controller.
   * @param buzz The intensity of the rumble (0.0 to 1.0).*/
  private void setBoth(double buzz) {
    m_controller.setRumble(kBoth, buzz);
  }

  /**Stops the rumble on the specified side of the controller.
   * @param where The side of the controller to stop rumbling.*/
  private void stopBuzzing(RumbleType where) {
    m_controller.setRumble(where, 0);
  }

  public Command buzzLeft(double buzz) {
    return runEnd(
      ()-> setLeft(buzz),
      ()-> stopBuzzing(kLeft));
  }

  public Command buzzRight(double buzz) {
    return runEnd(
      ()-> setRight(buzz),
      ()-> stopBuzzing(kRight));
  }

  public Command buzzBoth(double buzz) {
    return runEnd(
      ()-> setBoth(buzz),
      ()-> stopBuzzing(kBoth));
  }
  
  public Command buzzFor(double buzz, RumbleType where, double time) {
    return runEnd(
      () -> m_controller.setRumble(where, buzz),
      () -> stopBuzzing(where)
    ).withTimeout(time);
  }

  public Command coralRumble() {
    double kTime = 3;
    double kStrength = 0.8;
    return buzzFor(kStrength, kLeft, kTime)
    .andThen(Commands.waitSeconds(kTime));
  }

}
