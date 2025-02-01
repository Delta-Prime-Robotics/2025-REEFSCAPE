// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CapstanSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard(DriveSubsystem m_drive, CapstanSubsystem m_capstan) {

    Shuffleboard.getTab("DriveSubsystem");
    Shuffleboard.getTab("CapstanSubsystem");

    Shuffleboard.getTab("CapstanSubsystem").addBoolean("Elev At Botttom", () -> m_capstan.isElevatorAtBottom());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
