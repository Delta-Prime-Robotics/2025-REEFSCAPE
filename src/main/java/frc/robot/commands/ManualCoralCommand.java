// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralCommand extends Command {
  private CoralSubsystem m_coral;
  private boolean m_reverse;
  private double m_speed;

  /**
   * Command to manually control the coral subsystem.
   *
   * @param speed The speed at which the coral subsystem should operate.
   * @param reversed A BooleanSupplier that determines if the direction should be reversed.
   * @param coralSubsystem The coral subsystem to be controlled.
   */
  public ManualCoralCommand(double speed, BooleanSupplier reversed, CoralSubsystem coralSubsystem) {
    m_coral = coralSubsystem;
    m_reverse = reversed.getAsBoolean();
    m_speed = speed;

    addRequirements(coralSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //the fact that this runs repeatedly might be a problem
    if (m_reverse){
      m_coral.setCoralMotor(m_speed * -1.0);
    }
    else {
      m_coral.setCoralMotor(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
