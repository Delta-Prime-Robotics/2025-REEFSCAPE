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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualAlgaeCommand extends Command {
  private AlgaeSubsystem m_algae;
  private boolean m_reverse;
  private boolean m_state;
  private double m_speed;

  /** Creates a new ManualAlgaeCommand. 
   * @param AlgaeSubsystem */
  public ManualAlgaeCommand(double speed, Trigger reversed, AlgaeSubsystem algaeSubsystem) {
    m_algae = algaeSubsystem;
    m_reverse = reversed.getAsBoolean();
    m_speed = speed;

    addRequirements(algaeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //the fact that this runs repeatedly might be a problem
    //it is going to be a problem, please put into the actual subsystem and set as default command when overided
    if (m_state != m_reverse) {
      if (m_reverse){
        m_algae.setMotors(m_speed * -1.0);
      }
      else {
        m_algae.setMotors(m_speed);
      }
      m_state = m_reverse;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
