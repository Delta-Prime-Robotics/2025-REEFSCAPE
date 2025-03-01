// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.UsbPort;
import frc.robot.commands.Autos;
import frc.robot.commands.ManualAlgaeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CapstanSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem;
  private final CapstanSubsystem m_CapstanSubsystem;
  private final AlgaeSubsystem m_AlgaeSubsystem;
  private final CoralSubsystem m_CoralSubsystem;

  // Utilitys
 //private final Autos m_Autos = new Autos();
  
  // The driver's controller
  private final CommandXboxController m_driverGamepad = new CommandXboxController(UsbPort.kDriveControler);
  private final CommandXboxController m_operatorGamepad = new CommandXboxController(UsbPort.kOperatorControler);
  private final CommandXboxController m_testingGampepad = new CommandXboxController(UsbPort.kTestingControler);

  private final SendableChooser<Command> m_pathChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_DriveSubsystem = new DriveSubsystem();
    m_CapstanSubsystem = new CapstanSubsystem();
    m_AlgaeSubsystem = new AlgaeSubsystem(m_CapstanSubsystem);
    m_CoralSubsystem = new CoralSubsystem(m_CapstanSubsystem);
    
    DriverStation.silenceJoystickConnectionWarning(true);


    // ! Must be called after subsystems are created 
    // ! and before building auto chooser
    configurePathPlaner();
    
    m_pathChooser = AutoBuilder.buildAutoChooser("");

    // Configure the trigger bindings
    configureDefaultCommands();
    configureBindings();
    
    SmartDashboard.putData("PathPlaner Chooser", m_pathChooser);
  }
  
  private void configureDefaultCommands() {
    m_DriveSubsystem.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> m_DriveSubsystem.drive(
              -MathUtil.applyDeadband(m_driverGamepad.getLeftY(), UsbPort.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverGamepad.getLeftX(), UsbPort.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverGamepad.getRightX(), UsbPort.kDriveDeadband),
              true),
          m_DriveSubsystem));

  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Drive Subsystem Bindings

    m_driverGamepad.back().and(m_driverGamepad.povUp())
    .onTrue(new InstantCommand(
      () ->m_DriveSubsystem.zeroHeading(),
      m_DriveSubsystem
    ));
  
    m_driverGamepad.x()
    .toggleOnTrue(new InstantCommand(
      () ->m_DriveSubsystem.setX(),
      m_DriveSubsystem
    ));


    m_operatorGamepad.x()
    .whileTrue(m_CapstanSubsystem.runAlgaeWrist(0.5));

    m_operatorGamepad.a()
    .whileTrue(m_CapstanSubsystem.runAlgaeWrist(-0.4));

    m_operatorGamepad.y()
    .whileTrue(m_CapstanSubsystem.runCoralWrist(0.3));

    m_operatorGamepad.b()
    .whileTrue(m_CapstanSubsystem.runCoralWrist(-0.3));

    m_operatorGamepad.leftBumper()
    .whileTrue(m_CapstanSubsystem.runElevator(0.5));

    m_operatorGamepad.rightBumper()
    .whileTrue(m_CapstanSubsystem.runElevator(-0.5));
  }

  private void configurePathPlaner(){
  //  NamedCommands.registerCommand(null, null);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   return m_pathChooser.getSelected();
  }
}
