// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.UsbPort;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Haptics;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  // Utilitys
 //private final Autos m_Autos = new Autos();
  
  // The driver's controller
  private final CommandXboxController  m_driverGamepad = new CommandXboxController(UsbPort.kDriveControler);
  private final Haptics m_driverHaptics = new Haptics(m_driverGamepad.getHID());

  private static boolean override_bool = false;
  private final SendableChooser<Command> m_pathChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // ! Must be called after subsystems are created 
    // ! and before building auto chooser
    configurePathPlaner();
    
    m_pathChooser = AutoBuilder.buildAutoChooser("");

    // Configure the trigger bindings
    configureDefaultCommands();
    configureBindings();
    SmartDashboard.putBoolean("Coral", false);
    SmartDashboard.putBoolean("Algae", false);
    SmartDashboard.putBoolean("Override", override_bool);
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
    Trigger TCoral = new Trigger(()-> SmartDashboard.getBoolean("Coral", false));
    Trigger TAlgae = new Trigger(()-> SmartDashboard.getBoolean("Algae", false));

    m_driverGamepad.back()
    .onTrue(new InstantCommand(
      () -> m_DriveSubsystem.zeroHeading(),
      m_DriveSubsystem
    ));
    
    m_driverGamepad.x()
    .toggleOnTrue(new InstantCommand(
      () -> m_DriveSubsystem.setX(),
      m_DriveSubsystem
    ));

    m_driverGamepad.leftTrigger()
    .onTrue(m_driverHaptics.coralBuzz());

    m_driverGamepad.rightTrigger()
    .onTrue(m_driverHaptics.algaeBuzz());

    m_driverGamepad.b()
    .onTrue(m_driverHaptics.onOffVarBuzzRepeat(0.4, 0.8, 0.25, 1));

    m_driverGamepad.a()
    .onTrue(m_driverHaptics.alternatingBuzzRepeat(0.9, 1, 0.5, 2.5));

    m_driverGamepad.x()
    .onTrue(m_driverHaptics.onOffBuzzRepeat(0.9, RumbleType.kLeftRumble, 0.15, 2));

    // m_driverGamepad.b()
    // .whileTrue(m_driverHaptics.babyModeBuzz());

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
