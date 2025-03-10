// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.UsbPort;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CapstanSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristsSubsystem;
import frc.robot.subsystems.CapstanSubsystem.Setpoint;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  private final WristsSubsystem m_WristsSubsystem;

  // Utilitys
 //private final Autos m_Autos = new Autos();
  
  // The driver's controller
  private final CommandXboxController m_driverGamepad = new CommandXboxController(UsbPort.kDriveControler);
  private final CommandXboxController m_operatorGamepad = new CommandXboxController(UsbPort.kOperatorControler);
  private final CommandXboxController m_testingGampepad = new CommandXboxController(UsbPort.kTestingControler);

  private final SendableChooser<Command> m_pathChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      //For USB/Ethernet Teathering at Compation
      PortForwarder.add(5800, "photonvision.local", 5800);
      
      // ! Must be called after subsyste ms are created 
      m_DriveSubsystem = new DriveSubsystem();
      m_CapstanSubsystem = new CapstanSubsystem();
      m_AlgaeSubsystem = new AlgaeSubsystem(m_CapstanSubsystem);
      m_CoralSubsystem = new CoralSubsystem(m_CapstanSubsystem);
      m_WristsSubsystem = new WristsSubsystem(m_CapstanSubsystem);
      
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
      
      m_driverGamepad.rightBumper()
      .whileTrue(
        new RunCommand(
        () -> m_DriveSubsystem.drive(
            -MathUtil.applyDeadband(m_driverGamepad.getLeftY(), UsbPort.kDriveDeadband)* UsbPort.kBabyModeWeight,
            -MathUtil.applyDeadband(m_driverGamepad.getLeftX(), UsbPort.kDriveDeadband)* UsbPort.kBabyModeWeight,
            -MathUtil.applyDeadband(m_driverGamepad.getRightX(), UsbPort.kDriveDeadband)* UsbPort.kBabyModeWeight,
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
      //-------------------------
      m_driverGamepad.back()
      .onTrue(new InstantCommand(
        () -> m_DriveSubsystem.zeroHeading(),
        m_DriveSubsystem
      ));
    
      m_driverGamepad.x()
      .toggleOnTrue(new InstantCommand(
        () ->m_DriveSubsystem.setX(),
        m_DriveSubsystem
      ));
      
      //Algae Bindings
      //---------------
      //Test to see if this is working (it does seem to be working)
      m_AlgaeSubsystem.setDefaultCommand(new RunCommand(
      ()->m_AlgaeSubsystem.manualControl(() -> m_operatorGamepad.getLeftY()),
        m_AlgaeSubsystem));

      // m_operatorGamepad.leftTrigger(0.1)
      // .whileTrue(m_AlgaeSubsystem.autoIntakeAlgae());
      // m_operatorGamepad.leftTrigger(0.1)
      // .whileTrue(m_WristsSubsystem.moveAlgaeWristToSetpointCommand(Setpoint.kGround));

      // m_operatorGamepad.rightTrigger(0.1)
      // .whileTrue(m_WristsSubsystem.moveAlgaeWristToSetpointCommand(Setpoint.kStore));

      m_operatorGamepad.x()
      .whileTrue(m_WristsSubsystem.runAlgaeWrist(()-> 0.5));
  
      m_operatorGamepad.a()
      .whileTrue(m_WristsSubsystem.runAlgaeWrist(()-> -0.4));
  
  
      //Coral Bindings
      //---------------
      m_CoralSubsystem.setDefaultCommand(new RunCommand(
      ()->m_CoralSubsystem.manualControl(() -> m_operatorGamepad.getRightY()),
        m_CoralSubsystem));

      // m_operatorGamepad.povUp()
      // .onTrue(() -> m_WristsSubsystem.setSetpointCommand(Setpoint.kStore));

      // m_operatorGamepad.povDown()
      // .onTrue(m_WristsSubsystem.setSetpointCommand(Setpoint.kL2));
 
      // m_operatorGamepad.leftTrigger(0.1)
      // .whileTrue(m_WristsSubsystem.moveCoralWristToSetpointCommand(Setpoint.kStore));
     
      // m_operatorGamepad.rightTrigger(0.1)
      // .whileTrue(m_WristsSubsystem.moveAlgaeWristToSetpointCommand(Setpoint.kGround)
      // .andThen(m_WristsSubsystem.moveCoralWristToSetpointCommand(Setpoint.kL2))
      // .andThen(m_CoralSubsystem.shootToL2L3Command()));

      m_operatorGamepad.y()
      .whileTrue(m_WristsSubsystem.runCoralWrist(()->0.3));
  
      m_operatorGamepad.b()
      .whileTrue(m_WristsSubsystem.runCoralWrist(()->-0.3));
      
      //Elevator Bindings
      //------------------
      m_operatorGamepad.leftBumper()
      .whileTrue(m_CapstanSubsystem.runElevatorCommand(-0.3));
  
      m_operatorGamepad.rightBumper()
      .whileTrue(m_CapstanSubsystem.runElevatorCommand(0.35));

      m_operatorGamepad.leftTrigger(0.1)
      .whileTrue(m_CapstanSubsystem.moveToSetpointCommand(Setpoint.kGround));

      m_operatorGamepad.rightTrigger(0.1)
      .whileTrue(m_CapstanSubsystem.moveToSetpointCommand(Setpoint.kL3)
      .andThen(m_WristsSubsystem.moveCoralWristToSetpointCommand(Setpoint.kL3))
      .andThen(m_CoralSubsystem.shootToL2L3Command()));


      // m_operatorGamepad.leftTrigger(0.05)
      // .whileTrue(new RunCommand(()-> m_CapstanSubsystem.setSpeed(
      //   MathUtil.clamp(
      //       m_operatorGamepad.getLeftTriggerAxis(),
      //       0.0,
      //       0.6)),
      //   m_CapstanSubsystem));
  
      // m_operatorGamepad.rightTrigger(0.05)
      // .whileTrue(new RunCommand(()-> m_CapstanSubsystem.setSpeed(
      //   -MathUtil.clamp(
      //       m_operatorGamepad.getLeftTriggerAxis(),
      //       0.0,
      //       0.6)),
      //   m_CapstanSubsystem));

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
