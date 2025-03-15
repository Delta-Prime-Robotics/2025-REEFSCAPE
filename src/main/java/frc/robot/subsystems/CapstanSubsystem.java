// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs.AlgaeConfig;
import frc.robot.Configs.Capstan;

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CapstanSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kStore,
    kGround,
    kFeederStation,
    kProcessor,
    kNet,
    kL1,
    kCoralL2,
    kAlgaeL2,
    kCoralL3,
    kAlgaeL3,
    kL4;
  }

  private final SparkMax m_elevatorLeader;
  private final SparkMax m_elevatorFollower;
  private final SparkMaxConfig m_ElevatorFollowerConfig = new SparkMaxConfig();

  private final RelativeEncoder m_elevatorEncoder;

  private final PIDController m_elevatorPIDController;

  private final DigitalInput m_hallSensor;

  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kStore; //Rotations
  private Setpoint currentSetpoint = Setpoint.kStore;

  private ElevatorFeedforward m_Feedforward; 

  /** Creates a new CapstanSubsystem. */
  public CapstanSubsystem() {
    m_elevatorLeader = new SparkMax(kElevatorLeaderCanId, MotorType.kBrushless);
    m_elevatorFollower = new SparkMax(kElevatorFollowerCanId, MotorType.kBrushless);

    m_elevatorEncoder = m_elevatorLeader.getEncoder();
    
    m_elevatorPIDController = new PIDController(0.14, 0.1, 0); //tune please
    m_elevatorPIDController.setTolerance(1.5);
    m_elevatorPIDController.setIZone(0.1);

    m_Feedforward = new ElevatorFeedforward(0,0.16,3.07, 0.02); //Need to guesstamate kS

    m_ElevatorFollowerConfig
      .apply(Capstan.elevatorConfig)
      .follow(kElevatorLeaderCanId, true);
      
    // m_elevatorLeader.configure(Capstan.elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_elevatorFollower.configure(m_ElevatorFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_elevatorLeader.configure(Capstan.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorFollower.configure(m_ElevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_elevatorEncoder.setPosition(0);
    // m_wristEncoder.setPosition(0);

    m_hallSensor = new DigitalInput(9);
  }

  public BooleanSupplier isElevatorAtBottom() {
    return ()-> !m_hallSensor.get();
  }

  public Setpoint getCurentElevatorSetpoint() {
    return currentSetpoint;
  }
  
  /**
   * @return elevtator position in rotations
   */
  private double getElevatorPostion() {
    return -m_elevatorEncoder.getPosition();
  }
  
  private double getElevatorVelocity() {
    return m_elevatorEncoder.getVelocity();
  }

   /**
   * @param setpoint
   * @return if the elevator is currently at inputed setpoint
   */
  public Trigger atElevatorSetpoint(Setpoint setpoint) {
    return new Trigger(() -> setpoint == getCurentElevatorSetpoint());
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && isElevatorAtBottom().getAsBoolean()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to
      // "pressed" to
      // prevent constant zeroing while pressed
      m_elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!isElevatorAtBottom().getAsBoolean()) {
      wasResetByLimit = false;
    }
  }

  private void zeroElevator() {
    m_elevatorEncoder.setPosition(0);
  }
  
  private void setSpeed(double speed) {
    // Upper limit
  //  if (speed <= -0.2) {speed = -0.2;}
    m_elevatorLeader.set(-speed);
  }

  private void setSpeedWithLimit(double speed){
    // Upper limit
    if (speed <= -0.2) {speed = -0.2;}
    m_elevatorLeader.set(-speed);
  }

  private void stopMotors() {
    m_elevatorLeader.stopMotor();
  }
  
  public Command runElevatorCommand(double speed) {
    return runEnd(() -> setSpeed(speed), () -> stopMotors());
  }


  /**
   * Command to move the elevator to the current set setpoint
   * Should make it stay in place.
   */
  public Command moveToSetpointCommand(Setpoint setSetpoint){
    return startRun(() -> setSetpoint(setSetpoint),
      () -> {
        setSpeed(m_elevatorPIDController.calculate(getElevatorPostion()));
      })
      .until(()-> m_elevatorPIDController.atSetpoint())
      .finallyDo(()-> stopMotors());
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to
   * their predefined positions for the given setpoint.
   */
  public void setSetpoint(Setpoint setpoint) {
          switch (setpoint) {
            case kStore:
              elevatorCurrentTarget = ElevatorSetpoints.kStore;
              currentSetpoint = Setpoint.kStore;
            case kProcessor:
              elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              currentSetpoint = Setpoint.kFeederStation;
              break;
            case kNet:
              elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
              break;
            case kL1:
              elevatorCurrentTarget = ElevatorSetpoints.kL1;
              currentSetpoint = Setpoint.kL1;
              break;
            case kCoralL2:
              elevatorCurrentTarget = ElevatorSetpoints.kCoralL2;
              currentSetpoint = Setpoint.kCoralL2;
              break;
            case kAlgaeL2:
              elevatorCurrentTarget = ElevatorSetpoints.kAlgaeL2;
              currentSetpoint = Setpoint.kAlgaeL2;
              break;
            case kCoralL3:
              elevatorCurrentTarget = ElevatorSetpoints.kCoralL3;
              currentSetpoint = Setpoint.kCoralL3;
              break;
            case kAlgaeL3:
              elevatorCurrentTarget = ElevatorSetpoints.kAlgaeL3;
              currentSetpoint = Setpoint.kAlgaeL3;
              break;
            case kL4:
              elevatorCurrentTarget = ElevatorSetpoints.kL4;
              currentSetpoint = Setpoint.kL4;
              break;
            case kGround:
              elevatorCurrentTarget = ElevatorSetpoints.kGround;
              currentSetpoint = Setpoint.kGround;
              break;
            default:
              break;
          }
    m_elevatorPIDController.setSetpoint(elevatorCurrentTarget);
 }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    zeroElevatorOnLimitSwitch();
    SmartDashboard.putBoolean("Elevator at bottom", isElevatorAtBottom().getAsBoolean());
    SmartDashboard.putNumber("Elevator Position", getElevatorPostion());
    SmartDashboard.putNumber("Elevator Goal", m_elevatorPIDController.getSetpoint());
  }
}
