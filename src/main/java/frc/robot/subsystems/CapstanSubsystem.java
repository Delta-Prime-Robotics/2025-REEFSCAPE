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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs.AlgaeConfig;
import frc.robot.Configs.Capstan;
import frc.robot.Constants.CapstanConstants.AlgaeWristSetpoints;
import frc.robot.Constants.CapstanConstants.CoralWristSetpoints;
import frc.robot.Constants.CapstanConstants.ElevatorSetpoints;

import static frc.robot.Constants.CapstanConstants.*;

import java.util.function.BooleanSupplier;

public class CapstanSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kStore,
    kFeederStation,
    kProcessor,
    kNet,
    kL1,
    kL2,
    kL3,
    kL4;
  }

  private final SparkMax m_elevatorLeader;
  private final SparkMax m_elevatorFollower;
  private final SparkMaxConfig m_ElevatorFollowerConfig = new SparkMaxConfig();

  private final RelativeEncoder m_elevatorEncoder;

  private final SparkClosedLoopController m_elevatorPIDController;

  private final DigitalInput m_hallSensor;

  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kStore;

  private Setpoint currentSetpoint = Setpoint.kStore;

  /** Creates a new CapstanSubsystem. */
  public CapstanSubsystem() {
    m_elevatorLeader = new SparkMax(kElevatorLeaderCanId, MotorType.kBrushless);
    m_elevatorFollower = new SparkMax(kElevatorFollowerCanId, MotorType.kBrushless);

    m_elevatorEncoder = m_elevatorLeader.getEncoder();
    
    m_elevatorPIDController = m_elevatorLeader.getClosedLoopController();

    m_ElevatorFollowerConfig
      .apply(Capstan.elevatorConfig)
      .follow(kElevatorLeaderCanId, true);
      
    // m_elevatorLeader.configure(Capstan.elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_elevatorFollower.configure(m_ElevatorFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_elevatorLeader.configure(Capstan.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorFollower.configure(m_ElevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_elevatorEncoder.setPosition(0);
    // m_wristEncoder.setPosition(0);

    m_hallSensor = new DigitalInput(0);
  }

  public BooleanSupplier isElevatorAtBottom() {
    return ()-> m_hallSensor.get();
  }

  public Setpoint getCurentElevatorSetpoint() {
    return currentSetpoint;
  }
  
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

  private void moveToSetpoint() {
    m_elevatorPIDController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
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
    // if(getElevatorPostion() <= kUpperLimit) {
      m_elevatorLeader.set(speed);
    // }
    // else {
    //   stopMotors();
    // }
  }

  private void stopMotors() {
    m_elevatorLeader.stopMotor();
  }
  
  public Command runElevator(double speed) {
    return runEnd(() -> setSpeed(speed), () -> stopMotors());
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to
   * their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
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
            case kL2:
              elevatorCurrentTarget = ElevatorSetpoints.kL2;
              currentSetpoint = Setpoint.kL2;
              break;
            case kL3:
              elevatorCurrentTarget = ElevatorSetpoints.kL3;
              currentSetpoint = Setpoint.kL3;
              break;
            case kL4:
              elevatorCurrentTarget = ElevatorSetpoints.kL4;
              currentSetpoint = Setpoint.kL4;
              break;
          }
        }).withName(setpoint.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    SmartDashboard.putNumber("Elevator Position", getElevatorPostion());
    SmartDashboard.putNumber("Elevator Velocity", getElevatorVelocity());
  }
}
