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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Capstan;
import frc.robot.Constants.CapstanConstants.AlgaeWristSetpoints;
import frc.robot.Constants.CapstanConstants.CoralWristSetpoints;
import frc.robot.Constants.CapstanConstants.ElevatorSetpoints;

import static frc.robot.Constants.CapstanConstants.*;

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

  private final SparkMax m_algaeWristMotor;
  private final SparkMax m_coralWristMotor;

  private final RelativeEncoder m_elevatorEncoder;
  private final RelativeEncoder m_algaeWristEncoder;
  private final RelativeEncoder m_coralWristEncoder;

  private final SparkClosedLoopController m_elevatorPIDController;
  private final SparkClosedLoopController m_algaeWristPIDController;
  private final SparkClosedLoopController m_coralWristPIDController;

  private final DigitalInput m_hallSensor;

  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kStore;
  private double algaeWristCurrentTarget = AlgaeWristSetpoints.kStore;
  private double coralWristCurrentTarget = CoralWristSetpoints.kStore;
  private Setpoint currentSetpoint = Setpoint.kStore;

  /** Creates a new CapstanSubsystem. */
  public CapstanSubsystem() {
    m_elevatorLeader = new SparkMax(kElevatorLeaderCanId, MotorType.kBrushless);
    m_elevatorFollower = new SparkMax(kElevatorFollowerCanId, MotorType.kBrushless);

    m_algaeWristMotor = new SparkMax(kAlgaeWristCanId, MotorType.kBrushless);
    m_coralWristMotor = new SparkMax(kCoralWristCanId, MotorType.kBrushless);

    m_elevatorEncoder = m_elevatorLeader.getEncoder();
    m_algaeWristEncoder = m_algaeWristMotor.getEncoder();
    m_coralWristEncoder = m_coralWristMotor.getEncoder();
    
    m_elevatorPIDController = m_elevatorLeader.getClosedLoopController();
    m_algaeWristPIDController = m_algaeWristMotor.getClosedLoopController();
    m_coralWristPIDController = m_coralWristMotor.getClosedLoopController();

    m_ElevatorFollowerConfig
      .apply(Capstan.elevatorConfig)
      .follow(kElevatorLeaderCanId)
      .inverted(true);
    
    m_elevatorLeader.configure(Capstan.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorFollower.configure(m_ElevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_algaeWristMotor.configure(Capstan.algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralWristMotor.configure(Capstan.coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevatorEncoder.setPosition(0);
    // m_wristEncoder.setPosition(0);

    m_hallSensor = new DigitalInput(0);

  }

  public boolean isElevatorAtBottom() {
    return m_hallSensor.get();
  }

  public Setpoint curentCapstanSetpoint() {
    return currentSetpoint;
  }

  private void moveToSetpoint() {
    m_algaeWristPIDController.setReference(algaeWristCurrentTarget, ControlType.kMAXMotionPositionControl);
    m_coralWristPIDController.setReference(coralWristCurrentTarget, ControlType.kMAXMotionPositionControl);
    m_elevatorPIDController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && isElevatorAtBottom()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to
      // "pressed" to
      // prevent constant zeroing while pressed
      m_elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!isElevatorAtBottom()) {
      wasResetByLimit = false;
    }
  }

  public Command runAlgaeWrist(double speed){
    return startEnd(
      ()-> m_algaeWristMotor.set(speed),
      ()-> m_algaeWristMotor.stopMotor());
  }

  public Command runCoralWrist(double speed){
    return startEnd(
      ()-> m_coralWristMotor.set(speed),
      ()-> m_coralWristMotor.stopMotor());
  }

  public Command runElevator(double speed) {
    return startEnd(
      ()-> m_elevatorLeader.set(speed),
      ()-> m_elevatorLeader.stopMotor());
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
              algaeWristCurrentTarget = AlgaeWristSetpoints.kStore;
              coralWristCurrentTarget = CoralWristSetpoints.kStore;
              elevatorCurrentTarget = ElevatorSetpoints.kStore;
              currentSetpoint = Setpoint.kStore;
            case kProcessor:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
            case kFeederStation:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kFeederStation;
              coralWristCurrentTarget = CoralWristSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              currentSetpoint = Setpoint.kFeederStation;
              break;
            case kNet:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
              break;
            case kL1:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL1;
              coralWristCurrentTarget = CoralWristSetpoints.kL1;
              elevatorCurrentTarget = ElevatorSetpoints.kL1;
              currentSetpoint = Setpoint.kL1;
              break;
            case kL2:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL2;
              coralWristCurrentTarget = CoralWristSetpoints.kL2;
              elevatorCurrentTarget = ElevatorSetpoints.kL2;
              currentSetpoint = Setpoint.kL2;
              break;
            case kL3:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL3;
              coralWristCurrentTarget = CoralWristSetpoints.kL3;
              elevatorCurrentTarget = ElevatorSetpoints.kL3;
              currentSetpoint = Setpoint.kL3;
              break;
            case kL4:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL4;
              coralWristCurrentTarget = CoralWristSetpoints.kL4;
              elevatorCurrentTarget = ElevatorSetpoints.kL4;
              currentSetpoint = Setpoint.kL4;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
  }
}
