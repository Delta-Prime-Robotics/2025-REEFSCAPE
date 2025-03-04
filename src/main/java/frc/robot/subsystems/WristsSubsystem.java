// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CapstanConstants.kAlgaeWristCanId;
import static frc.robot.Constants.CapstanConstants.kCoralWristCanId;
import static frc.robot.Constants.CapstanConstants.kElevatorFollowerCanId;
import static frc.robot.Constants.CapstanConstants.kElevatorLeaderCanId;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Capstan;
import frc.robot.Constants.CapstanConstants.AlgaeWristSetpoints;
import frc.robot.Constants.CapstanConstants.CoralWristSetpoints;
import frc.robot.Constants.CapstanConstants.ElevatorSetpoints;
import frc.robot.subsystems.CapstanSubsystem.Setpoint;

public class WristsSubsystem extends SubsystemBase {

  private final SparkMax m_algaeWristMotor;
  private final SparkMax m_coralWristMotor;

  private final RelativeEncoder m_algaeWristEncoder;
  private final RelativeEncoder m_coralWristEncoder;

  private final SparkClosedLoopController m_algaeWristPIDController;
  private final SparkClosedLoopController m_coralWristPIDController;


  private double algaeWristCurrentTarget = AlgaeWristSetpoints.kStore;
  private double coralWristCurrentTarget = CoralWristSetpoints.kStore;
  private Setpoint currentSetpoint = Setpoint.kStore;

  private final CapstanSubsystem m_Capstan;
  private Setpoint m_CurrentSetpoint;

  /** Creates a new WristsSubsystem. */
  public WristsSubsystem(CapstanSubsystem capstan) {
    this.m_Capstan = capstan;
    m_CurrentSetpoint = m_Capstan.getCurentElevatorSetpoint();
    
    m_algaeWristMotor = new SparkMax(kAlgaeWristCanId, MotorType.kBrushless);
    m_coralWristMotor = new SparkMax(kCoralWristCanId, MotorType.kBrushless);

    m_algaeWristEncoder = m_algaeWristMotor.getEncoder();
    m_coralWristEncoder = m_coralWristMotor.getEncoder();
    
    m_algaeWristPIDController = m_algaeWristMotor.getClosedLoopController();
    m_coralWristPIDController = m_coralWristMotor.getClosedLoopController();
      
    // m_algaeWristMotor.configure(Capstan.algaeWristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_coralWristMotor.configure(Capstan.coralWristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_algaeWristMotor.configure(Capstan.algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralWristMotor.configure(Capstan.coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(runOnce(()-> stopBothMotors()));
  }

  private double getAlgaeEncoderPosition() {
    return m_algaeWristEncoder.getPosition();
  }

  // private void setBothWristSetpoints(Double algaeSetpoint, Double coralSetpoint) {
  //   m_algaeWristPIDController.setReference(
  //     algaeSetpoint,
  //     ControlType.kMAXMotionPositionControl,
  //     ClosedLoopSlot.kSlot0,
  //     new ArmFeedforward(0,0,0).calculateWithVelocities(
  //       Units.rotationsToRadians(getAlgaeEncoderPosition()),
  //       0,
  //       0
  //     ),
  //     ArbFFUnits.kVoltage);
  // }

  private double getCoralEncoderPosition() {
    return m_coralWristEncoder.getPosition();
  }

  private void setAlgaeWristMotor(Double speed) {
    // if (getAlgaeEncoderPosition() >= 0.1) {
      m_algaeWristMotor.set(speed);
    // }
    // else{
    //   stopAlgaeWristMotor();
    // }
  }

  private void setCoralWristMotor(Double speed) {
    m_coralWristMotor.set(speed);
    // if (getCoralEncoderPosition() >= 0.1) {
    //   m_coralWristMotor.set(speed.getAsDouble());
    // }
    // else{
    //   stopCoralWristMotor();
    // }
  }
  

  private void stopAlgaeWristMotor() {
    m_algaeWristMotor.stopMotor();
  }
  private void stopCoralWristMotor() {
    m_coralWristMotor.stopMotor();
  }

  private void stopBothMotors() {
    m_algaeWristMotor.stopMotor();
    m_coralWristMotor.stopMotor();
  }

  public Command runCoralWrist(DoubleSupplier speed){
    return run(()-> setCoralWristMotor(speed.getAsDouble()))
    .finallyDo(()-> stopCoralWristMotor());
  }

  public Command runAlgaeWrist(DoubleSupplier speed){
    return run(()-> setAlgaeWristMotor(speed.getAsDouble()))
    .finallyDo(()-> stopAlgaeWristMotor());
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
              currentSetpoint = Setpoint.kStore;
            case kProcessor:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
            case kFeederStation:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kFeederStation;
              coralWristCurrentTarget = CoralWristSetpoints.kFeederStation;
              currentSetpoint = Setpoint.kFeederStation;
              break;
            case kNet:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
              break;
            case kL1:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL1;
              coralWristCurrentTarget = CoralWristSetpoints.kL1;
              currentSetpoint = Setpoint.kL1;
              break;
            case kL2:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL2;
              coralWristCurrentTarget = CoralWristSetpoints.kL2;
              currentSetpoint = Setpoint.kL2;
              break;
            case kL3:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL3;
              coralWristCurrentTarget = CoralWristSetpoints.kL3;
              currentSetpoint = Setpoint.kL3;
              break;
            case kL4:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL4;
              coralWristCurrentTarget = CoralWristSetpoints.kL4;
              currentSetpoint = Setpoint.kL4;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    m_CurrentSetpoint = m_Capstan.getCurentElevatorSetpoint();
    // This method will be called once per scheduler run
  }
}
