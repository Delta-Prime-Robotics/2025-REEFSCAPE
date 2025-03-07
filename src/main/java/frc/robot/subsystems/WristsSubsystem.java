// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Capstan;
import static frc.robot.Constants.WristConstants.*;
import frc.robot.subsystems.CapstanSubsystem.Setpoint;

public class WristsSubsystem extends SubsystemBase {

  private final SparkMax m_algaeWristMotor;
  private final SparkMax m_coralWristMotor;

  private final AbsoluteEncoder m_algaeWristEncoder;
  private final RelativeEncoder m_coralWristEncoder;

  private final PIDController m_algaeWristPIDController;
  private final PIDController m_coralWristPIDController;

  private final ArmFeedforward m_algaeFeedforward;

  private final CapstanSubsystem m_Capstan;

  private double algaeWristCurrentTarget = AlgaeWristSetpoints.kStore;
  private double coralWristCurrentTarget = CoralWristSetpoints.kStore;
  private Setpoint currentSetpoint;

  /** Creates a new WristsSubsystem. */
  public WristsSubsystem(CapstanSubsystem capstan) {
    this.m_Capstan = capstan;
    currentSetpoint = m_Capstan.getCurentElevatorSetpoint();
    
    m_algaeWristMotor = new SparkMax(kAlgaeWristCanId, MotorType.kBrushless);
    m_coralWristMotor = new SparkMax(kCoralWristCanId, MotorType.kBrushless);

    m_algaeWristEncoder = m_algaeWristMotor.getAbsoluteEncoder();
    m_coralWristEncoder = m_coralWristMotor.getEncoder();

    m_algaeWristPIDController = new PIDController(0, 0, 0);
    m_coralWristPIDController = new PIDController(0, 0, 0);

    m_algaeFeedforward = new ArmFeedforward(0,0.34,1.56, 0.01);
      
    // m_algaeWristMotor.configure(Capstan.algaeWristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_coralWristMotor.configure(Capstan.coralWristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_algaeWristMotor.configure(Capstan.algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralWristMotor.configure(Capstan.coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(runOnce(()-> stopBothMotors()));
  }

  /**
   * @return a value between [0,1) which wraps
   */
  private double getAlgaeEncoderPosition() {
    return m_algaeWristEncoder.getPosition();
  }

  private double getCoralEncoderPosition() {
    return m_coralWristEncoder.getPosition();
  }

  private void setAlgaeWristMotor(Double speed) {
    double pos = getAlgaeEncoderPosition();

    if (pos >= kAlgaeLowerLimit && pos <= kAlgaeUpperLimit) {
      stopAlgaeWristMotor();
    }
    else{
      m_algaeWristMotor.set(speed);
    }
  }
  
  private void stopAlgaeWristMotor() {
    m_algaeWristMotor.stopMotor();
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
 
  // public Command moveBothToSetpointCommand() {
  //   return run(()-> {

  //   });
  // }

  public Command moveAlgaeWristToSetpointCommand() {
    return run(
      () -> {setAlgaeWristMotor(
        m_algaeWristPIDController.calculate(
          getAlgaeEncoderPosition(),
          algaeWristCurrentTarget)
        + m_algaeFeedforward.calculate(algaeWristCurrentTarget, algaeWristCurrentTarget, 0)

      );});
  }

  // public Command moveCoralWristToSetpointCommand() {
  //   return run();
  // }

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
              break;
            case kProcessor:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              break;
            case kFeederStation:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kFeederStation;
              coralWristCurrentTarget = CoralWristSetpoints.kFeederStation;
              break;
            case kNet:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kProcessor;
              coralWristCurrentTarget = CoralWristSetpoints.kProcessor;
              break;
            case kL1:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL1;
              coralWristCurrentTarget = CoralWristSetpoints.kL1;
              break;
            case kL2:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL2;
              coralWristCurrentTarget = CoralWristSetpoints.kL2;
              break;
            case kL3:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL3;
              coralWristCurrentTarget = CoralWristSetpoints.kL3;
              break;
            case kL4:
              algaeWristCurrentTarget = AlgaeWristSetpoints.kL4;
              coralWristCurrentTarget = CoralWristSetpoints.kL4;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    currentSetpoint = m_Capstan.getCurentElevatorSetpoint();
    SmartDashboard.putNumber("Algae Wrist Pos", getAlgaeEncoderPosition());
    // This method will be called once per scheduler run
  }
}
