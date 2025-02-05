// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Capstan;
import frc.robot.Constants.CapstanConstants;
import frc.robot.Constants.CapstanConstants.WristSetpoints;
import frc.robot.Constants.CapstanConstants.ElevatorSetpoints;

public class CapstanSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kStore,
    kFeederStation,
    kProcessor,
    kL1,
    kL2,
    kL3,
    kL4;
  }

  private final SparkMax m_elevatorMotor;
  private final SparkMax m_wristMotor;

  private final RelativeEncoder m_elevatorEncoder;
  private final RelativeEncoder m_wristEncoder;

  private final SparkClosedLoopController m_elevatorPIDController;
  private final SparkClosedLoopController m_wristPIDController;

  private final DigitalInput m_hallSensor;

  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kStore;
  private double wristCurrentTarget = WristSetpoints.kStore;
  private Setpoint currentSetpoint = Setpoint.kStore;

  /** Creates a new CapstanSubsystem. */
  public CapstanSubsystem() {
    m_elevatorMotor = new SparkMax(CapstanConstants.kElevatorCanId, MotorType.kBrushless);
    m_wristMotor = new SparkMax(CapstanConstants.kWristCanId, MotorType.kBrushless);

    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_wristEncoder = m_wristMotor.getEncoder();

    m_elevatorPIDController = m_elevatorMotor.getClosedLoopController();
    m_wristPIDController = m_elevatorMotor.getClosedLoopController();

    m_elevatorMotor.configure(Capstan.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wristMotor.configure(Capstan.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevatorEncoder.setPosition(0);
    //m_wristEncoder.setPosition(0);

    m_hallSensor = new DigitalInput(0);

  }

  public boolean isElevatorAtBottom() {
    return m_hallSensor.get();
  }

  public Setpoint curentCapstanSetpoint() {
    return currentSetpoint;
  }

  private void moveToSetpoint() {
    m_wristPIDController.setReference(wristCurrentTarget, ControlType.kMAXMotionPositionControl);
    m_elevatorPIDController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
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

    /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {

    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kStore:
              wristCurrentTarget = WristSetpoints.kStore;
              elevatorCurrentTarget = ElevatorSetpoints.kStore;
              currentSetpoint = Setpoint.kStore;
            case kProcessor:
              wristCurrentTarget = WristSetpoints.kProcessor;
              elevatorCurrentTarget = ElevatorSetpoints.kProcessor;
              currentSetpoint = Setpoint.kProcessor;
            case kFeederStation:
              wristCurrentTarget = WristSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              currentSetpoint = Setpoint.kFeederStation;
              break;
            case kL1:
              wristCurrentTarget = WristSetpoints.kL1;
              elevatorCurrentTarget = ElevatorSetpoints.kL1;
              currentSetpoint = Setpoint.kL1;
              break;
            case kL2:
              wristCurrentTarget = WristSetpoints.kL2;
              elevatorCurrentTarget = ElevatorSetpoints.kL2;
              currentSetpoint = Setpoint.kL2;
              break;
            case kL3:
              wristCurrentTarget = WristSetpoints.kL3;
              elevatorCurrentTarget = ElevatorSetpoints.kL3;
              currentSetpoint = Setpoint.kL3;
              break;
            case kL4:
              wristCurrentTarget = WristSetpoints.kL4;
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
