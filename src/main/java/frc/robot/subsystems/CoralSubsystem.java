// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CapstanSubsystem.Setpoint;
import frc.robot.Configs.CoralConfig;
import frc.robot.Constants.CoralConstants;;

public class CoralSubsystem extends SubsystemBase {
  private final SparkMax m_coralLeader;
  private final SparkMax m_coralFollower;
  private final SparkMaxConfig m_followerConfig = new SparkMaxConfig();
  //private static LaserCan m_coralLaser;
  
  private final CapstanSubsystem m_Capstan;
  private Setpoint m_CurrentSetpoint;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem(CapstanSubsystem capstan) {
    this.m_Capstan = capstan;
    m_CurrentSetpoint = m_Capstan.curentCapstanSetpoint();

    m_coralLeader = new SparkMax(CoralConstants.kCoralLeader, MotorType.kBrushless);
    m_coralFollower = new SparkMax(CoralConstants.kCoralFollower, MotorType.kBrushless);
    //m_coralLaser = new LaserCan(CoralConstants.kCoralLaser);

    m_followerConfig
      .apply(CoralConfig.coralConfig)
      .follow(m_coralLeader, true);

    // m_coralLeader.configure(CoralConfig.coralConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_coralFollower.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_coralLeader.configure(CoralConfig.coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralFollower.configure(m_followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Checks if coral is detected by using the coral laser sensor.
   * @return true if the distance measured by the coral laser sensor is less than 10 millimeters, false otherwise.
   */
  public boolean isCoralDetected(){
    return false;
  }

  public void setMotor(double speed){
    m_coralLeader.set(speed);
  }

  public void stopMotor() {
    m_coralLeader.stopMotor();
  }
  
  public Command runCoralMotor(double speed) {
    return run(()-> setMotor(speed))
    .finallyDo(()-> stopMotor());
  }

  public Command runCoralManually(double speed, BooleanSupplier reversed) {
    return new ConditionalCommand(runCoralMotor(speed*-1.0), runCoralMotor(speed), reversed);
  }

  // public Command autoIntakeCoral() {
  //   return runCoralMotor(CoralConstants.kFeederStationSpeed)
  //       .until(() -> isCoralDetected())
  //       .finallyDo(() -> stopCoralMotor());
  // }

  public Command autoCoralSpeeds(){
      switch (m_CurrentSetpoint) {
        case kFeederStation:
          return runCoralMotor(1);
        case kL1:
          return runCoralMotor(-0.25);
        case kL2:
          return runCoralMotor(-0.5);
        case kL3:
          return runCoralMotor(-0.75);
        case kL4:
          return runCoralMotor(-1.0);
        default:
          return runOnce(() -> stopMotor());
      }
  }

  @Override
  public void periodic() {
    m_CurrentSetpoint = m_Capstan.curentCapstanSetpoint();
    // This method will be called once per scheduler run
  }
}
