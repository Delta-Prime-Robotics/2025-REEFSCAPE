// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs.AlgaeConfig;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.CapstanSubsystem.Setpoint;

public class AlgaeSubsystem extends SubsystemBase {
  //Spark Maxes
  private final SparkMax m_algaeLeader;
  private final SparkMax m_algaeFollower;
  private static SparkMaxConfig m_algaeFollowerConfig = new SparkMaxConfig();
 
  //Lasers
  private final LaserCan m_algaeLaser;

  private final CapstanSubsystem m_Capstan;
  private Setpoint m_CurrentSetpoint;

  /** Creates a new Trident. */
  public AlgaeSubsystem(CapstanSubsystem capstan) {
    this.m_Capstan = capstan;
    m_CurrentSetpoint = m_Capstan.getCurentElevatorSetpoint();

    m_algaeLeader = new SparkMax(AlgaeConstants.kAlgaeLeader, MotorType.kBrushless);
    m_algaeFollower = new SparkMax(AlgaeConstants.kAlgaeFollower, MotorType.kBrushless);

    // RegionOfInterest roi = new RegionOfInterest(0, 0, 0, 0);
    m_algaeLaser = new LaserCan(AlgaeConstants.kAlgaeLaser); //.setRegionOfInterest();

    m_algaeFollowerConfig
      .apply(AlgaeConfig.algaeConfig)
      .follow(m_algaeLeader,true);

    // m_algaeLeader.configure(AlgaeConfig.algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_algaeFollower.configure(m_algaeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_algaeLeader.configure(AlgaeConfig.algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_algaeFollower.configure(m_algaeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // setDefaultCommand(runOnce(()-> stopMotors()));
  }


  private double getAlgaeLazer_mm(){
    LaserCan.Measurement measurement = m_algaeLaser.getMeasurement();

    if(measurement != null){
      if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        return measurement.distance_mm;
      } 
      else {
        DataLogManager.log("Oh no! The target is out of range, or we can't get a reliable measurement:" + measurement.status);
        return measurement.distance_mm; // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
      }
    }
    else{
      // DataLogManager.log("Error Could Not Read LaserCan: NullPointerException");
      DriverStation.reportError("Error Could Not Read LaserCan: NullPointerException" , true );
      return 0;
    }
  }

  /** 
   * Checks if algae is detected by using the algae laser sensor.
   * @return true if algae is detected (distance is less than 20 mm), false otherwise.
   */
  public boolean isAlgaeDetected(){
    if (getAlgaeLazer_mm() < 20) {return true;}
    else {return false;}
  }

  private void setMotors(double speed) {
    m_algaeLeader.set(-speed);
  } 

  private void stopMotors() {
    m_algaeLeader.stopMotor();
  }

  public void manualControl(DoubleSupplier speed){
    double previousSpeed = 0.0;
    double currentSpeed = speed.getAsDouble();
    if (currentSpeed != previousSpeed){
      previousSpeed = currentSpeed;
      setMotors(currentSpeed);
    }
    else{
      stopMotors();
    }
  }

  public Command runAlgaeMotors(double speed) {
    return runEnd(()-> setMotors(speed), ()-> stopMotors());
  }

  // public Command runAlgaeMotors(double speed) {
  //   return runEnd(
  //     ()-> System.out.println("Running Algae"),
  //    ()-> System.out.println("Algae Stopped"));
  // }

  public Command autoIntakeAlgae() {
    return runAlgaeMotors(AlgaeConstants.kIntakeSpeed)
        .until(()-> isAlgaeDetected())
        .finallyDo(()-> stopMotors())
        .withName("Auto Intake Algae");
  }

  public Command outToProcesser() { 
    return runAlgaeMotors(AlgaeConstants.kProcessorSpeed)
    .withName("Processer");
  }

  public Command outToNet() {
    return runAlgaeMotors(AlgaeConstants.kNetSpeed)
    .withName("Net");
  }

  public Command intakeAlgae() {
    return runAlgaeMotors(AlgaeConstants.kIntakeSpeed)
    .withName("Intake Algae");
  }
  
  // // public Command autoAlgaeSpeeds(Setpoint level) {
  // //   switch (level) {
  // public Command autoAlgaeSpeeds() {
  //   switch (m_CurrentSetpoint) {
  //     case kNet:
  //       return outToNet();
  //     case kProcessor:
  //       return outToProcesser();
  //     case kL2:
  //       return intakeAlgae();
  //     case kL3:
  //       return intakeAlgae();
  //     default:
  //       return runOnce(()-> stopMotors());
  //   }
  // }
  
  @Override
  public void periodic() {
    m_CurrentSetpoint = m_Capstan.getCurentElevatorSetpoint();
    // SmartDashboard.putData(this.getCurrentCommand());
    // This method will be called once per scheduler run
  }
}
