// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.CoralConfig;
import frc.robot.Constants.CoralConstants;;

public class CoralSubsystem extends SubsystemBase {
  private static SparkMax m_coralMotor;
  private static LaserCan m_coralLaser;
  
  public enum OutSpeeds {
    kL1,
    kL2,
    kL3,
    kL4
  }

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    m_coralMotor = new SparkMax(CoralConstants.kCoralMotor, MotorType.kBrushless);

    m_coralLaser = new LaserCan(CoralConstants.kCoralLaser);

    m_coralMotor.configure(CoralConfig.coralConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void configurePersist(){
    m_coralMotor.configure(CoralConfig.coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double getCoralLazer_mm(){
    LaserCan.Measurement measurement = m_coralLaser.getMeasurement();

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
      // DriverStation.reportError("Error Could Not Read LaserCan: NullPointerException" , true );
      return 0;
    }
  }

  /**
   * Checks if coral is detected by using the coral laser sensor.
   * @return true if the distance measured by the coral laser sensor is less than 10 millimeters, false otherwise.
   */
  public boolean isCoralDetected(){
    if (getCoralLazer_mm() < 10) {return true;}
    else {return false;}
  }

  private void setCoralMotor(double speed){
    m_coralMotor.set(speed);
  }

  private void stopCoralMotor() {
    m_coralMotor.stopMotor();
  }
  
  public Command runCoralMotor(double speed) {
    return this.startEnd(
      () -> setCoralMotor(speed),
      () -> stopCoralMotor());
  }

  public Command autoIntakeCoral() {
    return runCoralMotor(1)
        .until(() -> isCoralDetected())
        .finallyDo(() -> stopCoralMotor());
  }

  public Command outtakeCoral(OutSpeeds level){
      switch (level) {
        case kL1:
          return runCoralMotor(-0.25);
        case kL2:
          return runCoralMotor(-0.5);
        case kL3:
          return runCoralMotor(-0.75);
        case kL4:
          return runCoralMotor(-1.0);
        default:
          return runOnce(() -> stopCoralMotor());
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
