// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.time.format.DateTimeParseException;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private final SparkFlexConfig m_drivingConfig;
  private final SparkMaxConfig m_turningConfig;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
   

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    //Configs
    m_drivingConfig = new SparkFlexConfig();
    m_turningConfig = new SparkMaxConfig();

    // ** Driving Encoder Config
    m_drivingConfig.encoder
      .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // ** Turning Encoder Config
    m_turningConfig.absoluteEncoder
      .inverted(ModuleConstants.kTurningEncoderInverted)
    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
      .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    // ** Driving PID Config
    m_drivingConfig.closedLoop
    	.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    	.pidf(ModuleConstants.kDrivingP, 
            ModuleConstants.kDrivingI,
            ModuleConstants.kDrivingD,
            ModuleConstants.kDrivingFF)
    	.outputRange(ModuleConstants.kDrivingMinOutput,
    		ModuleConstants.kDrivingMaxOutput);
		// Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
    
    // ** Turning PID Config
    m_turningConfig.closedLoop
    	.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    	.positionWrappingEnabled(true)
    	.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
    	.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
      .pidf(ModuleConstants.kTurningP,
            ModuleConstants.kTurningI,
            ModuleConstants.kTurningD,
            ModuleConstants.kTurningFF)
      .outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);
    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!


    // ** Driving Motor Config
    m_drivingConfig
      .idleMode(ModuleConstants.kDrivingMotorIdleMode)
      .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    
    // ** Turning Motor Config
    m_turningConfig
      .idleMode(ModuleConstants.kTurningMotorIdleMode)
      .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    
    // * CONFIGURE MOTORS
    m_drivingSparkFlex.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @SuppressWarnings("deprecation")
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // ! will be Deprecated next year
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
        correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /**Returns an double array with {DriveMotorTemp, TurningMotorTemp} */
  public double[] motorTemps(){
    double temps[] = {m_drivingSparkFlex.getMotorTemperature(), m_turningSparkMax.getMotorTemperature()};
    return temps;
  }
}
