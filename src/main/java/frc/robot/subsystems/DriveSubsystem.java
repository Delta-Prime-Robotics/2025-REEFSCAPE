// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.NotActiveException;
import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final static AHRS m_navx = new AHRS(NavXComType.kMXP_SPI); 
  //public final static AHRS m_navx = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k40Hz); 

  private final Field2d m_Field = new Field2d();

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     m_navx.getRotation2d(),//Rotation2d.fromDegrees(m_navx.getAngle()),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     });
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getHeading(),
    getModulePositions(),
    new Pose2d());
    
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    try {
      m_navx.reset();
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
    }

    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
                  this::getPose, // Robot pose supplier
                  this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                  this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                  (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                  new PPHolonomicDriveController( //, this should likely live in your
                                                  // Constants class
                          new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                          new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
                  ),
                  config, // Configuration for the path follower
                  () -> {
                      // Boolean supplier that controls when the path will be mirrored for the red
                      // alliance
                      // This will flip the path being followed to the red side of the field.
                      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                      var alliance = DriverStation.getAlliance();
                      if (alliance.isPresent()) {
                          return alliance.get() == DriverStation.Alliance.Red;
                      }
                      return false;
                  },
                  this // Reference to this subsystem to set requirements
          );
    } catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
      
    //field
    SmartDashboard.putData("Field", m_Field);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
      getHeading(),
      getModulePositions()
      );
    
    //robot postion on field
    m_Field.setRobotPose(getPose());
    SmartDashboard.putNumber("Get Heading", this.getHeading().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      getHeading(),
      getModulePositions(),    
      pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    
    Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent() && ally.get() == Alliance.Red) {
          xSpeedDelivered *= -1;
          ySpeedDelivered *= -1;
    }
    
    drive(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), fieldRelative);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()); //m_navx.getRotation2d()
    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Retrieves the current states of all swerve modules.
   *
   * @return An array of the current states of the swerve modules.
   */
  private SwerveModuleState[] getModuleStates() {
      return new SwerveModuleState[] {
              m_frontLeft.getState(),
              m_frontRight.getState(),
              m_rearLeft.getState(),
              m_rearRight.getState()
      };
  }

  /**
   * Retrieves the current positions of all swerve modules.
   *
   * @return An array of the current positions of the swerve modules.
   */
  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navx.reset();
    resetOdometry(new Pose2d(getPose().getTranslation(), m_navx.getRotation2d()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_navx.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
