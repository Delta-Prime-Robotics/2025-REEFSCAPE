package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ToolBox;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.VisionConstants;

import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignAndDriveToAprilTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final PIDController rotationController, driveController;
  private final List<Integer> targetTags;

  private boolean targetVisible = false;
  private double targetYaw = 0.0; 
  private double targetRange = 0.0;
  private static final double ROTATION_TOLERANCE = 10; // Degrees
  private static final double DISTANCE_TOLERANCE = 0.1; // Meters (50 cm)
  private static final double MAX_ROT_SPEED = 1.0; // Radians/sec
  private static final double MAX_DRIVE_SPEED = 1.0; // Meters/sec
  private static final double DISTANCE_SETPOINT = 0.3; //Meters
  // private static final double VISION_DES_ANGLE_deg = 0.0;
  // private static final double VISION_DES_RANGE_m = 1.25;
  
  public AlignAndDriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem vision, List<Integer> targetTags) {
      this.m_drive = driveSubsystem;
      this.m_vision = vision;
      this.targetTags = targetTags;
      addRequirements(driveSubsystem, vision);

      rotationController = new PIDController(0.02, 0.0, 0.0);
      driveController = new PIDController(1.0, 0.0, 0.0);

      rotationController.setTolerance(ROTATION_TOLERANCE);
      driveController.setTolerance(DISTANCE_TOLERANCE);
  }

  public AlignAndDriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem vision, Integer targetTag) {
    this(driveSubsystem, vision, List.of(targetTag)); // Calls the primary constructor
}
  
  @Override
  public void initialize() {
      rotationController.reset();
      driveController.reset();
  }
  
  @Override
  public void execute() {
    targetVisible = false;

    if (!m_vision.getLatestResults().isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = m_vision.getLatestResults().get(m_vision.getLatestResults().size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        PhotonTrackedTarget target = result.getBestTarget();
        if (targetTags.contains(target.getFiducialId())) {
          // Found Tag 7, record its information
          targetYaw = target.getYaw();
        
          targetRange = target.bestCameraToTarget.getMeasureX().in(Meters);
          // PhotonUtils.calculateDistanceToTargetMeters(
          //           0.2, // Measured with a tape measure, or in CAD.
          //           AprilTagConstants.kID_HIGHTS.get( target.getFiducialId() ), 
          //           Units.degreesToRadians(30.0), // Measured with a protractor, or in CAD.
          //           Units.degreesToRadians( target.getPitch() ) );
          
          
          targetVisible = true;
        }
      }
    }
   SmartDashboard.putNumber("Range", targetRange);
   SmartDashboard.putNumber("Yaw", targetYaw);
    double turnSpeed = rotationController.calculate(targetYaw, 0.0);
    double driveSpeed = driveController.calculate(targetRange, DISTANCE_SETPOINT); // Stop at 0.5m

    turnSpeed = MathUtil.clamp(turnSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED); //Math.max(-MAX_ROT_SPEED, Math.min(MAX_ROT_SPEED, turnSpeed));
    driveSpeed = MathUtil.clamp(driveSpeed, MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

    ChassisSpeeds speeds = new ChassisSpeeds(driveSpeed, 0.0, turnSpeed);
    m_drive.drive(speeds, false);
  }

  @Override
  public boolean isFinished() {
      return rotationController.atSetpoint() && driveController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      System.out.println("FINISHED");
      m_drive.stop();
  }
}
