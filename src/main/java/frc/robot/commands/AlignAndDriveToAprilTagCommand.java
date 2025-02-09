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

import java.util.List;
import java.util.Optional;

public class AlignAndDriveToAprilTagCommand extends Command {
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    private final PIDController rotationController, driveController;
    private final List<Integer> targetTags;
    
    private static final double ROTATION_TOLERANCE = 1.0; // Degrees
    private static final double DISTANCE_TOLERANCE = 0.1; // Meters (10 cm)
    private static final double MAX_ROT_SPEED = 1.0; // Radians/sec
    private static final double MAX_DRIVE_SPEED = 1.0; // Meters/sec
    private static final double DISTANCE_SETPOINT = 0.5; //Meters

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

    @Override
    public void initialize() {
        rotationController.reset();
        driveController.reset();
    }

    @Override
    public void execute() {
        // Get AprilTag pose relative to robot
        Optional<Pose2d> tagPose = m_vision.getTargetPose(targetTags, m_vision.getAllUnreadResults());

        if (tagPose.isEmpty()) {
            m_drive.stop();
            Commands.print("No AprilTag Detected");
            return; // No valid AprilTag detected
        }

        Pose2d target = tagPose.get();
        double yawError = ToolBox.getYawToTarget(target);
        double distanceError = ToolBox.getDistanceToTarget(target);

        double turnSpeed = rotationController.calculate(yawError, 0.0);
        double driveSpeed = driveController.calculate(distanceError, DISTANCE_SETPOINT); // Stop at 0.5m

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
        m_drive.stop();
    }
}
