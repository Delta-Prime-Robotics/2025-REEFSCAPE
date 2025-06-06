package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class MoveByDistanceCommand extends Command {
    private final DriveSubsystem m_drive;
    private final PIDController xController, yController, rotController;
    private double targetX, targetY, targetRot;
    private final double xMeters, yMeters, radians;
    private static final double meterTOLERANCE = 0.1;// 10cm tolerance //change to bigger than this
    private static final double angleTOLERANCE = 5;

    /**
     * Command to move the robot by a specified distance in the X and Y directions and rotate by a specified angle.
     * This movement is robot relitive.
     * For rotation please use the range (180,-180] and not (0,360)
     * 
     * @param driveSubsystem The drive subsystem used to control the robot's movement.
     * @param xMeters The distance to move in the X direction, in meters.
     * @param yMeters The distance to move in the Y direction, in meters.
     * @param radians The angle to rotate, in radians.
     */
    public MoveByDistanceCommand(DriveSubsystem driveSubsystem, double xMeters, double yMeters, double radians) {
        this.m_drive = driveSubsystem;
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.radians = radians;
        addRequirements(driveSubsystem);

        // Initialize PID controllers for X and Y motion and Rotation
        xController = new PIDController(1.0, 0.0, 0.0);
        yController = new PIDController(1.0, 0.0, 0.0);
        rotController = new PIDController(1.0, 0.0, 0.0);

        xController.setTolerance(meterTOLERANCE);
        yController.setTolerance(meterTOLERANCE);
        rotController.setTolerance(angleTOLERANCE);
    }

    @Override
    public void initialize() {
        // Calculate target pose
        Pose2d initialPose = m_drive.getPose();
        targetX = initialPose.getX() + xMeters;
        targetY = initialPose.getY() + yMeters;
        targetRot = initialPose.getRotation().rotateBy(new Rotation2d(radians)).getRadians();
        //Setpoints
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        rotController.setSetpoint(Units.degreesToRadians(angleTOLERANCE));
    }

    @Override
    public void execute() {
        // Compute speed commands using PID controllers
        double xSpeed = xController.calculate(m_drive.getPose().getX());
        double ySpeed = yController.calculate(m_drive.getPose().getY());
        double turnSpeed = rotController.calculate(m_drive.getPose().getRotation().getRadians());

        // Convert to swerve module states
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        m_drive.driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_drive.stop();
    }
}
