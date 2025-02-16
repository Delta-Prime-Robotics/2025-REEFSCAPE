package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class ToolBox {
  public static double getYawToTarget(Pose2d target) {
      return target.getRotation().getDegrees(); // Degrees
  }

  public static double getDistanceToTarget(Pose2d target) {
    return target.getTranslation().getNorm(); // Meters
  }

}
