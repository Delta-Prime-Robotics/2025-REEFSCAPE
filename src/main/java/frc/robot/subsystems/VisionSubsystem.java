// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Robot;

public class VisionSubsystem extends DriveSubsystem {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  
    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {
      m_camera = new PhotonCamera(kCameraName);
  
      m_photonEstimator = new PhotonPoseEstimator(
              kTagLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              kRobotToCam);

      m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }
  
  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : m_camera.getAllUnreadResults()) {
          visionEst = m_photonEstimator.update(change);
          updateEstimationStdDevs(visionEst, change.getTargets());
      }
      return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
          Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
          // No pose input. Default to single-tag std devs
          curStdDevs = kSingleTagStdDevs;

      } else {
          // Pose present. Start running Heuristic
          var estStdDevs = kSingleTagStdDevs;
          int numTags = 0;
          double avgDist = 0;

          // Precalculation - see how many tags we found, and calculate an average-distance metric
          for (var tgt : targets) {
              var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
              if (tagPose.isEmpty()) continue;
              numTags++;
              avgDist +=
                      tagPose
                              .get()
                              .toPose2d()
                              .getTranslation()
                              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          }

          if (numTags == 0) {
              // No tags visible. Default to single-tag std devs
              curStdDevs = kSingleTagStdDevs;
          } else {
              // One or more tags visible, run the full heuristic.
              avgDist /= numTags;
              // Decrease std devs if multiple targets are visible
              if (numTags > 1) estStdDevs = kMultiTagStdDevs;
              // Increase std devs based on (average) distance
              if (numTags == 1 && avgDist > 4)
                  estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
              else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
              curStdDevs = estStdDevs;
          }
      }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
      return curStdDevs;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Correct pose estimate with vision measurements
    var visionEst = this.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = this.getEstimationStdDevs();

                super.addVisionMeasurement(
                  est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
  }
}
