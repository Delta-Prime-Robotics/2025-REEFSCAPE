// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  private static List<PhotonPipelineResult> currentPipelineResults;
  private DriveSubsystem m_drive;
  
    /** Creates a new VisionSubsystem. */
    public VisionSubsystem(DriveSubsystem m_drive) {
      this.m_drive = m_drive;
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
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(List<PhotonPipelineResult> results) { 
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : results) {
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
  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
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

  /**
   * Retrieves the pose of the first target that matches any of the specified target tags.
   *
   * @param targetTags A list of integer IDs representing the target tags to search for.
   * @param allPipelineResults A list of PhotonPipelineResult objects containing the results from the vision pipeline.
   * @return An Optional containing the Pose2d of the first matching target if found, otherwise an empty Optional.
   */
  public Optional<Pose2d> getTargetPose(List<Integer> targetTags, List<PhotonPipelineResult> allPipelineResults) {
    List<PhotonPipelineResult> results = allPipelineResults;
    for (PhotonPipelineResult result : results) {
        if (!result.hasTargets()) continue;

        return result.getTargets().stream()
            .filter(target -> targetTags.contains(target.getFiducialId()))
            .map(target -> new Pose2d(target.getBestCameraToTarget().getX(),
                                      target.getBestCameraToTarget().getY(), 
                                      new Rotation2d(target.getYaw())))
            .findFirst();
    }

    return Optional.empty();
  }

  /**
   * Checks if any of the detected tags are present in the target list.
   *
   * @param targetTags A list of target tag IDs to search for.
   * @param allPipelineResults A list of PhotonPipelineResult objects containing the results from the vision pipeline.
   * @return true if any of the detected tags are in the target list, false otherwise.
   */
  public boolean isTargetPresent(List<Integer> targetTags, List<PhotonPipelineResult> allPipelineResults) {
    Optional<List<PhotonTrackedTarget>> targets = this.getTargets(allPipelineResults);
    if(!targets.isEmpty());
      List<PhotonTrackedTarget> result = targets.get();
      // Get the list of detected tag IDs
      List<Integer> detectedTagIds = 
      result.stream()
            .mapToInt(PhotonTrackedTarget::getFiducialId)
            .boxed()
            .collect(Collectors.toList());

      // Check if any of the detected tags are in our target list
      for (Integer tagId : detectedTagIds) {
          if (targetTags.contains(tagId)) {
              return true; // Found a matching tag
          }
    }
    return false;
  }

  public Optional<List<PhotonTrackedTarget>> getTargets(List<PhotonPipelineResult> allPipelineResults) {
    
    for(PhotonPipelineResult result : allPipelineResults) {
      if (!result.hasTargets()) continue;
      
      return Optional.of(result.getTargets());
    }
    return Optional.empty(); 
  }

  private void updateAllUnreadResults() {
      List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
      currentPipelineResults = results;
  };

  public List<PhotonPipelineResult> getAllUnreadResults(){
    return currentPipelineResults;
  }


  @Override
  public void periodic() {
    updateAllUnreadResults();
    // This method will be called once per scheduler run
    // Correct pose estimate with vision measurements
    var visionEst = this.getEstimatedGlobalPose(getAllUnreadResults());
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = this.getEstimationStdDevs();
                  m_drive.addVisionMeasurement(
                  est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
  }
}
