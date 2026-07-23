package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;

/**
 * VisionSubsystem is the robot's "eyes." It uses PhotonVision running on a
 * coprocessor (e.g. an Orange Pi) to spot AprilTags on the field and figure out
 * exactly where the robot is standing.
 *
 * <p>This replaces the old Limelight-based vision. It runs TWO cameras. Every loop
 * it asks each camera "where do you think the robot is?" and, if the answer looks
 * trustworthy, it hands that position to the swerve drive's pose estimator. The
 * pose estimator then blends camera data with wheel/gyro data to keep an accurate
 * map of the robot on the field — which is what makes PathPlanner autos precise.
 */
public class VisionSubsystem extends SubsystemBase {

  // The swerve drive owns the pose estimator we feed our measurements into.
  private final SwerveDrive swerveDrive;

  // The field map: tells us the real-world location of every AprilTag. Without this,
  // seeing a tag is meaningless — we need to know where that tag physically is.
  private final AprilTagFieldLayout fieldLayout;

  // One entry per physical camera. Each pairs a camera with its own pose estimator.
  private final List<CameraUnit> cameras = new ArrayList<>();

  // The most recent trustworthy pose we got from vision, and when we got it.
  // visionAlignCommand() and updateHeadingWithVision() reuse this.
  private Pose2d lastVisionPose = null;
  private double lastVisionTimestamp = 0.0;

  /**
   * Bundles a camera together with the pose estimator that interprets its images.
   */
  private static final class CameraUnit {
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    // True only for cameras rigidly bolted to the chassis. A camera on the turret
    // moves as the turret swivels, so its fixed robot-to-camera transform is wrong
    // for global pose — those cameras are used for aiming only, never odometry.
    final boolean contributesToPose;
    // The AprilTag targets from this camera's most recent frame. Updated every loop
    // so commands (e.g. turret aim) can read fresh detections.
    List<PhotonTrackedTarget> latestTargets = new ArrayList<>();

    CameraUnit(String name, PhotonCamera camera, PhotonPoseEstimator estimator, boolean contributesToPose) {
      this.name = name;
      this.camera = camera;
      this.estimator = estimator;
      this.contributesToPose = contributesToPose;
    }
  }

  /**
   * CONSTRUCTOR. We take the SwerveSubsystem so we can push our position
   * corrections straight into its pose estimator.
   */
  public VisionSubsystem(SwerveSubsystem swerve) {
    this.swerveDrive = swerve.getSwerveDrive();

    // Load the current season's AprilTag field layout that ships with WPILib.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Front camera is fixed to the chassis -> contributes to pose estimation.
    addCamera(VisionConstants.kCameraLeftName, VisionConstants.kRobotToCameraLeft, true);
    // Turret camera moves with the turret -> aiming only, EXCLUDED from pose estimation.
    addCamera(VisionConstants.kCameraRightName, VisionConstants.kRobotToCameraRight, false);
  }

  /**
   * Sets up a single camera and its pose estimator.
   *
   * @param name          the PhotonVision nickname of the camera
   * @param robotToCamera where the camera is mounted relative to robot center
   */
  private void addCamera(String name, Transform3d robotToCamera, boolean contributesToPose) {
    PhotonCamera camera = new PhotonCamera(name);

    // MULTI_TAG_PNP_ON_COPROCESSOR is the best strategy: when the camera sees 2+ tags,
    // the coprocessor triangulates one highly-accurate pose. If it only sees ONE tag,
    // it can't triangulate, so we fall back to LOWEST_AMBIGUITY (trust the clearest tag).
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    cameras.add(new CameraUnit(name, camera, estimator, contributesToPose));
  }

  /**
   * This runs every 20ms. For each camera we pull any new frames, turn them into
   * a robot position, sanity-check it, and feed the good ones to the pose estimator.
   */
  @Override
  public void periodic() {
    int acceptedThisLoop = 0;

    for (CameraUnit unit : cameras) {
      // getAllUnreadResults() returns every frame that arrived since we last checked.
      // Processing all of them (instead of just the newest) keeps timestamps honest.
      List<PhotonPipelineResult> results = unit.camera.getAllUnreadResults();

      // Pose estimation: ONLY chassis-fixed cameras. The turret camera is skipped here
      // (its transform isn't valid while the turret rotates) but still tracks targets
      // below for aiming.
      if (unit.contributesToPose) {
        for (PhotonPipelineResult result : results) {
          Optional<EstimatedRobotPose> maybeEstimate = unit.estimator.update(result);
          if (maybeEstimate.isEmpty()) {
            continue; // No usable tags in this frame.
          }

          EstimatedRobotPose estimate = maybeEstimate.get();
          int tagCount = result.getTargets().size();
          double avgDistance = averageTagDistanceMeters(result);

          // Throw out estimates from tags that are too far away — they get jittery
          // and can yank the robot's position around.
          if (avgDistance > VisionConstants.kMaxAverageTagDistanceMeters) {
            continue;
          }

          Pose2d visionPose = estimate.estimatedPose.toPose2d();
          Matrix<N3, N1> stdDevs = computeStdDevs(tagCount, avgDistance);

          // Hand the position to the swerve pose estimator. The stdDevs tell it how
          // much to trust us versus the wheel odometry.
          swerveDrive.addVisionMeasurement(visionPose, estimate.timestampSeconds, stdDevs);

          lastVisionPose = visionPose;
          lastVisionTimestamp = estimate.timestampSeconds;
          acceptedThisLoop++;
        }
      }

      // Publish per-camera targeting data from the newest frame we received this loop.
      // (Last element of getAllUnreadResults() is the most recent.)
      if (!results.isEmpty()) {
        PhotonPipelineResult newest = results.get(results.size() - 1);
        unit.latestTargets = newest.getTargets();
        publishTargetingData(unit.name, newest);
      }
    }

    // Overall vision health so drivers/programmers can see vision is alive.
    SmartDashboard.putBoolean("Vision/HasTarget", acceptedThisLoop > 0);
    SmartDashboard.putNumber("Vision/AcceptedMeasurements", acceptedThisLoop);
    if (lastVisionPose != null) {
      SmartDashboard.putNumber("Vision/LastX", lastVisionPose.getX());
      SmartDashboard.putNumber("Vision/LastY", lastVisionPose.getY());
      SmartDashboard.putNumber("Vision/AgeSeconds", Timer.getFPGATimestamp() - lastVisionTimestamp);
    }
  }

  /**
   * Publishes raw targeting data for one camera to SmartDashboard, namespaced under
   * "Vision/&lt;cameraName&gt;/...". This is the per-tag detail (yaw, pitch, area,
   * distance, ambiguity, and which tag IDs are in view) — handy for aiming and for
   * debugging why a camera is or isn't producing a pose.
   */
  private void publishTargetingData(String cameraName, PhotonPipelineResult result) {
    String prefix = "Vision/" + cameraName + "/";
    List<PhotonTrackedTarget> targets = result.getTargets();

    SmartDashboard.putBoolean(prefix + "HasTargets", result.hasTargets());
    SmartDashboard.putNumber(prefix + "TargetCount", targets.size());

    // The list of every AprilTag ID currently visible to this camera.
    double[] tagIds = new double[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      tagIds[i] = targets.get(i).getFiducialId();
    }
    SmartDashboard.putNumberArray(prefix + "TagIDs", tagIds);

    // Detailed numbers for the single best target (clearest / lowest ambiguity).
    if (result.hasTargets()) {
      PhotonTrackedTarget best = result.getBestTarget();
      SmartDashboard.putNumber(prefix + "BestTagID", best.getFiducialId());
      SmartDashboard.putNumber(prefix + "BestYawDeg", best.getYaw());
      SmartDashboard.putNumber(prefix + "BestPitchDeg", best.getPitch());
      SmartDashboard.putNumber(prefix + "BestArea", best.getArea());
      SmartDashboard.putNumber(prefix + "BestAmbiguity", best.getPoseAmbiguity());
      SmartDashboard.putNumber(prefix + "BestDistanceMeters",
          best.getBestCameraToTarget().getTranslation().getNorm());
    } else {
      // Clear the "best" fields so stale numbers don't linger when tags leave view.
      SmartDashboard.putNumber(prefix + "BestTagID", -1);
      SmartDashboard.putNumber(prefix + "BestYawDeg", 0.0);
      SmartDashboard.putNumber(prefix + "BestPitchDeg", 0.0);
      SmartDashboard.putNumber(prefix + "BestArea", 0.0);
      SmartDashboard.putNumber(prefix + "BestAmbiguity", 0.0);
      SmartDashboard.putNumber(prefix + "BestDistanceMeters", 0.0);
    }
  }

  /**
   * Averages the straight-line distance from the camera to every tag it sees.
   * Farther tags = less reliable, so we use this both to reject and to scale trust.
   */
  private double averageTagDistanceMeters(PhotonPipelineResult result) {
    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty()) {
      return Double.MAX_VALUE;
    }
    double total = 0.0;
    for (PhotonTrackedTarget target : targets) {
      total += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return total / targets.size();
  }

  /**
   * Picks how much to trust this measurement. Multiple tags start out very trusted;
   * a single tag starts out barely trusted. Either way, the farther the tags, the
   * more we inflate the numbers (= trust less).
   */
  private Matrix<N3, N1> computeStdDevs(int tagCount, double avgDistanceMeters) {
    Matrix<N3, N1> base =
        (tagCount > 1) ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;

    // Grow the standard deviation with the square of distance. Close tags barely
    // change it; far tags multiply it a lot.
    double distanceFactor = 1.0 + (avgDistanceMeters * avgDistanceMeters) / 30.0;
    return base.times(distanceFactor);
  }

  /**
   * Command for PathPlanner autos (registered as "VisionAlign"). Snaps the robot's
   * internal map to the latest vision pose so a high-precision move starts dead-on.
   * Does nothing if we haven't seen a tag recently.
   */
  public Command visionAlignCommand() {
    return runOnce(() -> {
      if (lastVisionPose != null && (Timer.getFPGATimestamp() - lastVisionTimestamp) < 0.5) {
        swerveDrive.resetOdometry(lastVisionPose);
      }
    });
  }

  /**
   * Aligns the robot's heading at the start of Teleop. Uses the latest vision
   * heading if we have a fresh one; otherwise falls back to the gyro/odometry heading.
   */
  public void updateHeadingWithVision() {
    swerveDrive.setHeadingCorrection(true);

    Rotation2d finalRotation;
    if (lastVisionPose != null && (Timer.getFPGATimestamp() - lastVisionTimestamp) < 0.5) {
      finalRotation = lastVisionPose.getRotation();
    } else {
      finalRotation = swerveDrive.getOdometryHeading();
    }

    swerveDrive.setGyroOffset(new Rotation3d(0.0, 0.0, finalRotation.getRadians()));
  }

  /**
   * @return the most recent trustworthy vision pose, if we have one.
   */
  public Optional<Pose2d> getLastVisionPose() {
    return Optional.ofNullable(lastVisionPose);
  }

  /**
   * Every AprilTag target currently seen, combined across all cameras into one list.
   * Useful for "is this tag visible anywhere" checks — but NOT for aiming, because a
   * target's yaw is measured relative to the camera that saw it, so combining cameras
   * mixes reference frames.
   *
   * @return the combined target list (empty if nothing is in view).
   */
  public List<PhotonTrackedTarget> getLatestTargets() {
    List<PhotonTrackedTarget> all = new ArrayList<>();
    for (CameraUnit unit : cameras) {
      all.addAll(unit.latestTargets);
    }
    return all;
  }

  /**
   * Latest AprilTag targets from ONE specific camera, by its PhotonVision name.
   * Yaw/pitch on these targets are relative to that camera's optical axis.
   *
   * @return that camera's latest targets, or an empty list if the name isn't found.
   */
  public List<PhotonTrackedTarget> getTargets(String cameraName) {
    for (CameraUnit unit : cameras) {
      if (unit.name.equals(cameraName)) {
        return unit.latestTargets;
      }
    }
    return new ArrayList<>();
  }

  /**
   * Latest targets from the TURRET-mounted camera specifically. This is what the
   * turret-aim command must use: it reads each target's getYaw(), which is only
   * meaningful relative to the camera physically on the turret.
   *
   * @return the turret camera's latest targets (empty if none in view).
   */
  public List<PhotonTrackedTarget> getTurretCameraTargets() {
    return getTargets(VisionConstants.kCameraRightName);
  }
}