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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;

/**
 * VisionSubsystem is the robot's "eyes." It uses PhotonVision running on a
 * coprocessor (e.g. an Orange Pi) to spot AprilTags on the field and figure out
 * exactly where the robot is standing.
 *
 * <p>This replaces the old Limelight-based vision. It runs TWO cameras with separate jobs:
 * <ul>
 *   <li>FRONT camera (chassis-fixed): robot localization. Its poses go into the swerve drive's
 *       pose estimator, which blends them with wheel/gyro data for PathPlanner.</li>
 *   <li>TURRET camera: finds where the TURRET is on the field and which way it faces, from
 *       the camera alone, for aiming only. It never touches the robot pose; see
 *       {@link #getTurretFieldPosition()}.</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  // The swerve drive owns the pose estimator we feed our measurements into.
  private final SwerveDrive swerveDrive;

  // The turret camera's mounting changes as the turret turns, so we read the turret angle.
  private final TurretSubsystem turret;

  // The field map: tells us the real-world location of every AprilTag. Without this,
  // seeing a tag is meaningless — we need to know where that tag physically is.
  private final AprilTagFieldLayout fieldLayout;

  // One entry per physical camera. Each pairs a camera with its own pose estimator.
  private final List<CameraUnit> cameras = new ArrayList<>();

  // The most recent trustworthy pose we got from vision, and when we got it.
  // visionAlignCommand() and updateHeadingWithVision() reuse this.
  private Pose2d lastVisionPose = null;
  private double lastVisionTimestamp = 0.0;

  // --- Turret pose from the turret camera (separate from the robot pose) ---
  // Raw Pigeon yaw by FPGA time. Raw = not affected by the Y button / alliance flip, so the
  // CHANGE between a frame and now is always right, even though its zero is arbitrary.
  private final TimeInterpolatableBuffer<Rotation2d> rawYawHistory =
      TimeInterpolatableBuffer.createBuffer(1.0);
  // Latest camera fix: turret axis position, and the robot's field heading worked out from it
  // (turret field heading minus the turret encoder angle at that frame). Null until the first.
  private Translation2d turretFixPosition = null;
  private Rotation2d robotHeadingAtFix = null;
  private Rotation2d rawYawAtFix = null;
  private double lastTurretFixTimestamp = 0.0;
  // How far the robot has driven since the fix (wheel speeds, turned by the heading above).
  private Translation2d travelledSinceFix = Translation2d.kZero;
  private double lastTravelTimestamp = -1.0;
  private final StructPublisher<Pose2d> turretPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Turret/FieldPose", Pose2d.struct).publish();

  /**
   * Bundles a camera together with the pose estimator that interprets its images.
   */
  private static final class CameraUnit {
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    // True for the camera riding on the turret. It finds the turret's position for aiming and
    // is NOT used for the robot pose.
    final boolean onTurret;
    // This camera's latest raw result: the robot pose (front camera) or the turret pose
    // (turret camera), published before filtering so rejected ones can be seen too.
    final StructPublisher<Pose2d> posePublisher;
    // The AprilTag targets from this camera's most recent frame. Updated every loop
    // so commands (e.g. turret aim) can read fresh detections.
    List<PhotonTrackedTarget> latestTargets = new ArrayList<>();

    CameraUnit(String name, PhotonCamera camera, PhotonPoseEstimator estimator, boolean onTurret) {
      this.name = name;
      this.camera = camera;
      this.estimator = estimator;
      this.onTurret = onTurret;
      this.posePublisher = NetworkTableInstance.getDefault()
          .getStructTopic("Vision/" + name + (onTurret ? "/TurretPose" : "/RobotPose"), Pose2d.struct).publish();
    }
  }

  /**
   * CONSTRUCTOR. We take the SwerveSubsystem so we can push front-camera position
   * corrections into its pose estimator (and read the gyro / wheel speeds), and the
   * TurretSubsystem so we know the turret angle when each turret-camera frame was taken.
   */
  public VisionSubsystem(SwerveSubsystem swerve, TurretSubsystem turret) {
    this.swerveDrive = swerve.getSwerveDrive();
    this.turret = turret;

    // Load the current season's AprilTag field layout that ships with WPILib.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Front camera -> robot pose. Turret camera -> turret pose only (its estimator is unused;
    // solveTurretCamera() does its own geometry).
    addCamera(VisionConstants.kCameraLeftName, VisionConstants.kRobotToCameraLeft, false);
    addCamera(VisionConstants.kCameraRightName, VisionConstants.kTurretToCamera, true);
  }

  /**
   * Sets up a single camera and its pose estimator.
   *
   * @param name          the PhotonVision nickname of the camera
   * @param robotToCamera where the camera is mounted relative to robot center
   */
  private void addCamera(String name, Transform3d robotToCamera, boolean onTurret) {
    PhotonCamera camera = new PhotonCamera(name);

    // MULTI_TAG_PNP_ON_COPROCESSOR is the best strategy: when the camera sees 2+ tags,
    // the coprocessor triangulates one highly-accurate pose. If it only sees ONE tag,
    // it can't triangulate, so we fall back to LOWEST_AMBIGUITY (trust the clearest tag).
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    cameras.add(new CameraUnit(name, camera, estimator, onTurret));
  }

  /**
   * This runs every 20ms. For each camera we pull any new frames, turn them into
   * a robot position, sanity-check it, and feed the good ones to the pose estimator.
   */
  @Override
  public void periodic() {
    int acceptedThisLoop = 0;
    double now = Timer.getFPGATimestamp();
    rawYawHistory.addSample(now, rawYaw());
    updateTravelled(now);

    for (CameraUnit unit : cameras) {
      // getAllUnreadResults() returns every frame that arrived since we last checked.
      // Processing all of them (instead of just the newest) keeps timestamps honest.
      List<PhotonPipelineResult> results = unit.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : results) {
        if (unit.onTurret) {
          updateTurretFix(unit, result);
        } else if (addRobotPoseMeasurement(unit, result)) {
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
      SmartDashboard.putNumber("Vision/AgeSeconds", now - lastVisionTimestamp);
    }

    getTurretFieldPose().ifPresent(turretPosePublisher::set);
    SmartDashboard.putBoolean("Vision/Turret Fix Known", turretFixPosition != null);
    SmartDashboard.putNumber("Vision/Turret Fix Age (s)", turretFixPosition != null ? now - lastTurretFixTimestamp : -1.0);
  }

  /** Raw Pigeon yaw (see {@link #rawYawHistory}). */
  private Rotation2d rawYaw() {
    return swerveDrive.getGyro().getRawRotation3d().toRotation2d();
  }

  /** Robot field heading now: the camera-derived heading at the fix plus the gyro turn since. */
  private Rotation2d robotHeadingNow() {
    return robotHeadingAtFix.plus(rawYaw().minus(rawYawAtFix));
  }

  /** Add up how far the robot has driven since the last turret fix. */
  private void updateTravelled(double now) {
    if (turretFixPosition != null && lastTravelTimestamp >= 0.0) {
      ChassisSpeeds v = swerveDrive.getRobotVelocity();
      double dt = now - lastTravelTimestamp;
      travelledSinceFix = travelledSinceFix.plus(
          new Translation2d(v.vxMetersPerSecond * dt, v.vyMetersPerSecond * dt).rotateBy(robotHeadingNow()));
    }
    lastTravelTimestamp = now;
  }

  /** Front camera: PhotonVision's 3D robot pose into the swerve pose estimator. */
  private boolean addRobotPoseMeasurement(CameraUnit unit, PhotonPipelineResult result) {
    String statusKey = "Vision/" + unit.name + "/Status";

    Optional<EstimatedRobotPose> maybeEstimate = unit.estimator.update(result);
    if (maybeEstimate.isEmpty()) {
      SmartDashboard.putString(statusKey, "no usable tags");
      return false;
    }

    Pose2d visionPose = maybeEstimate.get().estimatedPose.toPose2d();
    unit.posePublisher.set(visionPose);
    int tagCount = result.getTargets().size();
    double avgDistance = averageTagDistanceMeters(result);

    // Throw out estimates from tags that are too far away — they get jittery
    // and can yank the robot's position around.
    if (avgDistance > VisionConstants.kMaxAverageTagDistanceMeters) {
      SmartDashboard.putString(statusKey, String.format("rejected: tags too far (%.1f m)", avgDistance));
      return false;
    }
    if (tagCount == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity) {
      SmartDashboard.putString(statusKey, "rejected: ambiguous single tag");
      return false;
    }

    // Hand the position to the swerve pose estimator. The stdDevs tell it how
    // much to trust us versus the wheel odometry.
    swerveDrive.addVisionMeasurement(visionPose, result.getTimestampSeconds(), computeStdDevs(tagCount, avgDistance));
    SmartDashboard.putString(statusKey, "accepted");

    lastVisionPose = visionPose;
    lastVisionTimestamp = result.getTimestampSeconds();
    return true;
  }

  /** Turret camera: solve the turret's field pose and store it as the new turret fix. */
  private void updateTurretFix(CameraUnit unit, PhotonPipelineResult result) {
    String statusKey = "Vision/" + unit.name + "/Status";

    Optional<Pose2d> maybeTurretPose = solveTurretCamera(unit, result, statusKey);
    if (maybeTurretPose.isEmpty()) {
      return; // solveTurretCamera() already set the status
    }
    Pose2d turretPose = maybeTurretPose.get();

    // The robot's field heading, from the camera: which way the turret faces on the field minus
    // which way it faces on the robot (encoder) at that same moment.
    double timestamp = result.getTimestampSeconds();
    Rotation2d turretOnRobot = TurretSubsystem.headingForAngle(turret.getAngleAt(timestamp));
    turretFixPosition = turretPose.getTranslation();
    robotHeadingAtFix = turretPose.getRotation().minus(turretOnRobot);
    rawYawAtFix = rawYawHistory.getSample(timestamp).orElse(rawYaw());
    travelledSinceFix = Translation2d.kZero;
    lastTurretFixTimestamp = timestamp;

    SmartDashboard.putNumber("Vision/TurretSolve/Turret Field Heading (deg)", turretPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/TurretSolve/Turret Heading On Robot (deg)", turretOnRobot.getDegrees());
    SmartDashboard.putNumber("Vision/TurretSolve/Robot Heading From Camera (deg)", robotHeadingAtFix.getDegrees());
    SmartDashboard.putString(statusKey, "turret fix");
  }

  /**
   * Where the turret's rotation axis is on the field, from the turret camera. Between frames
   * (or with no tag in view) the last fix is carried along by how far the robot has driven and
   * turned since. Never uses or changes the robot pose.
   *
   * @return empty until the turret camera has seen a tag once
   */
  public Optional<Translation2d> getTurretFieldPosition() {
    if (turretFixPosition == null) {
      return Optional.empty();
    }
    // If the turret axis is off robot center, turning the robot swings it around the center.
    Translation2d swing = TurretConstants.kRobotToTurret.rotateBy(robotHeadingNow())
        .minus(TurretConstants.kRobotToTurret.rotateBy(robotHeadingAtFix));
    return Optional.of(turretFixPosition.plus(travelledSinceFix).plus(swing));
  }

  /**
   * The robot's field heading as the turret camera sees it: camera-derived at the last fix,
   * plus how far the gyro has turned since. Independent of the gyro's zero / alliance flip.
   *
   * @return empty until the turret camera has seen a tag once
   */
  public Optional<Rotation2d> getRobotHeadingFromTurretCamera() {
    return turretFixPosition == null ? Optional.empty() : Optional.of(robotHeadingNow());
  }

  /** Turret position plus which way the shooter faces on the field right now. */
  public Optional<Pose2d> getTurretFieldPose() {
    return getTurretFieldPosition().map(position -> new Pose2d(position,
        robotHeadingNow().plus(TurretSubsystem.headingForAngle(turret.getAngle()))));
  }

  /**
   * Turret camera: the turret's field pose from PhotonVision's 3D tag solve, camera only. Uses
   * the coprocessor multi-tag pose when 2+ tags are seen, else the clearest single tag. Needs the
   * camera calibrated with 3D enabled in PhotonVision.
   *
   * @return the turret axis position, facing the way the shooter pointed in that frame
   */
  private Optional<Pose2d> solveTurretCamera(CameraUnit unit, PhotonPipelineResult result, String statusKey) {
    if (!result.hasTargets()) {
      SmartDashboard.putString(statusKey, "no tags");
      return Optional.empty();
    }

    Pose3d fieldToCamera;
    double distance;
    double singleTagAmbiguity = 0.0; // stays 0 for multi-tag, which can't flip
    if (result.getMultiTagResult().isPresent()) {
      // Same math as PhotonPoseEstimator's coprocessor multi-tag.
      Transform3d fieldToCameraTf = result.getMultiTagResult().get().estimatedPose.best;
      fieldToCamera = Pose3d.kZero.plus(fieldToCameraTf).relativeTo(fieldLayout.getOrigin());
      distance = averageTagDistanceMeters(result);
      SmartDashboard.putNumber("Vision/TurretSolve/Tags Used", result.getMultiTagResult().get().fiducialIDsUsed.size());
    } else {
      PhotonTrackedTarget best = null;
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getPoseAmbiguity() >= 0.0
            && fieldLayout.getTagPose(target.getFiducialId()).isPresent()
            && (best == null || target.getPoseAmbiguity() < best.getPoseAmbiguity())) {
          best = target;
        }
      }
      if (best == null || best.getBestCameraToTarget().getTranslation().getNorm() < 1e-6) {
        SmartDashboard.putString(statusKey, "no 3D pose: turn on 3D mode for this pipeline in PhotonVision");
        return Optional.empty();
      }
      fieldToCamera = fieldLayout.getTagPose(best.getFiducialId()).get()
          .transformBy(best.getBestCameraToTarget().inverse());
      distance = best.getBestCameraToTarget().getTranslation().getNorm();
      singleTagAmbiguity = best.getPoseAmbiguity();
      SmartDashboard.putNumber("Vision/TurretSolve/Tags Used", 1);
      SmartDashboard.putNumber("Vision/TurretSolve/Tag ID", best.getFiducialId());
      SmartDashboard.putNumber("Vision/TurretSolve/Ambiguity", best.getPoseAmbiguity());
    }
    SmartDashboard.putNumber("Vision/TurretSolve/Distance (m)", distance);

    // Step back from the camera to the turret axis. The turret frame is level with x along the
    // shooter, so its 2D pose is the turret position + shooter direction on the field.
    Pose2d turretPose = fieldToCamera.transformBy(VisionConstants.kTurretToCamera.inverse()).toPose2d();
    // Published BEFORE the checks below, so a rejected solve can still be seen in AdvantageScope.
    unit.posePublisher.set(turretPose);

    // Throw out solves from far-away tags — they get jittery.
    if (distance > VisionConstants.kMaxAverageTagDistanceMeters) {
      SmartDashboard.putString(statusKey, String.format("rejected: tags too far (%.1f m)", distance));
      return Optional.empty();
    }
    // A single tag seen nearly head-on can flip to a mirror-image solution. Multi-tag (two hub
    // tags on one face, with multi-target on in PhotonVision) doesn't have this problem.
    if (singleTagAmbiguity > VisionConstants.kMaxSingleTagAmbiguity) {
      SmartDashboard.putString(statusKey, "rejected: ambiguous single tag (turn on multi-target in PhotonVision)");
      return Optional.empty();
    }

    return Optional.of(turretPose);
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