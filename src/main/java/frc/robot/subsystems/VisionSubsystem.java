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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShootOnMoveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.SelectHub;
import swervelib.SwerveDrive;

/**
 * VisionSubsystem is the robot's "eyes." It uses PhotonVision running on a
 * coprocessor (e.g. an Orange Pi) to spot AprilTags on the field and figure out
 * exactly where the robot is standing.
 *
 * <p>This replaces the old Limelight-based vision. It runs TWO cameras with separate jobs:
 * <ul>
 *   <li>FRONT camera (chassis-fixed): robot localization. Its poses go into the swerve drive's
 *       pose estimator, which blends them with wheel/gyro data for PathPlanner. Each accepted
 *       frame's own solve also corrects the turret fix (below).</li>
 *   <li>TURRET camera: finds where the TURRET is on the field and which way it faces, from
 *       the camera alone, for aiming only. It never touches the robot pose; see
 *       {@link #getTurretFieldPosition()}.</li>
 * </ul>
 *
 * <p>CROSS-CHECK: before a frame updates the turret fix (or, for the front camera, the robot
 * pose), it is compared with the other camera's latest frame. Three voters: front camera, turret
 * camera, and the gyro + wheel track. If the cameras agree, the frame is used (and if they both
 * disagree with the track, the track is reset to them). If they disagree, the camera closer to the
 * track wins and the other's frame is thrown out. See {@link #crossCheck}.
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
  // Turret-camera frames rejected in a row for disagreeing with the tracked heading.
  private int turretFramesRejectedInARow = 0;
  // How far the robot has driven since the fix (wheel speeds, turned by the heading above).
  private Translation2d travelledSinceFix = Translation2d.kZero;
  private double lastTravelTimestamp = -1.0;
  private final StructPublisher<Pose2d> turretPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Turret/FieldPose", Pose2d.struct).publish();

  // Chassis yaw rate from the raw Pigeon yaw, rad/s CCW-positive, lightly filtered.
  private final LinearFilter yawRateFilter = LinearFilter.singlePoleIIR(0.04, 0.02);
  private Rotation2d lastRawYaw = null;
  private double lastYawRateTimestamp = -1.0;
  private double yawRateRadPerSec = 0.0;

  // Pure gyro + wheel dead-reckoning of the turret axis in the raw-gyro frame. Never corrected by
  // a camera, so its short-term motion carries one camera's frame to another frame's time.
  private final TimeInterpolatableBuffer<Pose2d> rawOdomHistory = TimeInterpolatableBuffer.createBuffer(1.0);
  private Pose2d rawOdom = Pose2d.kZero;
  // The turret track (getTurretFieldPosition + robotHeadingNow) by FPGA time, to judge frames by.
  private final TimeInterpolatableBuffer<Pose2d> trackHistory = TimeInterpolatableBuffer.createBuffer(1.0);

  // Ball time of flight (s) by turret-to-hub distance (m), for shoot on the move.
  private final InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

  /**
   * One camera's candidate turret fix: turret axis position and robot field heading (as a Pose2d)
   * at the frame time. onAccept runs only if it survives the cross-check.
   */
  private record TurretCandidate(CameraUnit unit, Pose2d pose, double timestamp, String statusKey, Runnable onAccept) {}

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
    // This camera's latest candidate turret fix (cross-check), and how far it was from the track
    // (cross-check score units, see trackError). Null until the first one.
    TurretCandidate lastCandidate = null;
    double lastCandidateTrackError = 0.0;
    // How many cross-check votes this camera has lost.
    int crossCheckLosses = 0;

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

    for (double[] point : ShootOnMoveConstants.kTimeOfFlight) {
      timeOfFlight.put(point[0], point[1]);
    }
    // Saved on the roboRIO: created with these defaults once, then keep whatever was set.
    Preferences.initBoolean(ShootOnMoveConstants.kEnabledKey, true);
    Preferences.initDouble(ShootOnMoveConstants.kLeadGainKey, 1.0);
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
    int[] acceptedThisLoop = {0};
    double now = Timer.getFPGATimestamp();
    rawYawHistory.addSample(now, rawYaw());
    updateYawRate(now);
    updateRawOdom(now);
    updateTravelled(now);
    if (turretFixPosition != null) {
      trackHistory.addSample(now, new Pose2d(getTurretFieldPosition().get(), robotHeadingNow()));
    }

    // Collect every camera's candidate turret fix first, then cross-check and apply them in
    // frame-time order, so each frame is judged against the other camera's latest one.
    List<TurretCandidate> candidates = new ArrayList<>();
    for (CameraUnit unit : cameras) {
      // getAllUnreadResults() returns every frame that arrived since we last checked.
      // Processing all of them (instead of just the newest) keeps timestamps honest.
      List<PhotonPipelineResult> results = unit.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : results) {
        Optional<TurretCandidate> candidate = unit.onTurret
            ? turretCameraCandidate(unit, result)
            : frontCameraCandidate(unit, result, () -> acceptedThisLoop[0]++);
        candidate.ifPresent(candidates::add);
      }

      // Publish per-camera targeting data from the newest frame we received this loop.
      // (Last element of getAllUnreadResults() is the most recent.)
      if (!results.isEmpty()) {
        PhotonPipelineResult newest = results.get(results.size() - 1);
        unit.latestTargets = newest.getTargets();
        publishTargetingData(unit.name, newest);
      }
    }

    candidates.sort((a, b) -> Double.compare(a.timestamp(), b.timestamp()));
    for (TurretCandidate candidate : candidates) {
      Optional<Boolean> verdict = crossCheck(candidate);
      if (verdict.isPresent()) {
        candidate.onAccept().run();
        applyTurretFix(candidate.pose().getTranslation(), candidate.pose().getRotation(),
            candidate.timestamp(), candidate.statusKey(), verdict.get());
      }
    }

    // Overall vision health so drivers/programmers can see vision is alive.
    SmartDashboard.putBoolean("Vision/HasTarget", acceptedThisLoop[0] > 0);
    SmartDashboard.putNumber("Vision/AcceptedMeasurements", acceptedThisLoop[0]);
    if (lastVisionPose != null) {
      SmartDashboard.putNumber("Vision/LastX", lastVisionPose.getX());
      SmartDashboard.putNumber("Vision/LastY", lastVisionPose.getY());
      SmartDashboard.putNumber("Vision/AgeSeconds", now - lastVisionTimestamp);
    }

    getTurretFieldPose().ifPresent(turretPosePublisher::set);
    SmartDashboard.putBoolean("Vision/Turret Fix Known", turretFixPosition != null);
    SmartDashboard.putNumber("Vision/Turret Fix Age (s)", turretFixPosition != null ? now - lastTurretFixTimestamp : -1.0);
    for (CameraUnit unit : cameras) {
      SmartDashboard.putNumber("Vision/CrossCheck/" + unit.name + " Losses", unit.crossCheckLosses);
    }
    publishShootOnMove();
  }

  /** Differentiate the raw Pigeon yaw for the chassis yaw rate. */
  private void updateYawRate(double now) {
    Rotation2d yaw = rawYaw();
    if (lastRawYaw != null && now > lastYawRateTimestamp) {
      // minus() wraps to +/-180, so crossing the +/-180 seam doesn't spike the rate.
      double rate = yaw.minus(lastRawYaw).getRadians() / (now - lastYawRateTimestamp);
      yawRateRadPerSec = yawRateFilter.calculate(rate);
    }
    lastRawYaw = yaw;
    lastYawRateTimestamp = now;
  }

  /** Chassis yaw rate, rad/s CCW-positive, from the Pigeon. */
  public double getYawRateRadPerSec() {
    return yawRateRadPerSec;
  }

  /** Turret axis velocity in the ROBOT frame (m/s): wheel speeds plus the swing of an off-center turret. */
  private Translation2d turretRobotRelativeVelocity() {
    ChassisSpeeds v = swerveDrive.getRobotVelocity();
    Translation2d r = TurretConstants.kRobotToTurret;
    return new Translation2d(v.vxMetersPerSecond - yawRateRadPerSec * r.getY(),
        v.vyMetersPerSecond + yawRateRadPerSec * r.getX());
  }

  /** Dead-reckon the turret axis in the raw-gyro frame (see {@link #rawOdomHistory}). */
  private void updateRawOdom(double now) {
    Rotation2d yaw = rawYaw();
    double dt = lastTravelTimestamp >= 0.0 ? now - lastTravelTimestamp : 0.0;
    Translation2d step = turretRobotRelativeVelocity().times(dt).rotateBy(yaw);
    rawOdom = new Pose2d(rawOdom.getTranslation().plus(step), yaw);
    rawOdomHistory.addSample(now, rawOdom);
  }

  /** Turret axis velocity on the FIELD (m/s), or zero until a camera has seen a tag once. */
  public Translation2d getTurretFieldVelocity() {
    if (turretFixPosition == null) {
      return Translation2d.kZero;
    }
    return turretRobotRelativeVelocity().rotateBy(robotHeadingNow());
  }

  /**
   * Vector from the turret axis to where it should aim (meters, field frame). With shoot on the
   * move enabled, that is a virtual hub shifted against the turret's velocity by the ball's flight
   * time, so the robot's motion carries the ball into the real hub. Its length drives the shot map
   * and its direction drives the turret.
   *
   * @return empty until a camera has seen a tag once
   */
  public Optional<Translation2d> getAimVector() {
    Optional<Translation2d> maybeTurretPosition = getTurretFieldPosition();
    if (maybeTurretPosition.isEmpty()) {
      return Optional.empty();
    }
    Translation2d turretPosition = maybeTurretPosition.get();
    Translation2d hub = SelectHub.hubPosition(new Pose2d(turretPosition, Rotation2d.kZero));
    Translation2d target = hub;
    if (Preferences.getBoolean(ShootOnMoveConstants.kEnabledKey, true)) {
      Translation2d velocity = getTurretFieldVelocity();
      double leadGain = Preferences.getDouble(ShootOnMoveConstants.kLeadGainKey, 1.0);
      // The flight time depends on the distance, which depends on the lead: a few rounds settle it.
      for (int i = 0; i < 3; i++) {
        double flightTime = timeOfFlight.get(target.minus(turretPosition).getNorm());
        target = hub.minus(velocity.times(flightTime * leadGain));
      }
    }
    return Optional.of(target.minus(turretPosition));
  }

  private void publishShootOnMove() {
    Translation2d velocity = getTurretFieldVelocity();
    SmartDashboard.putNumber("ShootOnMove/Velocity X (mps)", velocity.getX());
    SmartDashboard.putNumber("ShootOnMove/Velocity Y (mps)", velocity.getY());
    SmartDashboard.putNumber("ShootOnMove/Yaw Rate (deg per s)", Math.toDegrees(yawRateRadPerSec));
    getTurretFieldPosition().ifPresent(turretPosition -> {
      Translation2d toHub = SelectHub.hubPosition(new Pose2d(turretPosition, Rotation2d.kZero)).minus(turretPosition);
      Translation2d toAim = getAimVector().get();
      SmartDashboard.putNumber("ShootOnMove/Lead (m)", toAim.minus(toHub).getNorm());
      SmartDashboard.putNumber("ShootOnMove/TOF (s)", timeOfFlight.get(toAim.getNorm()));
    });
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

  /**
   * Front camera: PhotonVision's 3D robot pose, as a turret-fix candidate. Only if it survives the
   * cross-check does it go into the swerve pose estimator (onAccept).
   */
  private Optional<TurretCandidate> frontCameraCandidate(CameraUnit unit, PhotonPipelineResult result, Runnable countAccepted) {
    String statusKey = "Vision/" + unit.name + "/Status";

    Optional<EstimatedRobotPose> maybeEstimate = unit.estimator.update(result);
    if (maybeEstimate.isEmpty()) {
      SmartDashboard.putString(statusKey, "no usable tags");
      return Optional.empty();
    }

    Pose2d visionPose = maybeEstimate.get().estimatedPose.toPose2d();
    unit.posePublisher.set(visionPose);
    int tagCount = result.getTargets().size();
    double avgDistance = averageTagDistanceMeters(result);

    // Throw out estimates from tags that are too far away — they get jittery
    // and can yank the robot's position around.
    if (avgDistance > VisionConstants.kMaxAverageTagDistanceMeters) {
      SmartDashboard.putString(statusKey, String.format("rejected: tags too far (%.1f m)", avgDistance));
      return Optional.empty();
    }
    if (tagCount == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity) {
      SmartDashboard.putString(statusKey, "rejected: ambiguous single tag");
      return Optional.empty();
    }

    double timestamp = result.getTimestampSeconds();
    Runnable onAccept = () -> {
      // Hand the position to the swerve pose estimator. The stdDevs tell it how
      // much to trust us versus the wheel odometry.
      swerveDrive.addVisionMeasurement(visionPose, timestamp, computeStdDevs(tagCount, avgDistance));
      SmartDashboard.putString(statusKey, "accepted");
      lastVisionPose = visionPose;
      lastVisionTimestamp = timestamp;
      countAccepted.run();
    };

    // The same camera-only pose also corrects the turret fix. This camera is fixed to the chassis,
    // so it keeps seeing tags even when a bad turret fix has the turret pointed away from them.
    // Uses this frame's own solve, never the fused odometry pose.
    Translation2d turretPosition = visionPose.getTranslation()
        .plus(TurretConstants.kRobotToTurret.rotateBy(visionPose.getRotation()));
    return Optional.of(new TurretCandidate(unit, new Pose2d(turretPosition, visionPose.getRotation()),
        timestamp, "Vision/" + unit.name + "/Turret Fix Status", onAccept));
  }

  /** Turret camera: solve the turret's field pose as a turret-fix candidate. */
  private Optional<TurretCandidate> turretCameraCandidate(CameraUnit unit, PhotonPipelineResult result) {
    String statusKey = "Vision/" + unit.name + "/Status";

    Optional<Pose2d> maybeTurretPose = solveTurretCamera(unit, result, statusKey);
    if (maybeTurretPose.isEmpty()) {
      return Optional.empty(); // solveTurretCamera() already set the status
    }
    Pose2d turretPose = maybeTurretPose.get();

    // The robot's field heading, from the camera: which way the turret faces on the field minus
    // which way it faces on the robot (encoder) at that same moment.
    double timestamp = result.getTimestampSeconds();
    Rotation2d turretOnRobot = TurretSubsystem.headingForAngle(turret.getAngleAt(timestamp));
    Rotation2d cameraHeading = turretPose.getRotation().minus(turretOnRobot);
    SmartDashboard.putNumber("Vision/TurretSolve/Turret Field Heading (deg)", turretPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/TurretSolve/Turret Heading On Robot (deg)", turretOnRobot.getDegrees());
    SmartDashboard.putNumber("Vision/TurretSolve/Robot Heading From Camera (deg)", cameraHeading.getDegrees());

    return Optional.of(new TurretCandidate(unit, new Pose2d(turretPose.getTranslation(), cameraHeading),
        timestamp, statusKey, () -> {}));
  }

  /**
   * How far a candidate is from the track at its frame time, in cross-check units (1.0 = at the
   * agreement limit in position or heading). Zero with no track yet.
   */
  private double trackError(TurretCandidate candidate) {
    Optional<Pose2d> track = trackHistory.getSample(candidate.timestamp());
    return track.map(t -> poseError(candidate.pose(), t)).orElse(0.0);
  }

  /** Difference between two (turret position, robot heading) poses in cross-check units. */
  private static double poseError(Pose2d a, Pose2d b) {
    return Math.max(a.getTranslation().getDistance(b.getTranslation()) / VisionConstants.kCrossCheckMaxMeters,
        Math.abs(a.getRotation().minus(b.getRotation()).getDegrees()) / VisionConstants.kCrossCheckMaxDegrees);
  }

  /**
   * 2-of-3 vote between this candidate, the other camera's latest candidate, and the gyro + wheel
   * track. The other camera's frame is carried to this frame's time by the uncorrected dead-reckoning
   * ({@link #rawOdomHistory}), so robot motion between the two frames cancels out.
   *
   * <p>A CONSTANT "Vision/CrossCheck/Offset" means a mounting transform is wrong
   * (kRobotToCameraLeft, kTurretToCamera, or kRobotToTurret), not that a camera is noisy.
   *
   * @return empty to reject the candidate; true to force a turret fix reset to it (both cameras
   *     agree but the track doesn't); false to apply it normally
   */
  private Optional<Boolean> crossCheck(TurretCandidate candidate) {
    CameraUnit unit = candidate.unit();
    double candidateTrackError = trackError(candidate);
    TurretCandidate other = null;
    CameraUnit otherUnit = null;
    for (CameraUnit u : cameras) {
      if (u != unit && u.lastCandidate != null
          && Math.abs(candidate.timestamp() - u.lastCandidate.timestamp()) <= VisionConstants.kCrossCheckWindowSeconds) {
        other = u.lastCandidate;
        otherUnit = u;
      }
    }
    double otherTrackError = otherUnit != null ? otherUnit.lastCandidateTrackError : 0.0;
    unit.lastCandidate = candidate;
    unit.lastCandidateTrackError = candidateTrackError;

    if (other == null) {
      // Nothing recent from the other camera: single-camera rules in applyTurretFix().
      return Optional.of(false);
    }

    Optional<Pose2d> rawAtCandidate = rawOdomHistory.getSample(candidate.timestamp());
    Optional<Pose2d> rawAtOther = rawOdomHistory.getSample(other.timestamp());
    if (rawAtCandidate.isEmpty() || rawAtOther.isEmpty()) {
      return Optional.of(false);
    }
    Pose2d otherNow = other.pose().plus(rawAtCandidate.get().minus(rawAtOther.get()));

    // Offset is always turret camera minus front camera.
    boolean candidateOnTurret = unit.onTurret;
    Pose2d turretCam = candidateOnTurret ? candidate.pose() : otherNow;
    Pose2d frontCam = candidateOnTurret ? otherNow : candidate.pose();
    boolean agree = poseError(candidate.pose(), otherNow) <= 1.0;
    SmartDashboard.putNumber("Vision/CrossCheck/Offset X (m)", turretCam.getX() - frontCam.getX());
    SmartDashboard.putNumber("Vision/CrossCheck/Offset Y (m)", turretCam.getY() - frontCam.getY());
    SmartDashboard.putNumber("Vision/CrossCheck/Offset Heading (deg)",
        turretCam.getRotation().minus(frontCam.getRotation()).getDegrees());
    SmartDashboard.putBoolean("Vision/CrossCheck/Agree", agree);

    if (agree) {
      // Two cameras outvote the track: if the track is off by more than the heading-jump limit,
      // reset it now instead of rejecting frames until kTurretResyncFrames.
      boolean trackIsOff = turretFixPosition != null && trackHistory.getSample(candidate.timestamp())
          .map(t -> Math.abs(candidate.pose().getRotation().minus(t.getRotation()).getDegrees())
              > VisionConstants.kTurretMaxHeadingJumpDegrees)
          .orElse(false);
      return Optional.of(trackIsOff);
    }

    // Disagree: the camera closer to the track wins.
    if (turretFixPosition == null || candidateTrackError <= otherTrackError) {
      otherUnit.crossCheckLosses++;
      return Optional.of(false);
    }
    unit.crossCheckLosses++;
    SmartDashboard.putString(candidate.statusKey(), "rejected: disagrees with " + otherUnit.name);
    // The front camera's frame is also kept out of the robot pose.
    SmartDashboard.putString("Vision/" + unit.name + "/Status", "rejected: disagrees with " + otherUnit.name);
    return Optional.empty();
  }

  /**
   * Take one camera's turret position and robot field heading (from either camera) as the new
   * turret fix, or blend it in, or reject it as a heading jump.
   */
  private void applyTurretFix(Translation2d turretPosition, Rotation2d cameraHeading, double timestamp, String statusKey,
      boolean forceReset) {
    // The two cameras' frames can arrive out of order. An older frame than the current fix
    // would rewind the fix time, so skip it.
    if (turretFixPosition != null && timestamp < lastTurretFixTimestamp) {
      SmartDashboard.putString(statusKey, "skipped: older than current turret fix");
      return;
    }
    Rotation2d rawYawAtFrame = rawYawHistory.getSample(timestamp).orElse(rawYaw());

    if (turretFixPosition == null
        || forceReset
        || turretFramesRejectedInARow >= VisionConstants.kTurretResyncFrames
        || timestamp - lastTurretFixTimestamp > VisionConstants.kTurretFixStaleSeconds) {
      // First fix, the last fix is stale, both cameras agree against the track, or the camera
      // has disagreed long enough that the tracked heading is the wrong one: take it outright.
      turretFixPosition = turretPosition;
      robotHeadingAtFix = cameraHeading;
      SmartDashboard.putString(statusKey, "turret fix (reset)");
    } else {
      // Where the gyro and wheels say we are, at this frame's time.
      Rotation2d trackedHeading = robotHeadingAtFix.plus(rawYawAtFrame.minus(rawYawAtFix));
      double headingJumpDeg = cameraHeading.minus(trackedHeading).getDegrees();
      SmartDashboard.putNumber("Vision/TurretSolve/Heading Jump (deg)", headingJumpDeg);
      if (Math.abs(headingJumpDeg) > VisionConstants.kTurretMaxHeadingJumpDegrees) {
        // A single tag's mirror-flip or a noisy solve. Keep the tracked fix.
        turretFramesRejectedInARow++;
        SmartDashboard.putString(statusKey, String.format("rejected: heading jump %.0f deg", headingJumpDeg));
        return;
      }
      // Agrees with the gyro: nudge toward the camera instead of snapping to it.
      Translation2d trackedPosition = getTurretFieldPosition().get();
      robotHeadingAtFix = trackedHeading.interpolate(cameraHeading, VisionConstants.kTurretHeadingBlend);
      turretFixPosition = trackedPosition.interpolate(turretPosition, VisionConstants.kTurretPositionBlend);
      SmartDashboard.putString(statusKey, "turret fix");
    }
    turretFramesRejectedInARow = 0;
    rawYawAtFix = rawYawAtFrame;
    travelledSinceFix = Translation2d.kZero;
    lastTurretFixTimestamp = timestamp;
  }

  /**
   * Where the turret's rotation axis is on the field, from the camera tag solves (turret camera,
   * or the front camera's pose shifted by kRobotToTurret; never the fused odometry). Between frames
   * (or with no tag in view) the last fix is carried along by how far the robot has driven and
   * turned since. Never uses or changes the robot pose.
   *
   * @return empty until a camera has seen a tag once
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
   * The robot's field heading from the camera tag solves: camera-derived at the last fix,
   * plus how far the gyro has turned since. Independent of the gyro's zero / alliance flip.
   *
   * @return empty until a camera has seen a tag once
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
    if (singleTagAmbiguity > VisionConstants.kTurretMaxTagAmbiguity) {
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

  /**
   * Distance for the shot map: turret to the (shoot-on-move virtual) hub, minus 0.54 m because
   * the map was calibrated to the tag, not the hub center. 0.0 until a camera has seen a tag once.
   */
  public double getLiveDistanceToHub() {
    return getAimVector().map(toHub -> toHub.getNorm() - 0.54).orElse(0.0);
  }
}