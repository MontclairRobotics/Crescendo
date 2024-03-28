package frc.robot.command;

import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;


/**
 * A command to use Pose Esimation to align the robot to a note that it is chasing
 * This command should be run in parallel to a path finder path that is heading towards a note
 */
public class AutoPoseEstimateToNote extends Command {

  // Locations of the center line notes on the field
  private static final HashMap<Character, Translation2d> noteLocations = new HashMap<>();
  {
    noteLocations.put('D', new Translation2d(8.3, 0.752856));
    noteLocations.put('E', new Translation2d(8.3, 2.429256));
    noteLocations.put('F', new Translation2d(8.3, 4.105656));
    noteLocations.put('G', new Translation2d(8.3, 5.782056));
    noteLocations.put('H', new Translation2d(8.3, 7.458456));
  }

  // The note we are chasing  
  private final char note;
  // Location of note we are chasing
  private final Translation2d noteLocation;
  // Limelight
  private final Limelight limelight;
  // Name of camera
  private final String cameraName;
  // Timestamp of last limelight vision measurement
  private double lastLimelightTimestamp = 0;

  // Use to not log every frame
  private int logCount = 0;

  // KLUDGE!!!
  // These fields are replicating logic in PoseEstimator
  // Ideally we could get data directly from their but it's a private field
  // copying here (see more KLUDGE notes below)
  private SwerveDriveKinematics kinematics;
  private static final double kBufferDuration = 1.5;
  private final TimeInterpolatableBuffer<InterpolationRecord> m_poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDuration);

  /**
   * Create the command
   * 
   * @param kinematics KLUDGE
   * @param note The note we are chasing
   */
  public AutoPoseEstimateToNote(SwerveDriveKinematics kinematics, char note) {
    super();
    this.kinematics = kinematics;
    this.note = note;
    this.noteLocation = noteLocations.get(note);
    this.limelight = RobotContainer.intakeLimelight;
    // TODO: we need this to use LimelightHelpers, but really those functions should just be moved to LimeLight class
    this.cameraName = "limelight-intake";
  }

  /**
   * Log a message periodically
   * @param msg
   */
  private void log(String msg) {
    if (logCount == 0) {
      System.out.println(msg);
    }
  }


  /**
   * Get the robot pose at a past timestamp
   * KLUDGE
   */
  private Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
    // If time is too far in past, no data
    try {
      if (m_poseBuffer.getInternalBuffer().lastKey() - kBufferDuration > timestampSeconds) {
        return Optional.empty();
      }
    } catch (NoSuchElementException ex) {
      return Optional.empty();
    }

    // Get the sample from that time and return it
    var sample = m_poseBuffer.getSample(timestampSeconds);
    if (sample.isEmpty()) {
      return Optional.empty();
    }  
    return Optional.of(sample.get().poseMeters);
  }

  /**
   * This code calculates the location that the limelight thinks the robot is on the field 
   * by getting the angle to the note we are chasing and the distance to that note
   * It then uses that location for PoseEstimator
   */
  public void execute() {
    if (logCount++ > 0) {
      logCount = 0;
    }

    // KLUDGE
    // This is annoying
    // We are recreating what is happening in PoseEstimator because we don't have
    // acccess to the pose buffer in there.
    m_poseBuffer.addSample(
      Timer.getFPGATimestamp(),
      new InterpolationRecord(RobotContainer.drivetrain.getSwerveDrive().getPose(), RobotContainer.drivetrain.getSwerveDrive().getYaw(), new SwerveDriveWheelPositions(RobotContainer.drivetrain.getSwerveDrive().getModulePositions()))
    );
    
    if (limelight.getPipelineType() == DetectionType.NOTE && limelight.hasValidTarget()) {
      // Angle to note from camera
      double limelightAngle = limelight.getObjectTX();

      // Get timestamp of when note was detected
      // This is last time the entry in network tables was updated minus latency of limelight
      var poseEntry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "tx");
      double limelightLatency = (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
        + LimelightHelpers.getLimelightNTDouble(cameraName, "tl"));
      // getlastchange() in microseconds, ll latency in milliseconds
      double limelightTimestamp = (poseEntry.getLastChange() / 1000000.0) - (limelightLatency/1000.0);

      // Only update if we have a new measurement from limelight
      // Saves processing time since limelight updates slower than this periodic call
      if (limelightTimestamp != lastLimelightTimestamp) {
        lastLimelightTimestamp = limelightTimestamp;

        // Get the pose of the robot at the time the limelight made it's observation
        Optional<Pose2d> robotPoseOption = getPoseAtTimestamp(lastLimelightTimestamp);

        // If we don't have a pose from that time, ignore
        if (!robotPoseOption.isEmpty()) {
          Pose2d robotPose = robotPoseOption.get();
          log("robotX: " + robotPose.getX());

          // Distance from robot to note
          // To get the true location the limelight thinks the robot is at, 
          // we'd calculate distance use ty from limelight, instead of this
          // But this is really close, and honestly probably more accurate
          // This could be wrong, but if odemetry is so far off that this is not working,
          // we are probably too far off for this alignment system to help anyway :)
          double distanceToNote = noteLocation.getDistance(robotPose.getTranslation());
          log("distanceToNote: " + distanceToNote);

          // Only use this if we are within 2.5 meters of the note
          // Otherwise we will get confused by close notes
          // If we end up using this for the close notes, this should be a different number for those
          // TODO: Is this a good distance?
          if (distanceToNote < 2.5) {
            // Get the field relative angle from robot to note according to limelight
            // This works because 0 rotation is always toward center of field
            // TODO: Is that correct?
            log("limelightAngle: " + limelightAngle);
            log("robotAngle: " + robotPose.getRotation().getDegrees());
            double limelightFieldRelativeAngle = robotPose.getRotation().getDegrees() + limelightAngle;

            // Get the location the the limelight thinks the robot is in
            // This is not location minus distance to note and the angle to note
            Translation2d limelightLocation = noteLocation.minus(new Translation2d(distanceToNote, Rotation2d.fromDegrees(limelightFieldRelativeAngle)));

            // Only use this measurement if we are within a 25 centimeters of where we thing we are
            // TODO: What is a good value to use here???
            // TODO: Or should we just let the PoseEstimate std deviations take care of this?
            double distanceBetweenPoses = limelightLocation.getDistance(robotPose.getTranslation());
            log("robotLocation: " + robotPose.getX() + ", " + robotPose.getY());
            log("limelightLocation: " + limelightLocation.getX() + ", " + limelightLocation.getY());
            log("distanceBetweenPoses: " + distanceBetweenPoses);
            if (distanceBetweenPoses < 0.25) {
              // Average limelight location with robot's location to smooth this out
              // Note sure this is needed, but adding this because I'm worried that limelight detection of note isn't all that accurate
              // Commented this out, since pose estimate basically does this already
              // Translation2d averagedLocation = limelightLocation.plus(robotPose.getTranslation()).div(2);
              // log("averagedLocation: " + averagedLocation.getX() + ", " + averagedLocation.getY());

              // Use same rotation as robot to get a pose from limelight Location
              // TODO: Should we be using averaged location or just limelight location????
              Pose2d limelightPose = new Pose2d(limelightLocation, robotPose.getRotation());

              // Add this new position as a vision measurement
              Pose2d originalPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
              log("originalPose: " + originalPose.getX() + ", " + originalPose.getY());
              Logger.recordOutput("Drivetrain/NotePose", limelightPose);
              RobotContainer.drivetrain.getSwerveDrive().addVisionMeasurement(limelightPose, limelightTimestamp);
              Pose2d newPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
              log("newPose: " + newPose.getX() + ", " + newPose.getY());
            }
          }
        }
      }
    }
  }

  /**
   * Command ends when a note is in the transport
   * This command should probably be used with a timeout in case it never gets the note
   */
  public boolean isFinished() {
    return RobotContainer.shooter.isNoteInTransport();
  }  







  /**
   * KLUDGE
   * This is a straight copy from PoseEstimator, it interpolates between two samples in time
   * 
   * Represents an odometry record. The record contains the inputs provided as well as the pose that
   * was observed based on these inputs, as well as the previous record and its inputs.
   */
  private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
    // The pose observed given the current sensor inputs and the previous pose.
    private final Pose2d poseMeters;

    // The current gyro angle.
    private final Rotation2d gyroAngle;

    // The current encoder readings.
    private final SwerveDriveWheelPositions wheelPositions;

    /**
     * Constructs an Interpolation Record with the specified parameters.
     *
     * @param poseMeters The pose observed given the current sensor inputs and the previous pose.
     * @param gyro The current gyro angle.
     * @param wheelPositions The current encoder readings.
     */
    private InterpolationRecord(Pose2d poseMeters, Rotation2d gyro, SwerveDriveWheelPositions wheelPositions) {
      this.poseMeters = poseMeters;
      this.gyroAngle = gyro;
      this.wheelPositions = wheelPositions;
    }

    /**
     * Return the interpolated record. This object is assumed to be the starting position, or lower
     * bound.
     *
     * @param endValue The upper bound, or end.
     * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
     * @return The interpolated value.
     */
    @Override
    public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
      if (t < 0) {
        return this;
      } else if (t >= 1) {
        return endValue;
      } else {
        // Find the new wheel distances.
        var wheelLerp = wheelPositions.interpolate(endValue.wheelPositions, t);

        // Find the new gyro angle.
        var gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);

        // Create a twist to represent the change based on the interpolated sensor inputs.
        Twist2d twist = kinematics.toTwist2d(wheelPositions, wheelLerp);
        twist.dtheta = gyroLerp.minus(gyroAngle).getRadians();

        return new InterpolationRecord(poseMeters.exp(twist), gyroLerp, wheelLerp);
      }
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (!(obj instanceof AutoPoseEstimateToNote.InterpolationRecord)) {
        return false;
      }
      var record = (AutoPoseEstimateToNote.InterpolationRecord) obj;
      return Objects.equals(gyroAngle, record.gyroAngle)
          && Objects.equals(wheelPositions, record.wheelPositions)
          && Objects.equals(poseMeters, record.poseMeters);
    }

    @Override
    public int hashCode() {
      return Objects.hash(gyroAngle, wheelPositions, poseMeters);
    }
  }
}
