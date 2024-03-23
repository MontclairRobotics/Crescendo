package frc.robot.vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

public class Limelight extends SubsystemBase {
  private double lastTx = 0;

  private String cameraName;
  private DetectionType defaultPipe;
  private Debouncer targetDebouncer =
      new Debouncer(VisionConstants.TARGET_DEBOUNCE_TIME, DebounceType.kFalling);

  public Limelight(String cameraName, DetectionType defaultPipe) {
    this.defaultPipe = defaultPipe;
    double[] camerapose_robotspace = new double[] {-1, -1, -1, 0.0, 0.0, 0.0};
    LimelightHelpers.setLimelightNTDoubleArray(
        cameraName, "camerapose_robotspace", camerapose_robotspace);

    this.cameraName = cameraName;
  }

  @AutoLogOutput
  public double getTimestampSeconds() {
    double latency =
        (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
                + LimelightHelpers.getLimelightNTDouble(cameraName, "tl"))
            / 1000.0;

    return Timer.getFPGATimestamp() - latency;
  }

  @AutoLogOutput
  public boolean hasValidTarget() {
    boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
    return targetDebouncer.calculate(hasMatch);
  }

  public boolean currentPipelineMatches(DetectionType type) {

    int pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
    return type.getPipe() == pipeline;
  }

  public double getObjectXSafe() {
    if (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") != 1) {
      return lastTx;
    } else {
      lastTx = getObjectTX();
      return getObjectTX();
    }
  }

  public void setPipelineTo(DetectionType type) {
    if (type == DetectionType.DRIVER) {
      LimelightHelpers.setCameraMode_Driver(cameraName);
    } else { LimelightHelpers.setCameraMode_Processor(cameraName);
    }

    LimelightHelpers.setPipelineIndex(cameraName, type.getPipe());
  }

  @AutoLogOutput
  public double getObjectTX() {
    return LimelightHelpers.getLimelightNTDouble(cameraName, "tx");
  }

  @AutoLogOutput
  public double getObjectTY() {
    return LimelightHelpers.getLimelightNTDouble(cameraName, "ty");
  }

  public double getPipeline() {
    return LimelightHelpers.getLimelightNTDouble(cameraName, "getpipe");
  }

  public Pose2d getBotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(cameraName);
  }

  public DetectionType getPipelineType() {
    if (LimelightHelpers.getLimelightNTDouble(cameraName, "camMode") == 1) {
      return DetectionType.DRIVER;
    }
    return DetectionType.getType((int) getPipeline());
  }

  public Pose2d getTargetPoseRobotSpace() { //TODO did I screw this up?
    double[] doubleArr =  LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    return new Pose2d(doubleArr[0], doubleArr[1], Rotation2d.fromDegrees(doubleArr[5]));
  }


  public void setPriorityTagID(int id) {
    LimelightHelpers.setPriorityTagID(cameraName, id);
  }
  

  public double getDistanceToSpeaker() {

    double distance =
        (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.SHOOTER_LIMELIGHT_HEIGHT)
            / Math.tan(
                (Math.PI / 180.0)
                    * (VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES + getObjectTY()));

    return distance/Math.cos(getObjectTX() * (Math.PI / 180));
  }


  // public double getAngleForSpeaker() {
  //   double distance = getDistanceToSpeaker() + 14.5 - 6.0; // in the middle of speaker, not the edge.
  //   double targetHeight = VisionConstants.SPEAKER_GOAL_HEIGHT - 4.5;

  //   double angle = Math.atan((targetHeight / distance));

  //   return angle * (180.0 / Math.PI);

  // }

  

  public void setDefaultPipeline() {
    // if (DriverStation.isTeleop()) {
    //   setPipelineTo(DetectionType.DRIVER);
    // } else {
    //   setPipelineTo(defaultPipe);
    // }
    setPipelineTo(defaultPipe);
  }

  @Override
  public void periodic() {

    // if (getPipelineType() == DetectionType.APRIL_TAG && hasValidTarget()) { //TODO test this TEST THIS
    //   LimelightHelpers.PoseEstimate targetPose = getAdjustedPose();
    //   if (targetPose.tagCount >= 2) {
    //     RobotContainer.drivetrain.addVisionMeasurement(
    //       targetPose.pose,
    //       targetPose.timestampSeconds
    //     );
    //   }
    // }

    // if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        LimelightHelpers.setPriorityTagID(cameraName, 4); //4
      } else {
        LimelightHelpers.setPriorityTagID(cameraName, 7);
      }
    // }

  }

  public double bestFit() {
    double x = getDistanceToSpeaker();
    // return (0.001717 * (Math.pow(x, 2))) + (-0.6251 * x) + (83.41);
    return (78.3*Math.exp(-0.0177*x)) + 25.04;
  }

  public double getSpeedForSpeaker() {
    return ShooterConstants.SPEAKER_EJECT_SPEED - 20 * Math.abs(90-RobotContainer.drivetrain.getWrappedRotation().getDegrees());
  }

  public LimelightHelpers.PoseEstimate getAdjustedPose() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && DriverStation.isTeleop()) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
  }

  public boolean isAligned() {
    double tx = getObjectTX();
    return Drivetrain.angleDeadband(Rotation2d.fromDegrees(tx), RobotContainer.drivetrain.getWrappedRotation(), Drivetrain.wrapRotation(Rotation2d.fromDegrees(DriveConstants.ANGLE_DEADBAND)));
  }

  public boolean isAlignedAuto() {
    return Math.abs(getObjectTX()) < DriveConstants.ANGLE_DEADBAND;
  }

  

}

