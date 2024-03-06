package frc.robot.vision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
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
    return LimelightHelpers.getBotPose2d(cameraName);
  }

  public DetectionType getPipelineType() {
    if (LimelightHelpers.getLimelightNTDouble(cameraName, "camMode") == 1) {
      return DetectionType.DRIVER;
    }
    return DetectionType.getType((int) getPipeline());
  }


  public double getDistanceToSpeaker() {

    double distance =
        (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.SHOOTER_LIMELIGHT_HEIGHT)
            / Math.tan(
                (Math.PI / 180.0)
                    * (VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES + getObjectTY()));

    return distance;
  }

  public void setDefaultPipeline() {
    if (DriverStation.isTeleop()) {
      setPipelineTo(DetectionType.DRIVER);
    } else {
      setPipelineTo(defaultPipe);
    }
  }

  @Override
  public void periodic() {
    if (getPipelineType() == DetectionType.APRIL_TAG && hasValidTarget()) {
      double[] targetArr = LimelightHelpers.getBotPose(cameraName);
      RobotContainer.drivetrain.addVisionMeasurement(
        new Pose2d(targetArr[0], targetArr[1], Rotation2d.fromDegrees(targetArr[5])),
        targetArr[6]
      );
    }

  }

  

}
