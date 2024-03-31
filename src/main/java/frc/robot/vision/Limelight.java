package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.util.GeometryUtil;

public class Limelight extends SubsystemBase {
  private double lastTx = 0;

  private String cameraName;
  private DetectionType defaultPipe;
  private Debouncer targetDebouncer =
      new Debouncer(VisionConstants.TARGET_DEBOUNCE_TIME, DebounceType.kFalling);
  private double priorityId = 0;

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
    return LimelightHelpers.getTX(cameraName);
  }

  @AutoLogOutput
  public double getObjectTY() {
    return LimelightHelpers.getTY(cameraName);
  }

  public double getPipeline() {
    return LimelightHelpers.getCurrentPipelineIndex(cameraName);
  }

  public Pose2d getBotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(cameraName);
  }

  public Pose2d funBotPose() {
    return LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(cameraName));
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

  //returns the component of distance in a straight wall from the lens of the limelight to the wall
  public double getStraightDistanceToSpeaker() {
    double distance =
        (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.SHOOTER_LIMELIGHT_HEIGHT)
            / Math.tan(
                (Math.PI / 180.0)
                    * (VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES + getObjectTY()));

    return distance;

  }


  //returns adjusted heading to aim for the actual target instead of the april tag
  public Rotation2d maxIsStupid() {
    double distance = getStraightDistanceToSpeaker();
    double gyroHeading = RobotContainer.drivetrain.getWrappedRotation().getRadians();
    double offsetFromTag = 6.0;

    double angle = Math.asin((offsetFromTag * Math.sin(gyroHeading)) / (Math.sqrt(Math.pow(distance,2) + Math.pow(offsetFromTag,2) - 2 * distance * offsetFromTag * Math.cos(gyroHeading))));

    return Rotation2d.fromRadians(angle);

  }



  

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
    
    
    // TODO: LOOK INTO THIS PLEASE TYSM <3
    if (getPipelineType() == DetectionType.APRIL_TAG && hasValidTarget()) { //TODO test this TEST THIS
      LimelightHelpers.PoseEstimate targetPose = getAdjustedPose();
      if (targetPose.tagCount >= 2) {
        RobotContainer.drivetrain.addVisionMeasurement(
          targetPose.pose,
          targetPose.timestampSeconds,
          VisionConstants.VISION_STD_DEVS
        );
        
      }
    }

    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        LimelightHelpers.setPriorityTagID(cameraName, 4); //4
        priorityId = 4;
      } else {
        LimelightHelpers.setPriorityTagID(cameraName, 7);
        priorityId = 7;
      }
    }

  }

  //degrees
  public double getAngleToSpeaker() {
    LimelightHelpers.PoseEstimate botPose = getAdjustedPose();
    Pose2d odomPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
    // if (botPose.tagCount < 2 || botPose.avgTagDist > 4) {
    LimelightHelpers.RawFiducial[] tagArr = botPose.rawFiducials;
    tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);
    if (tagArr.length == 1) {
      return -tagArr[0].txnc + RobotContainer.shooterLimelight.maxIsStupid().getDegrees();
      // }
    }

    double verticalDistance = 5.55-odomPose.getY();
    double horizontalDistance = odomPose.getX() - 0.1524; //6 inches in meters

    if (DriverStation.isAutonomous()) {
      horizontalDistance = 16.54 - odomPose.getX();
    }

    double rotation = 180 + (180 / Math.PI) * Math.atan(verticalDistance/horizontalDistance); //180 + because we are on the back of robot

    if (DriverStation.isAutonomous()) {
      rotation = 180 - rotation;
    }
    return rotation;
    


    

  }

    public double getPoseDistanceToSpeaker() {
    LimelightHelpers.PoseEstimate botPose = getAdjustedPose();
    Pose2d odomPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
    // if (botPose.tagCount < 2 || botPose.avgTagDist > 4) {
    LimelightHelpers.RawFiducial[] tagArr = botPose.rawFiducials;
    tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);
    if (tagArr.length == 1) {
      getDistanceToSpeaker();
      // }
    }

    double verticalDistance = 5.55-odomPose.getY();
    double horizontalDistance = odomPose.getX() - 0.1524; //6 inches in meters

    if (DriverStation.isAutonomous()) {
      horizontalDistance = 16.54 - odomPose.getX();
    }

    double distance = Math.sqrt(Math.pow(verticalDistance, 2) + Math.pow(horizontalDistance, 2));

    return distance;
    


    

  }

  public double bestFit() {
    // double  
    // return (0.001717 * (Math.pow(x, 2))) + (-0.6251 * x) + (83.41);
    return bestFitFromDistance(getDistanceToSpeaker());
  }

  public double bestFitFromDistance(double x) {
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

