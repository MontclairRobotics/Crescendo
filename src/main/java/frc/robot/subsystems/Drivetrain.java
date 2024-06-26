package frc.robot.subsystems;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

public class Drivetrain extends SubsystemBase {

  private ChassisSpeeds speeds;
  private final SwerveDrive swerveDrive;
  public final PIDController thetaController;
  Timer timer = new Timer();

  SwerveModule[] modules;
  Orchestra orchestra;

 private boolean isFieldRelative;

  // private AHRS navX;

  public Drivetrain(File directory) {

    this.isFieldRelative = true;

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

    timer.start();
    // })
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA);
    swerveDrive.replaceSwerveModuleFeedforward(ff);


    double kp = swerveDrive.getSwerveController().thetaController.getP();
    double ki = swerveDrive.getSwerveController().thetaController.getI();
    double kd = swerveDrive.getSwerveController().thetaController.getD();
    thetaController = new PIDController(kp, ki, kd);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    thetaController.setTolerance(DriveConstants.ANGLE_DEADBAND * ((Math.PI ) / 180 ), DriveConstants.VELOCITY_DEADBAND * ((Math.PI ) / 180 ));
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    // Shuffleboard.getTab("Debug").addDouble("Drivetrain/FrontLeftVoltage", getSwerveDrive().getModules()[0].getDriveMotor()::getVoltage);
    modules = swerveDrive.getModules();

    for (int i = 0; i < modules.length; i++) {
      modules[i].getAngleMotor().setMotorBrake(true);
    }
    // ArrayList<TalonFX> motors = new ArrayList<TalonFX>();
    // motors.add((TalonFX) modules[0].getDriveMotor().getMotor());
    // motors.add((TalonFX) modules[1].getDriveMotor().getMotor());
    // motors.add((TalonFX) modules[2].getDriveMotor().getMotor());
    // motors.add((TalonFX) modules[3].getDriveMotor().getMotor());
    // orchestra = new Orchestra();
    // orchestra.addInstrument(motors.get(0));
    // orchestra.addInstrument(motors.get(1));
    // orchestra.addInstrument(motors.get(2));
    // orchestra.addInstrument(motors.get(3));
    // orchestra.loadMusic("nationGood.chrp");


    // DriveConstants.kp.whenUpdate(getSwerveDrive().getSwerveController().thetaController::setP);
    // DriveConstants.kd.whenUpdate(getSwerveDrive().getSwerveController().thetaController::setD);
    // DriveConstants.ki.whenUpdate(getSwerveDrive().getSwerveController().thetaController::setI);

    // Shuffleboard.getTab("Debug").addDouble("Gyroscope Angle", () -> {
    //   return getSwerveDrive().getOdometryHeading().getDegrees();
    // });

    
    // Shuffleboard.getTab("Debug").addDouble("Wrapped Angle", () -> RobotContainer.drivetrain.getWrappedRotation().getDegrees());
    // Shuffleboard.getTab("Debug").addDouble("Front Left Velocity", () -> {
    //   return motors.get(0).getVelocity().getValueAsDouble();
    // });
    // Shuffleboard.getTab("Debug").addDouble("Front Right Velocity", () -> {
    //   return motors.get(1).getVelocity().getValueAsDouble();
    // });
    // Shuffleboard.getTab("Debug").addDouble("Back Left Velocity", () -> {
    //   return motors.get(2).getVelocity().getValueAsDouble();
    // });
    // Shuffleboard.getTab("Debug").addDouble("Back Right Velocity", () -> {
    //   return motors.get(3).getVelocity().getValueAsDouble();
    // });

    
  }

  public double calculateVelocityForHeading(Rotation2d targetHeading) {
    return thetaController.calculate(getWrappedRotation().getRadians(), targetHeading.getRadians());
  }

  public static boolean angleDeadband(Rotation2d angle1, Rotation2d angle2, Rotation2d deadband) {
    double degrees1 = wrapRotation(angle1).getDegrees();
    double degrees2 = wrapRotation(angle2).getDegrees();
    double deadbandDeg = wrapRotation(deadband).getDegrees();

    return Math.abs(degrees1 - degrees2) < deadbandDeg || Math.abs(degrees1 - degrees2) > 360 - deadbandDeg;
  }

  public static Rotation2d flipAngle(Rotation2d angle) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return GeometryUtil.flipFieldRotation(angle);
    }
    return angle;
  }

  public static Rotation2d wrapRotation(Rotation2d rot) {
    double degrees = rot.getDegrees() % 360;
    if (degrees < 0) {
      degrees += 360;
    }
    return Rotation2d.fromDegrees(degrees);
  }

  public void playMusic() {
    System.out.println("MUUUUSIC");
    orchestra.play();
  }

  public void stopMusic() {
    orchestra.stop();
  }

  /** It drives with certain linear velocities with a certain rotational velocity */
  public void drive(Translation2d translation, double rotation) {
    
    swerveDrive.drive(translation, rotation, this.isFieldRelative, DriveConstants.IS_OPEN_LOOP);
  }

  /** Moves chassis */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {

    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /** logs data: module positions, gyro rotation, and pose */
  @Override
  public void periodic() {
    // Logger.recordOutput("Drivetrain/Module-Positions", getSwerveDrive().getModulePositions());
    // Logger.recordOutput("Drivetrain/Gyro-Rotation", getSwerveDrive().getGyroRotation3d());
    // Logger.recordOutput("Drivetrain/Pose", getSwerveDrive().getPose());
    
    // if (timer.get() >= 0.4) {
      // System.out.println("FL " + modules[0].getDriveMotor().getVelocity());
      // System.out.println("FR " + modules[1].getDriveMotor().getVelocity());
      // System.out.println("BL " + modules[2].getDriveMotor().getVelocity());
      // System.out.println("FR " + modules[3].getDriveMotor().getVelocity());
      // timer.reset();
      // timer.start();
    // } 

    // Pose2d pose = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_wpiBlue("limelight-shooter"));
    // System.out.println(Units.metersToInches(pose.minus(new Pose2d(0, 5.55, new Rotation2d())).getTranslation().getNorm()));
    // System.out.println(Units.metersToInches(LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace("limelight-shooter")).getTranslation().getNorm()));

    // RobotContainer.field.setRobotPose(swerveDrive.getPose());
    // System.out.println(swerveDrive.getPose().getX() + " " + swerveDrive.getPose().getY() + " " + swerveDrive.getPose().getRotation().getDegrees());
  }

  public void addVisionMeasurement(Pose2d pose, double time) {
    swerveDrive.addVisionMeasurement(pose, time);
  }
  public void addVisionMeasurement(Pose2d pose, double time, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(pose, time, visionMeasurementStdDevs);
  }

  /** sets isFieldRelative to either true or false, used for getIsFieldRelative */
  public void setIsFieldRelative(boolean relative) {
    this.isFieldRelative = relative;
  }

  
  public boolean getIsFieldRelative(boolean relative) {
    return this.isFieldRelative;
  }

  public void zeroGyro() {
    this.swerveDrive.zeroGyro();
  }

  public SwerveDrive getSwerveDrive() {
    return this.swerveDrive;
  }

  /** Resets the odometer */
  public void resetOdometry() {
    this.swerveDrive.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  

  // @AutoLogOutput
  /** returns direction */
  public Rotation2d getRotation() {

    return this.swerveDrive.getOdometryHeading();
  }

  public void enableFieldRelative() {
    System.out.println("Enabled field relative!");
    isFieldRelative = true;
  }

  public void disableFieldRelative() {
    System.out.println("Disabled field relative!");
    isFieldRelative = false;
  }

  /** Returns angle of the robot between 0 and 360 */
  public Rotation2d getWrappedRotation() {
    double angle = getRotation().getDegrees() % 360;
    if (angle < 0) angle = 360 + angle;
    return Rotation2d.fromDegrees(angle);
  }

  public void setInputFromController(CommandPS5Controller controller) {

    double thetaSpeed =
        -MathUtil.applyDeadband(controller.getRightX(), 0.05) * DriveConstants.MAX_ROT_SPEED;

    double xSpeed = -MathUtil.applyDeadband(controller.getLeftX(), 0.05) * DriveConstants.MAX_SPEED;
    double ySpeed = -MathUtil.applyDeadband(controller.getLeftY(), 0.05) * DriveConstants.MAX_SPEED;

    // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //   thetaSpeed *= -1;
    //   xSpeed *= -1;
    //   ySpeed *= -1;
    // }
    Translation2d targetTranslation = new Translation2d(ySpeed, xSpeed);
    // Logger.recordOutput("Drivetrain/Controller-Translation", targetTranslation);
    // Logger.recordOutput("Drivetrain/Controller-Theta", thetaSpeed);

    this.drive(targetTranslation, thetaSpeed);
  }

  public SysIdRoutine getSysIdAngle() {

        MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
        MutableMeasure<Angle> rotations = MutableMeasure.mutable(Rotations.of(0));
        MutableMeasure<Velocity<Angle>> motorVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));

        SwerveModule[] modules = swerveDrive.getModules();
        CANSparkMax frontLeft = (CANSparkMax) modules[0].getAngleMotor().getMotor();
        CANSparkMax frontRight = (CANSparkMax) modules[1].getAngleMotor().getMotor();
        CANSparkMax backLeft = (CANSparkMax) modules[2].getAngleMotor().getMotor();
        CANSparkMax backRight = (CANSparkMax) modules[3].getAngleMotor().getMotor();

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
                backLeft.setVoltage(volts.in(Volts));
                backRight.setVoltage(volts.in(Volts));
            }, 
            (SysIdRoutineLog log) -> {
                log.motor("front-left-steer")
                .voltage(appliedVoltage.mut_replace(frontLeft.getAppliedOutput() * frontLeft.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(frontLeft.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(frontLeft.getEncoder().getPosition(), Rotations));

                log.motor("front-right-steer")
                .voltage(appliedVoltage.mut_replace(frontRight.getAppliedOutput() * frontRight.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(frontRight.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(frontRight.getEncoder().getPosition(), Rotations));

                log.motor("back-left-steer")
                .voltage(appliedVoltage.mut_replace(backLeft.getAppliedOutput() * backLeft.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(backLeft.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(backLeft.getEncoder().getPosition(), Rotations));

                log.motor("back-right-steer")
                .voltage(appliedVoltage.mut_replace(backRight.getAppliedOutput() * backRight.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(backRight.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(backRight.getEncoder().getPosition(), Rotations));
            },
            this
            )
        );

        return routine;
    }



    public Command getSysIdCommand() {
      SysIdRoutine routine = SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12);
      return SwerveDriveTest.generateSysIdCommand(routine, 5, 2, 2);
    }

    public void driveOneMeter() {
      TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(4, 3);

      ProfiledPIDController snapController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);
	    ProfiledPIDController xController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);
	    ProfiledPIDController yController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);

      xController.setGoal(new TrapezoidProfile.State(1, 0));
		  yController.setGoal(new TrapezoidProfile.State(1, 0));
		  snapController.setGoal(new TrapezoidProfile.State(Math.toRadians(0), 0.0));

        // getPose() should return the robot's position on the field in meters, probably from odometry
        // getYaw180 just returns the reading from the gyro
		  double xAdjustment = xController.calculate(swerveDrive.getPose().getY());
		  double yAdjustment = -yController.calculate(swerveDrive.getPose().getX());
      double angleAdjustment = snapController.calculate(Math.toRadians(0));

      swerveDrive.drive(new Translation2d(xAdjustment, yAdjustment), angleAdjustment, true, true, new Translation2d());

    }
    
    public SysIdRoutine getSysIdDrive() {
        MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
        MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
        MutableMeasure<Velocity<Distance>> linearVelocity = MutableMeasure.mutable(MetersPerSecond.of(0));


        SwerveModule[] modules = swerveDrive.getModules();
        CANSparkMax frontLeft = (CANSparkMax) modules[0].getDriveMotor().getMotor();
        CANSparkMax frontRight = (CANSparkMax) modules[1].getDriveMotor().getMotor();
        CANSparkMax backLeft = (CANSparkMax) modules[2].getDriveMotor().getMotor();
        CANSparkMax backRight = (CANSparkMax) modules[3].getDriveMotor().getMotor();

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
                backLeft.setVoltage(volts.in(Volts));
                backRight.setVoltage(volts.in(Volts));
            }, 
            (SysIdRoutineLog log) -> {
                log.motor("front-left-drive")
                .voltage(appliedVoltage.mut_replace(frontLeft.getAppliedOutput() * frontLeft.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(frontLeft.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(frontLeft.getEncoder().getPosition() * 1 /* TODO ticks to meters */, Meters));

                log.motor("front-right-drive")
                .voltage(appliedVoltage.mut_replace(frontRight.getAppliedOutput() * frontRight.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(frontRight.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(frontRight.getEncoder().getPosition(), Meters));

                log.motor("back-left-drive")
                .voltage(appliedVoltage.mut_replace(backLeft.getAppliedOutput() * backLeft.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(backLeft.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(backLeft.getEncoder().getPosition(), Meters));

                log.motor("back-right-drive")
                .voltage(appliedVoltage.mut_replace(backRight.getAppliedOutput() * backRight.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(backRight.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(backRight.getEncoder().getPosition(), Meters));
            },
            this
            )
        );
        
        return routine;
    }

    public Rotation2d getHeadingForSpeaker() { // Ripped straight out of the cold dead hands of Thomas
      
      Pose2d targetPose;
      Pose2d currentPose = getSwerveDrive().getPose();
      LimelightHelpers.PoseEstimate visionPose = RobotContainer.shooterLimelight.getAdjustedPose();

      if (visionPose.tagCount > 1) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          targetPose = FieldConstants.BLUE_SPEAKER_POSE;
        } else {
          targetPose = FieldConstants.RED_SPEAKER_POSE;
        }
        return currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

      } else {
        return Rotation2d.fromDegrees(-RobotContainer.shooterLimelight.getObjectTX());
      }
      

      
    
    }
    

    public double getDistanceToSpeaker() { // Straight out of the guts of the robowarriors
      Pose2d currentPose = getSwerveDrive().getPose();
      Pose2d targetPose;
      LimelightHelpers.PoseEstimate visionPose = RobotContainer.shooterLimelight.getAdjustedPose();

      if (visionPose.tagCount > 1) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          targetPose = FieldConstants.BLUE_SPEAKER_POSE;
        } else {
          targetPose = FieldConstants.RED_SPEAKER_POSE;
        }

        return Units.metersToInches(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
      

      } else {
        return RobotContainer.shooterLimelight.getDistanceToSpeaker();
      }
     

      
      
    }


  
}
