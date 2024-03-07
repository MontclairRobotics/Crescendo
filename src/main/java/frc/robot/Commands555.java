package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import animation2.CelebrationAnimation;
import animation2.FlashAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;

public class Commands555 {
  private static double initTurnAngle;

  /**
   * Drive to a robot-relative point given a Translation2d & target Rotation2d.
   *
   * @param targetTranslation Field-relative Translation2d to drive the robot to.
   * @param theta             Target angle for end position.
   */
  public static Command driveToRobotRelativePoint(
      Translation2d targetTranslation, Rotation2d theta) {
    Pose2d currentRobotPosition = RobotContainer.drivetrain.getSwerveDrive().getPose();
    Rotation2d currentOdometryHeading = RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading();

    Translation2d targetTranslation2d = currentRobotPosition
        .getTranslation()
        .plus(targetTranslation.rotateBy(currentOdometryHeading));
    Pose2d targetPose = new Pose2d(targetTranslation2d.getX(), targetTranslation2d.getY(), theta);

    return AutoBuilder.pathfindToPose(
        targetPose,
        AutoConstants.PATH_CONSTRAINTS,
        AutoConstants.GOAL_END_VELOCITY,
        AutoConstants.ROTATION_DELAY_DISTANCE);
  }

  // Zero gyro
  public static Command zeroGyro() {
    return Commands.runOnce(RobotContainer.drivetrain.getSwerveDrive()::zeroGyro);
  }

  public static Command lockDrive() {
    return Commands.runOnce(() -> {
      RobotContainer.drivetrain.getSwerveDrive().lockPose();
    }, RobotContainer.drivetrain);
  }

  /**
   * Drive to a field-relative point given a targetPose
   *
   * @param targetPose field-relative pose2d to drive the robot to.
   */
  public static Command driveToFieldRelativePoint(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(
        targetPose,
        AutoConstants.PATH_CONSTRAINTS,
        AutoConstants.GOAL_END_VELOCITY,
        AutoConstants.ROTATION_DELAY_DISTANCE);
  }

  /*
   * - - - - - - - - - -
   * Intake Commands
   * - - - - - - - - - -
   */
  /**
   * runs the intake until the beam break sensor is broken, and then stops
   *
   * @return A
   */
  public static Command loadNote() {
    Command alignSprocket = Commands555.setSprocketAngle(ArmConstants.INTAKE_ANGLE);
    Command intakeAndTransport = Commands.sequence(alignSprocket, Commands.parallel(Commands555.intake(), Commands555.transport(ShooterConstants.TRANSPORT_SPEED)));
    return intakeAndTransport
        .withName("intake in")
        .until(() -> {
          return RobotContainer.shooter.isNoteInTransport();
        })
        .finallyDo(() -> {
          RobotContainer.intake.stop();
          RobotContainer.shooter.stopTransport();
        });

  }

  public static Command intake() {
    return Commands.run(RobotContainer.intake::in);
  }

  public static Command reverseIntake() {
    return Commands.runOnce(RobotContainer.intake::out, RobotContainer.intake)
        .withName("intake out");
  }

  public static Command stopIntake() {
    return Commands.runOnce(RobotContainer.intake::stop, RobotContainer.intake)
        .withName("intake stop");
  }

  public static Command ferryNote() {
    return Commands555.shoot(30, 30, 0.3, 0.2);
  }
  // /**
  // * Robot relative or field relative depending on isFieldRelative. Input angle
  // MUST be between 0
  // and 360 degrees
  // * @param angle
  // * @return a command
  // */

  /***
   *
   * @param rot       a field relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
   *
   */
  public static Command alignToAngleFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain)
        .until(
            () -> {
              return Math.abs(
                  RobotContainer.drivetrain.getWrappedRotation().getDegrees()
                      - rot.get().getDegrees()) < DriveConstants.ANGLE_DEADBAND;
            });
  }

  public static Command alignContinuousFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain);
  }

  /***
   *
   * @param rot       a robot relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
   *
   */
  public static Command alignToAngleRobotRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return Rotation2d.fromDegrees((initTurnAngle + rot.get().getDegrees()) % 360);
        },
        lockDrive)
        .beforeStarting(
            () -> {
              initTurnAngle = RobotContainer.drivetrain.getWrappedRotation().getDegrees();
            });
  }

  public static Command alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignContinuousFieldRelative(
        () -> {
          return Rotation2d.fromDegrees(
              (RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
        },
        lockDrive);
  }

  /**
   * @param angle     the target angle in field space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleFieldRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return angle;
        },
        lockDrive);
  }

  /**
   * @param angle     the target angle in robot space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleRobotRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleRobotRelative(
        () -> {
          return angle;
        },
        lockDrive);
  }

  /**
   * @param camera the limelight to use
   * @return a command that will align the robot to the target from the current
   *         limelight Will be
   *         canceled if the limelight loses its target
   */
  public static Command alignToLimelightTarget(Limelight camera, DetectionType targetType) {
    
    // Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
    return Commands.sequence(
      waitForPipe(camera, targetType),
      ifHasTarget(
          alignToAngleRobotRelativeContinuous(
              () -> {
                Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
                return targetAngle;
              },
              false),
          camera)
          .finallyDo(
              () -> {
                RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                camera.setDefaultPipeline();
              })
    ); // TODO should we lock drive?
  }

  public static Command scoreMode() {
    return Commands.parallel(
      alignToLimelightTarget(RobotContainer.shooterLimelight, DetectionType.APRIL_TAG),
      setSprocketAngle(RobotContainer.shooterLimelight::bestFit),
      Commands.runOnce(() -> {
        RobotContainer.isDriverMode = true;
        RobotContainer.shooter.shootActually(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
        // RobotContainer.shooter.transportStart(ShooterConstants.TRANSPORT_SPEED);
      })
    ).finallyDo(() -> {
      RobotContainer.shooter.stop();
      RobotContainer.isDriverMode = false;
    });
  }

  public static Command runTransportManual() {
    return Commands.run(() -> {
      RobotContainer.shooter.transportStart(ShooterConstants.TRANSPORT_SPEED);
    }).finallyDo(() -> {
      RobotContainer.shooter.stopTransport();
    });
  }

  // public static Command testPipeSwitch(Limelight camera, DetectionType pipe) {
  //   Timer timer = new Timer();
  //   return Commands.run(() -> {
  //     timer.reset();
  //     timer.start();
  //     camera.setPipelineTo(pipe);
  //     // System.out.println(Timer.getFPGATimestamp());
  //   }).until(() -> {return camera.getPipelineType() == pipe;}).finallyDo(() -> {
  //     System.out.println(timer.get());
  //     timer.stop();
  //   });
  // }
  /**
   * A command decorator to cancel a command if the limelight loses its target
   *
   * @param cmd   the command to be decorated
   * @param limey the limelight to be targeted
   * @return
   */
  public static Command ifHasTarget(Command cmd, Limelight limey) {
    return cmd.onlyWhile(() -> limey.hasValidTarget());
  }

  // TODO: Look over this
  /***
   * @param limey
   * @return A command that drives to the currently targeted april tag
   */
  // public static Command driveToAprilTag(Limelight limey) {
  // double[] aprilTagPoseArray =
  // LimelightHelpers.getLimelightNTDoubleArray(limey.getName(),
  // "targetpose_robotspace");
  // Pose2d aprilTagPose = new Pose2d(new Translation2d(aprilTagPoseArray[0],
  // aprilTagPoseArray[1]), new Rotation2d(aprilTagPoseArray[5]));
  // Pose2d targetPose =
  // aprilTagPose.relativeTo(DriveConstants.EDGE_OF_DRIVEBASE); //TODO does
  // this work the way I think it does
  // return driveToFieldRelativePoint(targetPose);

  // }

  /***
   * Enables field relative mode
   * 
   * @return a command that enables field relative control
   */
  public static Command enableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::enableFieldRelative);
  }

  /**
   * @return a command to disable field relative control
   */
  public static Command disableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::disableFieldRelative);
  }

  /*
   * - - - - - - - - - -
   * Sprocket Commands
   * - - - - - - - - - -
   */
  // public static Command goUp() {
  // return Commands.runOnce(RobotContainer.sprocket::goUp,
  // RobotContainer.sprocket).withName("sprocket up");
  // }

  // public static Command goDown() {
  // return Commands.runOnce(RobotContainer.sprocket::goDown,
  // RobotContainer.sprocket).withName("sprocket down");
  // }

  public static Command stopSprocket() {
    return Commands.runOnce(RobotContainer.sprocket::stop, RobotContainer.sprocket)
        .withName("sprocket stop");
  }

  /**
   * Angle in degrees
   *
   * @param angle target angle
   * @return Command that sets the target sprocket position to the given angle
   */
  public static Command setSprocketAngle(double angle) {
    System.out.println(angle);
    return Commands.runOnce(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle)), RobotContainer.sprocket);
  }

   public static Command setSprocketAngle(DoubleSupplier angle) {
    // System.out.println(angle);
    return Commands.run(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle.getAsDouble())), RobotContainer.sprocket);
  }

  /*
   * - - - - - - - - - -
   * Shooter Commands
   * - - - - - - - - - -
   */
  /**
   * 
   * @param topShootSpeed Top Motor (RPM)
   * @param bottomShootSpeed Bottom Motor (RPM)
   * @param transportSpeed Transport Motor (-1 / 1)
   * @return Command that waits for shooter to reach setpoint RPM, then starts transport with given speed
   */
  public static Command shoot(double topShootSpeed, double bottomShootSpeed, double transportSpeed, double rampUpTime) {

    return Commands.sequence(
        Commands.runOnce(() -> {
          RobotContainer.shooter.shootActually(topShootSpeed, bottomShootSpeed);
        }, RobotContainer.shooter),
        Commands555.waitForTime(rampUpTime),
        Commands555.transport(transportSpeed),
        log("Transported"),
        Commands555.waitForTime(0.5))
        .finallyDo(() -> {
          RobotContainer.shooter.stopTransport();
          RobotContainer.shooter.stopShooter();
        });
  }
  public static Command shoot(double speed, double transportSpeed) {
    return shoot(speed, speed, transportSpeed, 1);
  }

  public static Command log(String msg) {
    return Commands.runOnce(() -> {System.out.println(msg);});
  }

  public static Command setAutoPose(String autoString) {
    return Commands.runOnce(() -> {
      Pose2d startPose = new Pose2d();
      // if (RobotContainer.shooterLimelight.hasValidTarget() && RobotContainer.shooterLimelight.getPipelineType() == DetectionType.APRIL_TAG) {
      //   startPose = RobotContainer.shooterLimelight.getBotPose();
      //   RobotContainer.field.setRobotPose(startPose);
      // } else if (RobotContainer.intakeLimelight.hasValidTarget() && RobotContainer.intakeLimelight.getPipelineType() == DetectionType.APRIL_TAG) {
      //   startPose = RobotContainer.intakeLimelight.getBotPose();
      // } else {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          switch (autoString.charAt(0)) {
            case '1': startPose = AutoConstants.POSE_1_RED;
                      break;
            case '2': startPose = AutoConstants.POSE_2_RED;
                      break;
            case '3': startPose = AutoConstants.POSE_3_RED;
                      break;
            case '4': startPose = AutoConstants.POSE_4_RED;
                      break;
          }
        } else {
          switch (autoString.charAt(0)) {
            case '1': startPose = AutoConstants.POSE_1;
                      break;
            case '2': startPose = AutoConstants.POSE_2;
                      break;
            case '3': startPose = AutoConstants.POSE_3;
                      break;
            case '4': startPose = AutoConstants.POSE_4;
                      break;
          }
        // }
      }
      RobotContainer.drivetrain.getSwerveDrive().resetOdometry(startPose);
    });
  }

  public static Command stopTransport() {
    return Commands.runOnce(() -> {
      RobotContainer.shooter.stopTransport();
    }, RobotContainer.shooter);
  }

  public static Command shootAmp() {
    return Commands.runOnce(RobotContainer.shooter::shootAmp, RobotContainer.shooter)
        .withName("shoot amp");
  }

  public static Command stopShooter() {
    return Commands.runOnce(RobotContainer.shooter::stop, RobotContainer.shooter)
        .withName("shooter stop");
  }

  public static Command reverseShooter() {
    return Commands.runOnce(RobotContainer.shooter::reverseShooter, RobotContainer.shooter)
        .withName("shooter reverse");
  }

  public static Command shootVelocity(double velocity) {
    return Commands.runOnce(
        () -> {
          RobotContainer.shooter.shootVelocity(velocity);
        });
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        return condition.getAsBoolean();
      }
    };
  }

  public static Command waitForPipe(Limelight camera, DetectionType pipeline) {
    return Commands.run(() -> {}).beforeStarting(() -> {
      camera.setPipelineTo(pipeline);
    }).until(() -> {
      return camera.getPipelineType() == pipeline; //TODO need to check driver mode?
    });
  }

   /**
   * 
   * Used during auton for intaking notes using vision.
   * 
   * @return Command that aligns using intake limelight, sets sprocket to intake angle, and drives forward until either note is intaked or timeout is reached
   */
  public static Command autonomousIntake() {
    return Commands.sequence(
      Commands555.alignToLimelightTarget(RobotContainer.intakeLimelight, DetectionType.NOTE),
      new WaitUntilCommand(RobotContainer.sprocket::isAtAngle),
      Commands.parallel(
        Commands555.intake(),
        driveFromSpeeds(new ChassisSpeeds(AutoConstants.INTAKING_MOVE_SPEED, 0, 0))
      )
      .until(() ->
        RobotContainer.shooter.isNoteInTransport()
      )
      .withTimeout(AutoConstants.INTAKING_TIMEOUT)
      .finallyDo(() -> {
        RobotContainer.shooter.stopTransport();
        RobotContainer.intake.stop();
        RobotContainer.drivetrain.getSwerveDrive().drive(new ChassisSpeeds());
      })
    );    
  }

  public static Command driveFromSpeeds(ChassisSpeeds speeds) {
    return Commands.runOnce(() -> RobotContainer.drivetrain.getSwerveDrive().drive(speeds));
  }

  public static Command waitForTime(double seconds) {
    return Commands.run(() -> {
    }).withTimeout(seconds);
  }

  public static Command transport(double transportSpeed) {
    return Commands.runOnce(
        () -> {
          RobotContainer.shooter.transportStart(transportSpeed);
        });
  }

  public static Command scoreAmp() {
    return Commands.sequence(
        //setSprocketAngle(ArmConstants.AMP_SCORE_ANGLE),
        shoot(ShooterConstants.AMP_EJECT_SPEED, ShooterConstants.AMP_EJECT_SPEED, ShooterConstants.TRANSPORT_SPEED, 1),
        setSprocketAngle(ArmConstants.INTAKE_ANGLE));
        
  }

  //ONLY USE IN AUTO
  public static Command scoreSubwoofer() {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Shooting!")),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        waitUntil(() -> {return RobotContainer.sprocket.isAtAngle();}),
        shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED,
            ShooterConstants.TRANSPORT_SPEED, 0),
        Commands.runOnce(() -> System.out.println("Done Shooting!")),
        setSprocketAngle(ArmConstants.INTAKE_ANGLE));
  }

  public static Command receiveHumanPlayerNote() {
    return Commands.sequence(
        alignToLimelightTarget(RobotContainer.shooterLimelight, DetectionType.APRIL_TAG),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        reverseShooter(),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE));
  }

  // public static Command signalAmp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kOrange));
  // });
  // }

  // public static Command signalCoop() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlue));
  // });
  // }

  // // LED bits
  // public static Command celebrate() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new CelebrationAnimation());
  // });
  // }

  // public static Command ampItUp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kYellow));
  // });
  // }

  // public static Command cooperatition() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlueViolet));
  // });
  // }

  // ***********************CLIMBER COMMANDS*************************//
  public static Command climbersUp() {
    return Commands.runOnce(
        () -> {
          RobotContainer.climbers.up();
        });
  }

  public static Command climbersDown() {
    return Commands.runOnce(
        () -> {
          RobotContainer.climbers.down();
        });
  }

  public static Command climbersStop() {
    return Commands.runOnce(() -> {
      RobotContainer.climbers.stop();
    });
  }
}
