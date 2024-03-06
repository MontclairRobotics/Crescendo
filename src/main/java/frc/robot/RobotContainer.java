// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import animation2.AllianceAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import animation2.api.ConditionalAnimation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
import frc.robot.util.Tunable;
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;

public class RobotContainer {

  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static XboxController testController = new XboxController(2);
  public static Drivetrain drivetrain =
      new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));

  // Subsystems
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
  public static Limelight intakeLimelight = new Limelight("limelight-intake", DetectionType.APRIL_TAG);
  public static Limelight shooterLimelight = new Limelight("limelight-shooter", DetectionType.APRIL_TAG);
  public static Auto auto = new Auto();
  // public static LED led =
  //     new LED(
  //         new ConditionalAnimation(getTeleopDefaultAnim())
  //             .addCase(DriverStation::isDisabled, getDisabledAnimation()),
  //         new WipeTransition());
  public static Climbers climbers = new Climbers();
  

  public static final Field2d field = new Field2d();

  private Tunable<Double> topShooterSpeakerSpeed = Tunable.of(4000, "shooter/top-speaker-speed");
  private Tunable<Double> bottomShooterSpeakerSpeed =
      Tunable.of(4500, "shooter/bottom-speaker-speed"); // 4500 was most consistent in testing
  private Tunable<Double> transportSpeakerSpeed = Tunable.of(1000, "shooter/transport-speed");

 
  private Tunable<Double> angleSetpoint = Tunable.of(52, "Angle Setpoint");
  //public static Tunable<Double> speakerAngle = Tunable.of(50, "speaker Setpoint");
  

  private Tunable<Double> topShooterAmpSpeed = Tunable.of(1000, "shooter/top-amp-speed");
  private Tunable<Double> bottomShooterAmpSpeed = Tunable.of(1000, "shooter/bottom-amp-speed"); // 4500 was most consistent in testing
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    auto.setupPathPlanner();
    auto.setupAutoTab();
    setupDriverTab();
    

    drivetrain.setDefaultCommand(
        Commands.run(
            () -> {
              drivetrain.setInputFromController(driverController);
            },
            drivetrain));

    sprocket.setDefaultCommand(
        Commands.run(
            () -> {
              sprocket.setInputFromJoystick(operatorController);
            },
            sprocket));

    
    shooterLimelight.setPipelineTo(DetectionType.APRIL_TAG);
    intakeLimelight.setPipelineTo(DetectionType.NOTE);

    topShooterSpeakerSpeed.whenUpdate(
        (speed) -> {
          if (shooter.isShooting()) {
            shooter.shootVelocity(speed, bottomShooterSpeakerSpeed.get());
          }
        });
    bottomShooterSpeakerSpeed.whenUpdate(
        (speed) -> {
          if (shooter.isShooting()) {
            shooter.shootVelocity(topShooterSpeakerSpeed.get(), speed);
          }
        });
    transportSpeakerSpeed.whenUpdate(
        (speed) -> {
          if (shooter.isTransporting()) {
            shooter.transportVelocity(speed);
          }
        });


    // for(int port = 5800; port <= 5805; port++) {
    //   PortForwarder.add(port, "limelight.local", port);
    // }

    
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {

    // ************* DRIVER CONTROLLER BINDINGS **************** //
    driverController
        .L1()
        .onTrue(Commands555.disableFieldRelative())
        .onFalse(Commands555.enableFieldRelative());
    

    driverController
        .triangle()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    driverController
        .circle()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false));
    driverController
        .cross()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    driverController
        .square()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));

    driverController.R1().whileTrue(Commands555.scoreMode());
    // driverController.L2().onTrue(Commands555.alignToLimelightTarget(shooterLimelight));
    // driverController.R2().onTrue(Commands555.alignToLimelightTarget(intakeLimelight));

    driverController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drivetrain.getSwerveDrive().zeroGyro();
                    })
                .ignoringDisable(true));
    driverController.PS().onTrue(Commands555.lockDrive());
    

    
    

    
    
  }
  private void configureOperatorBindings() {
    // Shuffleboard.getTab("Debug").addBoolean("At speed", RobotContainer.shooter::isAtSpeed);
    // ControllerTools.getDPad(DPad.UP, operatorController)
    //     .toggleOnTrue(
    //         sprocket
    //             .getSysId()
    //             .quasistatic(Direction.kForward)
    //             .onlyWhile(sprocket::isSprocketSafe));
    // ControllerTools.getDPad(DPad.DOWN, operatorController)
    //     .toggleOnTrue(
    //         sprocket
    //             .getSysId()
    //             .quasistatic(Direction.kReverse)
    //             .onlyWhile(sprocket::isSprocketSafe));

    // ControllerTools.getDPad(DPad.RIGHT, operatorController)
    //     .toggleOnTrue(
    //         sprocket.getSysId().dynamic(Direction.kForward).onlyWhile(sprocket::isSprocketSafe));
    // ControllerTools.getDPad(DPad.LEFT, operatorController)
    //     .toggleOnTrue(
    //         sprocket.getSysId().dynamic(Direction.kReverse).onlyWhile(sprocket::isSprocketSafe));

    ControllerTools.getDPad(DPad.UP, operatorController).onTrue(Commands555.climbersUp()).onFalse(Commands555.climbersStop());
    ControllerTools.getDPad(DPad.DOWN, operatorController).onTrue(Commands555.climbersDown()).onFalse(Commands555.climbersStop());

    operatorController.R2().onTrue(Commands555.reverseIntake()).onFalse(Commands555.stopIntake());
    // operatorController.R2().onTrue(Commands555.testPipeSwitch(intakeLimelight, DetectionType.APRIL_TAG));
    operatorController.L2().whileTrue(Commands555.loadNote());


      
    operatorController.cross().whileTrue(Commands555.setSprocketAngle(angleSetpoint.get()));
    operatorController.square().whileTrue(Commands555.scoreSubwoofer());
    // operatorController.triangle().whileTrue(Commands555.scoreAmp());
    
    
    operatorController.circle().onTrue(Commands555.shoot(topShooterAmpSpeed.get(), bottomShooterAmpSpeed.get(), 0.6));
    operatorController.triangle().onTrue(Commands.sequence(
      Commands555.setSprocketAngle(RobotContainer.shooterLimelight.getAngleForSpeaker()),
      Commands555.waitUntil(() -> {
        return sprocket.isAtAngle();
      }),
      Commands555.shoot(.9, .9, .9)
    ));



    // operatorController.L1().onTrue(Commands555.celebrate());
    // operatorController.touchpad().onTrue(Commands555.ampItUp());
    // operatorController.PS().onTrue(Commands555.cooperatition());

  }

  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    // driverTab.addBoolean(
    //     "Good to shoot!",
    //     () -> {
    //       return shooter.isAtSpeed() && sprocket.isAtAngle();
    //     });
    // driverTab.addBoolean("Note Intaked", intake::hasPickedUp);

    driverTab.addDouble("Time Remaining", Timer::getMatchTime);

    driverTab.addBoolean("Note in Transport", shooter::isNoteInTransport);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("test");
    // return AutoBuilder.followPath(path);
    return auto.getAutoCommand();
  }
}
