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
  public static Limelight intakeLimelight = new Limelight("limelight");
  public static Limelight shooterLimelight = new Limelight("limelight-shooter");
  public static Auto auto = new Auto();
  public static LED led =
      new LED(
          new ConditionalAnimation(getTeleopDefaultAnim())
              .addCase(DriverStation::isDisabled, getDisabledAnimation()),
          new WipeTransition());
  //public static Climbers climber = new Climbers();

  public static final Field2d field = new Field2d();

  private Tunable<Double> topShooterSpeakerSpeed = Tunable.of(4000, "shooter/top-speaker-speed");
  private Tunable<Double> bottomShooterSpeakerSpeed =
      Tunable.of(4500, "shooter/bottom-speaker-speed"); // 4500 was most consistent in testing
  private Tunable<Double> transportSpeakerSpeed = Tunable.of(1000, "shooter/transport-speed");

  private Tunable<Double> angleSetpoint = Tunable.of(45, "sprocket setpoint");
  
  public static Climbers climbers = new Climbers();
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
    // driverController
    //     .circle()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));
    driverController
        .cross()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    driverController
        .square()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(270), false));

    driverController.R1().whileTrue(Commands555.scoreMode());
    driverController.L2().onTrue(Commands555.alignToLimelightTarget(shooterLimelight));
    driverController.R2().onTrue(Commands555.alignToLimelightTarget(intakeLimelight));

    driverController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drivetrain.getSwerveDrive().zeroGyro();
                    })
                .ignoringDisable(true));

    

    
    

    
    
  }
  private void configureOperatorBindings() {
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

    ControllerTools.getDPad(DPad.UP, operatorController).onTrue(Commands555.climberUp()).onFalse(Commands555.climberStop());
    ControllerTools.getDPad(DPad.DOWN, operatorController).onTrue(Commands555.climberDown()).onFalse(Commands555.climberStop());

    operatorController.R2().onTrue(Commands555.reverseIntake()).onFalse(Commands555.stopIntake());
    operatorController.L2().whileTrue(Commands555.loadNote());


   

    operatorController.triangle().onTrue(Commands555.shootSpeaker());
    // operatorController.circle().onTrue(Commands.run(() -> {
    //   sprocket.setPosition(Rotation2d.fromDegrees(angleSetpoint.get()));
    // }, sprocket));
    driverController.circle().onTrue(Commands555.alignToLimelightTarget(shooterLimelight));
    // operatorController.cross().onTrue(Commands555.transport()).onFalse(Commands.runOnce(() -> {
    //   shooter.stopTransport();
    // }, shooter));
    operatorController.square().onTrue(Commands.runOnce(() -> {
      RobotContainer.climbers.up();
    }, climbers)).onFalse(Commands.runOnce(() -> {
      RobotContainer.climbers.stop();
    }, climbers));

    operatorController.cross().onTrue(Commands.runOnce(() -> {
      RobotContainer.climbers.down();
    }, climbers)).onFalse(Commands.runOnce(() -> {
      RobotContainer.climbers.stop();
    }, climbers));
    

    //operatorController.L1().onTrue(Commands555.celebrate());
    operatorController.touchpad().onTrue(Commands555.ampItUp());
    operatorController.PS().onTrue(Commands555.cooperatition());

  }

  

  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addBoolean(
        "Good to shoot!",
        () -> {
          return shooter.isAtSpeed() && sprocket.isAtAngle();
        });
    driverTab.addBoolean("Note Intaked", intake::hasPickedUp);

    driverTab.addDouble("Time Remaining", Timer::getMatchTime);

    // driverTab.addBoolean("Note in Transport", shooter::getSensor);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}
