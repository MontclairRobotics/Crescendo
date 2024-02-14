// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.Tunable;
import frc.robot.vision.Limelight;

import java.io.File;
import animation2.AllianceAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import animation2.api.ConditionalAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  private static CommandPS5Controller driverController = new CommandPS5Controller(0);
  private static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  
  // public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  // Subsystems
  // public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  // public static Sprocket sprocket = new Sprocket();
  // public static Limelight intakeLimelight = new Limelight("intakeLimelight");
  // public static Limelight shooterLimelight = new Limelight("shooterLimelight");
  // public static Auto auto = new Auto();
  // public static LED led = new LED(new ConditionalAnimation(getTeleopDefaultAnim()).addCase(DriverStation::isDisabled, getDisabledAnimation()), new WipeTransition());

  public static final Field2d field = new Field2d();

  private Tunable<Double> topShooterSpeed = Tunable.of(1000, "shooter/top-speed");
  private Tunable<Double> bottomShooterSpeed = Tunable.of(1000, "shooter/bottom-speed");
  private Tunable<Double> transportSpeed = Tunable.of(1000, "shooter/transport-speed");

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // auto.setupPathPlanner();
    setupAutoTab();
    
    // drivetrain.setDefaultCommand(Commands.run(() -> {
    //   drivetrain.setInputFromController(driverController); 
    // }, drivetrain));

    // sprocket.setDefaultCommand(Commands.run(() -> {
    //   sprocket.setSpeed(
    //     MathUtil.applyDeadband(operatorController.getLeftY(), 0.05) * ArmConstants.MAX_SPEED
    //   );
    // }, sprocket));

    configureBindings();

    topShooterSpeed.whenUpdate((speed) -> {
      if (shooter.isShooting()) {
        shooter.shootVelocity(speed, bottomShooterSpeed.get());
      }
    });
    bottomShooterSpeed.whenUpdate((speed) -> {
      if (shooter.isShooting()) {
        shooter.shootVelocity(topShooterSpeed.get(), speed);
      }
    });
    transportSpeed.whenUpdate((speed) -> {
      if (shooter.isTransporting()) {
        shooter.transportVelocity(speed);
      }
    });
  }


  private void configureBindings() {
    
    // driverController.touchpad().onTrue(Commands.runOnce(() -> {
    //   drivetrain.getSwerveDrive().zeroGyro();
    // }));
    
    // // TODO: probably wrong
    // driverController.cross().onTrue(Commands555.scoreSpeaker()).onFalse(Commands555.stopShooter());
    // driverController.circle().onTrue(Commands555.intake()).onFalse(Commands555.stopIntake());

    // operatorController.circle().onTrue(Commands.runOnce(() -> {
    //   shooter.shootVelocity(500);
    // }));


    operatorController.triangle().onTrue(Commands.runOnce(() -> { shooter.shootVelocity(topShooterSpeed.get(), bottomShooterSpeed.get()); }));
    operatorController.circle().onTrue(Commands.runOnce(() -> { shooter.stopShooter(); }));
    operatorController.cross().onTrue(Commands.runOnce(() -> { shooter.transportVelocity(transportSpeed.get()); }));
    operatorController.square().onTrue(Commands.runOnce(() -> { shooter.stopTransport(); }));


    operatorController.options().onTrue(Commands555.getShooterSysIdCommand("top-bottom"));
    operatorController.create().onTrue(Commands555.getShooterSysIdCommand("transport"));

    // operatorController.L1().onTrue(Commands555.signalAmp());
    // operatorController.R1().onTrue(Commands555.signalCoop());


    //////////////////////////////
    ///// OPERATOR BINDINGS /////
    ////////////////////////////

    // operatorController.circle().onTrue(Commands.run(() -> {
    //   sprocket.goToAngle(45);
    // }));


    
  }

  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    //TODO make it the same string that was entered last time? I think i can mark nt key as persistent
    autoTab.add("Enter Command", "").withSize(3,1).withPosition(0,0);
    autoTab.add(field).withSize(6,4).withPosition(3,0);
    // autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);
    
    autoTab.add("Ignore Safety", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0,2);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Actual auto command.
    return Commands.run(() -> {
      return;
    }); 
  }
}
