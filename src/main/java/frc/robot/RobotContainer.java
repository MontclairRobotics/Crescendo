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
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;
import swervelib.SwerveDrive;

import java.io.File;
import animation2.AllianceAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import animation2.api.ConditionalAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {

  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  //public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  
  public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  // Subsystems
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
  public static Limelight intakeLimelight = new Limelight("limelight");
  public static Limelight shooterLimelight = new Limelight("shooterLimelight");
  public static Auto auto = new Auto();
  public static LED led = new LED(new ConditionalAnimation(getTeleopDefaultAnim()).addCase(DriverStation::isDisabled, getDisabledAnimation()), new WipeTransition());

  public static final Field2d field = new Field2d();

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    auto.setupPathPlanner();
    setupAutoTab();
    //TODO write the command
    drivetrain.setDefaultCommand()

    configureBindings();
    intakeLimelight.setPipelineTo(DetectionType.NOTE);
  }

  private void configureBindings() {  
    //TODO: figure out what method to use for "align to april tag" and assign the command to the button
    //TODO: write R2 and L2
    //TODO: write right stick and left stick
    driverController.L1().onTrue(Commands555.disableFieldRelative()).onFalse(Commands555.enableFieldRelative());
    driverController.R1().whileTrue(Commands555.scoreMode());
    driverController.triangle().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    driverController.circle().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));
    driverController.cross().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    driverController.square().onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(270), false));
    driverController.touchpad().onTrue(Commands555.zeroGyro());
      
    } catch (Exception e) {
      // TODO: handle exception
    }2().


  }
  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    // TODO make it the same string that was entered last time? I think i can mark nt key as persistent
    autoTab.add("Enter Command", "").withSize(3,1).withPosition(0,0);
    autoTab.add(field).withSize(6,4).withPosition(3,0);
    autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);
    
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
