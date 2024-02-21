// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Fliptop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import java.io.File;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;


public class RobotContainer {

<<<<<<< Updated upstream
  

  private static CommandPS5Controller driverController = new CommandPS5Controller(0);
=======
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  // public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
>>>>>>> Stashed changes
  
  //public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
<<<<<<< Updated upstream
  // Subsystems
  public static Intake intake = new Intake();
  public static Fliptop fliptop = new Fliptop();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
=======
  // // Subsystems
  //public static Intake intake = new Intake();
  //public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
  // public static Limelight intakeLimelight = new Limelight("limelight");
  // public static Limelight shooterLimelight = new Limelight("shooterLimelight");
  // public static Auto auto = new Auto();
  // public static LED led = new LED(new ConditionalAnimation(getTeleopDefaultAnim()).addCase(DriverStation::isDisabled, getDisabledAnimation()), new WipeTransition());

  // public static final Field2d field = new Field2d();
>>>>>>> Stashed changes

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
<<<<<<< Updated upstream
    
    drivetrain.setupPathPlanner();
    
    drivetrain.setDefaultCommand(Commands.run(() -> {
        // System.out.println(driverController.getLeftX());
      drivetrain.setInputFromController(
         driverController.getRightX(), 
       
          new Translation2d(driverController.getLeftX(),driverController.getLeftY())
      );
=======
    // auto.setupPathPlanner();
    //setupAutoTab();
    
    // drivetrain.setDefaultCommand(Commands.run(() -> {
    //   drivetrain.setInputFromController(driverController); 
    // }, drivetrain));
>>>>>>> Stashed changes

      
    }, drivetrain));
    configureBindings();
<<<<<<< Updated upstream
  }


  private void configureBindings() {
    driverController.cross().onTrue(new InstantCommand(() -> {

      Translation2d targetPose = new Translation2d(0.33, 0.33);
      Rotation2d currentRotation = drivetrain.getSwerveDrive().getOdometryHeading();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);

    }));
    driverController.circle().onTrue(Commands.runOnce(() -> {

      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getSwerveDrive().getOdometryHeading();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation.rotateBy(new Rotation2d(90)));

    }));
    driverController.touchpad().onTrue(Commands.runOnce(() -> {
      drivetrain.getSwerveDrive().zeroGyro();
    }));
    driverController.triangle().onTrue(Commands.run(() -> {
      System.out.println("BUTTON PRESSED");
      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getSwerveDrive().getOdometryHeading();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);
     
    }, drivetrain));
  }
=======
    //intakeLimelight.setPipelineTo(DetectionType.NOTE);
  }

  private void configureBindings() {  
    // driverController.touchpad().onTrue(Commands.runOnce(() -> {
    //   drivetrain.getSwerveDrive().zeroGyro();
    // }).ignoringDisable(true));

    driverController.square().whileTrue(Commands555.goUp()).onFalse(Commands555.stopSprocket());
    driverController.circle().whileTrue(Commands555.goDown()).onFalse(Commands555.stopSprocket());

    // ************** OPERATOR CONTROLLER BINDINGS ************** //
    // operatorController.R2().onTrue(Commands555.reverseIntake()).onFalse(Commands555.stopIntake());
    // operatorController.L2().onTrue(Commands555.intake()).onFalse(Commands555.stopIntake());
    // operatorController.circle().onTrue(Commands555.scoreAmp());
    // operatorController.square().onTrue(Commands555.scoreSpeaker());
    // operatorController.L1().onTrue(Commands555.celebrate());
    // operatorController.touchpad().onTrue(Commands555.ampItUp());
    ///operatorController.PS().onTrue(Commands555.Cooperatition());
    }


  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  // public void setupAutoTab() {
  //   ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  //   // TODO make it the same string that was entered last time? I think i can mark nt key as persistent
  //   autoTab.add("Enter Command", "").withSize(3,1).withPosition(0,0);
  //   autoTab.add(field).withSize(6,4).withPosition(3,0);
  //   autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);
    
  //   autoTab.add("Ignore Safety", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0,2);


  // }
>>>>>>> Stashed changes

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
