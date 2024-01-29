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

  

  private static CommandPS5Controller driverController = new CommandPS5Controller(0);
  
  public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  // Subsystems
  public static Intake intake = new Intake();
  public static Fliptop fliptop = new Fliptop();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    drivetrain.setupPathPlanner();
    
    drivetrain.setDefaultCommand(Commands.run(() -> {
        // System.out.println(driverController.getLeftX());
      drivetrain.setInputFromController(
         driverController.getRightX(), 
       
          new Translation2d(driverController.getLeftX(),driverController.getLeftY())
      );

      
    }, drivetrain));
    configureBindings();
  }


  private void configureBindings() {
    driverController.cross().onTrue(new InstantCommand(() -> {

      Translation2d targetPose = new Translation2d(0.33, 0.33);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);

    }));
    driverController.circle().onTrue(Commands.runOnce(() -> {

      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation.rotateBy(new Rotation2d(90)));

    }));
    driverController.touchpad().onTrue(Commands.runOnce(() -> {
      drivetrain.getSwerveDrive().zeroGyro();
    }));
    driverController.triangle().onTrue(Commands.run(() -> {
      System.out.println("BUTTON PRESSED");
      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);
     
    }, drivetrain));
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
