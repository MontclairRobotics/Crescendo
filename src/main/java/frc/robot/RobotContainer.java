// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.Drivetrain;


import java.io.File;
import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  private static CommandPS5Controller driverController = new CommandPS5Controller(0);
  
  public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    drivetrain.setupPathPlanner();
    drivetrain.setDefaultCommand(Commands.run(() -> {
      if (DriverStation.isAutonomous()) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds(0,0,0));
        return;
      } else {
        drivetrain.setInputFromController(
          new Translation2d(driverController.getRightX(),driverController.getRightY()), 
          new Translation2d(driverController.getLeftX(),driverController.getRightX())
        );

      }
    }));
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    driverController.cross().onTrue(new InstantCommand(() -> {

      Translation2d targetPose = new Translation2d(0.33, 0.33);
      Rotation2d currentRotation = drivetrain.getSwerveDrive().getOdometryHeading();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);

    }));
    driverController.circle().onTrue(new InstantCommand(() -> {

      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getSwerveDrive().getOdometryHeading();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation.rotateBy(new Rotation2d(90)));

    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return Commands.run(() -> {

    }); 

    
  }
}
