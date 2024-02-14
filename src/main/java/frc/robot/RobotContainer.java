// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.vision.Limelight;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
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

  //public static final Field2d field = new Field2d();

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    

    driverController.square().onTrue(Commands.runOnce(() -> {
      shooter.shootSpeaker();
    }, shooter)).onFalse(Commands555.stopShooter());
    
    driverController.cross().onTrue(Commands.runOnce(() -> {
      shooter.transport();
    }, shooter));

    driverController.triangle().onTrue(Commands.runOnce(() -> {
      shooter.stopTransport();
    }, shooter));
    
    
    // .onFalse(Commands.runOnce(() -> {
    //   shooter.stopTransport();
    // }, shooter));

    

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
