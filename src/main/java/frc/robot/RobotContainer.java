// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
<<<<<<< Updated upstream
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
=======

























  
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

  // public static final Field2d field = new Field2d();
>>>>>>> Stashed changes

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
<<<<<<< Updated upstream
    
    // Configure the trigger bindings
=======
    // auto.setupPathPlanner();
    // setupAutoTab();
    
    // drivetrain.setDefaultCommand(Commands.run(() -> {
    // drivetrain.setInputFromController(driverController);


      
    // }, drivetrain));
>>>>>>> Stashed changes
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
<<<<<<< Updated upstream

   
  }
=======
    // driverController.touchpad().onTrue(Commands.runOnce(() -> {
    //   drivetrain.getSwerveDrive().zeroGyro();
    // }));
    
    // TODO: probably wrong
    // driverController.cross().whileTrue(Commands.run(() -> {
    //   shooter.transport();
    // }, shooter)).whileFalse(Commands.runOnce(() -> {
    //   shooter.stop();
    // }, shooter));
    driverController.cross().onTrue(Commands.run(() -> {
      shooter.transport();
      System.out.println("abe is ugly");
    }));

    driverController.square().whileTrue(Commands.run(() -> {
      shooter.shoot();
    }, shooter)).onFalse(Commands.runOnce(()-> {
      shooter.stop();
    }, shooter));
  
   


    
    
    
  }

  // public static Animation getTeleopDefaultAnim() {
  //   return new AllianceAnimation();
  // }

  // public static Animation getDisabledAnimation() {
  //   return Constants.LEDConstants.DEMO_REEL;
  // }

  // public void setupAutoTab() {
  //   ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  //   //TODO make it the same string that was entered last time? I think i can mark nt key as persistent
  //   autoTab.add("Enter Command", "").withSize(3,1).withPosition(0,0);
  //   autoTab.add(field).withSize(6,4).withPosition(3,0);
  //   autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);
    
  //   autoTab.add("Ignore Safety", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0,2);


  //}
>>>>>>> Stashed changes

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
<<<<<<< Updated upstream
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
=======
  // public Command getAutonomousCommand() {
  //   // TODO: Actual auto command.
  //   return Commands.run(() -> {
  //     return;
  //   }); 

    
  // }
>>>>>>> Stashed changes
}
