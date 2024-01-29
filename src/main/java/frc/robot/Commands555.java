package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;

public class Commands555 {
    /**
     * Drive to a robot-relative point given a Translation2d & target Rotation2d.
     * @param targetTranslation Field-relative Translation2d to drive the robot to.
     * @param theta Target angle for end position.
     */
    public static Command driveToRobotRelativePoint(Translation2d targetTranslation, Rotation2d theta) {
        Pose2d currentRobotPosition = RobotContainer.drivetrain.getSwerveDrive().getPose();
        Rotation2d currentOdometryHeading = RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading();


        Translation2d targetTranslation2d = currentRobotPosition.getTranslation().plus(targetTranslation.rotateBy(currentOdometryHeading));
        Pose2d targetPose = new Pose2d(targetTranslation2d.getX(), targetTranslation2d.getY(), theta);

        return AutoBuilder.pathfindToPose(
            targetPose,
            PathPlannerConstants.PATH_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );      
    }
    /**
     * Drive to a field-relative point given a targetPose
     * @param targetPose field-relative pose2d to drive the robot to.
     */
    public static Command driveToFieldRelativePoint(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.PATH_CONSTRAINTS, 0.0, 0.0);
    }

    /* - - - - - - - - - -
     Intake Commands
    - - - - - - - - - - */
    public static Command eat() {
        return Commands.runOnce(RobotContainer.intake::in, RobotContainer.intake).withName("intake in");
    }

    public static Command barf() {
       return Commands.runOnce(RobotContainer.intake::out, RobotContainer.intake).withName("intake out");
    }

    public static Command stopIntake() {
        return Commands.runOnce(RobotContainer.intake::stop, RobotContainer.intake).withName("intake stop");
    }
    /* - - - - - - - - - -
     Fliptop Commands
    - - - - - - - - - - */
    public static Command foward() {
        return Commands.runOnce(RobotContainer.fliptop::forward, RobotContainer.fliptop).withName("fliptop forward");
    }

    public static Command backward() {
        return Commands.runOnce(RobotContainer.fliptop::backward, RobotContainer.fliptop).withName("fliptop backward");
    }
    /* - - - - - - - - - -
     Sprocket Commands
    - - - - - - - - - - */
    public static Command goUp() {
        return Commands.runOnce(RobotContainer.sprocket::goUp, RobotContainer.sprocket).withName("sprocket up");
    }
    public static Command goDown() {
        return Commands.runOnce(RobotContainer.sprocket::goDown, RobotContainer.sprocket).withName("sprocket down");
    }
    public static Command stopSprocket() {
        return Commands.runOnce(RobotContainer.sprocket::stop, RobotContainer.sprocket).withName("sprocket stop");
    }
    /* - - - - - - - - - -
     Shooter Commands
    - - - - - - - - - - */
    public static Command shoot() {
        return Commands.runOnce(RobotContainer.shooter::shoot, RobotContainer.shooter).withName("shooter shoot");
    }
    
    public static Command stopShooter() {
        return Commands.runOnce(RobotContainer.shooter::stop, RobotContainer.shooter).withName("shooter stop");
    }
    public static Command reverseShooter() {
        return Commands.runOnce(RobotContainer.shooter::reverseShooter, RobotContainer.shooter).withName("shooter reverse");
    }

}
