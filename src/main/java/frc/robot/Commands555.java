package frc.robot;


import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
        return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.PATH_CONSTRAINTS);
    }

    
    

}
