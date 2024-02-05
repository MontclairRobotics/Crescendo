package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.*;
import frc.robot.vision.Limelight;


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
            AutoConstants.PATH_CONSTRAINTS,
            AutoConstants.GOAL_END_VELOCITY,
            AutoConstants.ROTATION_DELAY_DISTANCE 
        );      
    }
    /**
     * Drive to a field-relative point given a targetPose
     * @param targetPose field-relative pose2d to drive the robot to.
     */
    public static Command driveToFieldRelativePoint(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
            targetPose, AutoConstants.PATH_CONSTRAINTS, 
            AutoConstants.GOAL_END_VELOCITY, 
            AutoConstants.ROTATION_DELAY_DISTANCE
        );
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
    public static Command goToAngle(double angle) {
        return RobotContainer.sprocket.goToAngle(angle);
    }

    /* - - - - - - - - - -
     Shooter Commands
    - - - - - - - - - - */
    public static Command shootSpeaker() {
        return Commands.runOnce(RobotContainer.shooter::shootSpeaker, RobotContainer.shooter).withName("shoot speaker");
    }

    public static Command shootAmp() {
        return Commands.runOnce(RobotContainer.shooter::shootAmp, RobotContainer.shooter).withName("shoot amp");
    }
    
    public static Command stopShooter() {
        return Commands.runOnce(RobotContainer.shooter::stop, RobotContainer.shooter).withName("shooter stop");
    }

    public static Command reverseShooter() {
        return Commands.runOnce(RobotContainer.shooter::reverseShooter, RobotContainer.shooter).withName("shooter reverse");
    }
    
    public static Command alignTo(Limelight limelight) {
        Pose2d currentRobotPose = RobotContainer.drivetrain.getSwerveDrive().getPose();

        // getObjectTX returns a degree offset between -29.8 & 29.8 degrees. Add this to our current heading to get the true target angle
        Rotation2d targetAngle = new Rotation2d(currentRobotPose.getRotation().getDegrees() + limelight.getObjectTX());

        Pose2d targetRobotPose = new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), targetAngle);
        return AutoBuilder.pathfindToPose(
            targetRobotPose,
            AutoConstants.PATH_CONSTRAINTS,
            AutoConstants.GOAL_END_VELOCITY,
            AutoConstants.ROTATION_DELAY_DISTANCE 
        ); 
    }

    public static Command scoreAmp() {
        return Commands.sequence(
            alignTo(RobotContainer.shooterLimelight),
            goToAngle(SubsystemConstants.AMP_SCORE_ANGLE),
            shootAmp(),
            goToAngle(SubsystemConstants.ENCODER_MIN_ANGLE)
        );
    }
    public static Command scoreSpeaker() {
        return Commands.sequence(
            alignTo(RobotContainer.shooterLimelight),
            goToAngle(SubsystemConstants.SPEAKER_SCORE_ANGLE),
            shootSpeaker(),
            goToAngle(SubsystemConstants.ENCODER_MIN_ANGLE)
        );
    }
    public static Command receiveHumanPlayerNote() {
        return Commands.sequence(
            alignTo(RobotContainer.shooterLimelight),
            goToAngle(SubsystemConstants.SPEAKER_SCORE_ANGLE),
            reverseShooter(),
            goToAngle(SubsystemConstants.ENCODER_MIN_ANGLE)
        );
    }


}