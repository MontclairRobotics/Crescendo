package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import java.io.File;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Drivetrain extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative;

    
    public Drivetrain(File directory) {

        this.isFieldRelative = true;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }

    }

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, this.isFieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    
    public void pidTest() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        
        swerveDrive.setModuleStates(states, false);
    }


    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void setIsFieldRelative(boolean relative) {
        this.isFieldRelative = relative;
    }
    public boolean getIsFieldRelative(boolean relative) {
        return this.isFieldRelative;
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }
    public void resetOdometry() {
        this.swerveDrive.resetOdometry(new Pose2d(0.0,0.0, new Rotation2d(0.0)));
    }
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            this.swerveDrive::getPose, // Robot pose supplier
            this.swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this.swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this.swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.PathPlannerConstants.PATH_FOLLOWER_CONFIG,
            () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this 
        );
    }

    public void setInputFromController(Translation2d turn, Translation2d drive) {
        double xSpeed = -MathUtil.applyDeadband(drive.getX(), 0.02) * DriveConstants.MAX_SPEED;
        double ySpeed = -MathUtil.applyDeadband(drive.getY(), 0.02) * DriveConstants.MAX_SPEED;
        double rot = MathUtil.applyDeadband(turn.getX(), 0.02) * DriveConstants.MAX_ROT_SPEED;

        Translation2d translation = new Translation2d(xSpeed,ySpeed);
        
        this.drive(translation, rot);
    }

}

