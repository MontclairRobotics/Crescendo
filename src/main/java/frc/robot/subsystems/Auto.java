package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Auto extends SubsystemBase {
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            RobotContainer.drivetrain.swerveDrive::getPose, // Robot pose supplier
            RobotContainer.drivetrain.swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            RobotContainer.drivetrain.swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            RobotContainer.drivetrain.swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.PathPlannerConstants.PATH_FOLLOWER_CONFIG,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this 
        );
    }
}
