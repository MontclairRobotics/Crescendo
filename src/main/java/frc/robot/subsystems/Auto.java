package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands555;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Auto extends SubsystemBase {
    

    private String feedbackValue = "Enter a command!";

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
    
    public void setFeedback(String feedback) {
        feedbackValue = feedback;
    }

    public String getFeedback() {
        return feedbackValue;
    }

    public Command getPathSequence(String autoString) {
       
        SequentialCommandGroup finalPath = new SequentialCommandGroup();
        String[] pathSequence = autoString.split(".");

        for (String wantedPath : pathSequence) {

            PathPlannerPath path = PathPlannerPath.fromPathFile(wantedPath);
            finalPath.addCommands(AutoBuilder.followPath(path));
            
            char decidingChar = wantedPath.charAt(2);

            if (Character.isDigit(decidingChar)) {
                finalPath.addCommands(Commands555.shoot());
            } else {
                finalPath.addCommands(Commands555.alignTo(RobotContainer.intakeLimelight),Commands555.eat()); 
            }
            
        }

        return Commands.sequence(finalPath);

    }

}
