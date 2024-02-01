package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.Arrays;
import java.util.HashSet;

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
    
    public boolean isValidPathSequence(String autoString) {
        for (int i = 0; i < autoString.length()-1; i++) {
            char char1 = autoString.charAt(i);
            char char2 = autoString.charAt(i+1);
            if (Constants.AutoConstants.notes.contains(char1) && Constants.AutoConstants.notes.contains(char2)) {
                //RobotContainer.auto.setFeedback("Insert criticism here");
                return false;
            }
            if (Constants.AutoConstants.scoringLocations.contains(char1) && Constants.AutoConstants.scoringLocations.contains(char2)) {
                //RobotContainer.auto.setFeedback("Insert criticism here");
                return false;
            }
        }
        return true;
    }

    public boolean stayingInLane(String autoString) {
        for (int i = 0; i < autoString.length()-1; i++) {
            char char1 = autoString.charAt(i);
            char char2 = autoString.charAt(i+1);
            if (Constants.AutoConstants.lane1.contains(char1) && Constants.AutoConstants.lane1.contains(char2)) {
                return true;
            }
            if (Constants.AutoConstants.lane2.contains(char1) && Constants.AutoConstants.lane2.contains(char2)) {
                return true;
            }
            if (Constants.AutoConstants.lane3.contains(char1) && Constants.AutoConstants.lane3.contains(char2)) {
                return true;
            }
            //RobotContainer.auto.setFeedback("Insert criticism here");
            return false;
        }
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
                finalPath.addCommands(Commands555.alignTo(),Commands555.eat()); 
            }
            
        }
        // 1-a.a-1.1-g-g-1.
        //return finalPath;
        return Commands.sequence(finalPath);

    }

    
}
