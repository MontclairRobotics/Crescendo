package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
    
    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
    private String feedbackValue = "Enter a command!";

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            RobotContainer.drivetrain.swerveDrive::getPose, // Robot pose supplier
            RobotContainer.drivetrain.swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            RobotContainer.drivetrain.swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            RobotContainer.drivetrain.swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.AutoConstants.PATH_FOLLOWER_CONFIG,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this 
        );
    } 
    //1A2
    public boolean isValidPathSequence(String autoString) {
        if (!(Constants.AutoConstants.scoringLocations.contains(autoString.charAt(0)))) {
            RobotContainer.auto.setFeedback("That's not a real starting spot.");
            return false;
        }
        for (int i = 0; i < autoString.length()-1; i++) {
            char char1 = autoString.charAt(i);
            char char2 = autoString.charAt(i+1);
            if (Constants.AutoConstants.notes.contains(char1) && Constants.AutoConstants.notes.contains(char2)) {
                setFeedback("Don't go from a note to a note.");
                return false;
            }
            if (Constants.AutoConstants.scoringLocations.contains(char1) && Constants.AutoConstants.scoringLocations.contains(char2)) {
                setFeedback("Don't go between scoring locations.");
                return false;
            }
            if (!(Constants.AutoConstants.ALL_POINTS.contains(char1) && Constants.AutoConstants.ALL_POINTS.contains(char2))) {
                setFeedback("You probably made a typo, or you're stupid");
                return false;
            }
        }
        return true;
    }

    public boolean isStayingInLane(String autoString) {

        Set<Character> lane ;
        if (autoString.charAt(0) == '1') lane = Constants.AutoConstants.lane1;
        if (autoString.charAt(0) == '2') lane = Constants.AutoConstants.lane2;
        if (autoString.charAt(0) == '3') lane = Constants.AutoConstants.lane3;
        else return false;
        
        for (int i = 0; i < autoString.length(); i++) {

            char character = autoString.charAt(i);
            if (!lane.contains(character)) {
                setFeedback("STAY IN YOUR LANE!!!");
                return false;
            } 
        }
        return true;
    }
    
    public void setFeedback(String feedback) {
        feedbackValue = feedback;
    }

    public String getFeedback() {
        return feedbackValue;
    }

    public Command getPathSequence(String autoString) {

        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        for (int i = 0; i < autoString.length()-1; i++) {

            char current = autoString.charAt(i);
            char next = autoString.charAt(i+1);

            PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + next);
            finalPath.addCommands(AutoBuilder.followPath(path));

            trajectories.add(path.getTrajectory(RobotContainer.drivetrain.getSwerveDrive().getRobotVelocity(), RobotContainer.drivetrain.getRotation()));

            if (Character.isDigit(next)) {
                finalPath.addCommands(Commands555.shoot());
            } else {
                finalPath.addCommands(Commands555.alignTo(RobotContainer.intakeLimelight),Commands555.eat());
            }
        }
        return Commands.sequence(finalPath);
    }

}
