package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPlannerTrajectory.State;


import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands555;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// ^(?=[^A-H1-4]*[A-H1-4])(?=[^0-9]*[0-9])(?!.*[A-Za-z]{2})(?!.*[0-9]{2})[A-H1-4]+$
public class Auto extends SubsystemBase {
    
    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
    private String feedbackValue = "Enter a command!";

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            RobotContainer.drivetrain.getSwerveDrive()::getPose, // Robot pose supplier
            RobotContainer.drivetrain.getSwerveDrive()::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            RobotContainer.drivetrain.getSwerveDrive()::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            RobotContainer.drivetrain.getSwerveDrive()::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.AutoConstants.PATH_FOLLOWER_CONFIG,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            RobotContainer.drivetrain 
        );
    } 
    //1A2
    public boolean isValidPathSequence(String autoString) {
        //Checks if the first point in the string is a starting point
        if (!(Constants.AutoConstants.STARTING_POINTS.contains(autoString.charAt(0)))) {
            //RobotContainer.auto.setFeedback("That's not a real starting spot.");
            return false;
        }
        //Loops through the string.
        for (int i = 0; i < autoString.length()-1; i++) {
            char char1 = autoString.charAt(i);
            char char2 = autoString.charAt(i+1);
            //Checks if there are two notes in a row (not good)
            if (Constants.AutoConstants.NOTES.contains(char1) && Constants.AutoConstants.NOTES.contains(char2)) {
                setFeedback("Don't go from a note to a note.");
                return false;
            }
            //Checks if there are two scoring spots in a row (not good)
            if (Constants.AutoConstants.SCORING_LOCATIONS.contains(char1) && Constants.AutoConstants.SCORING_LOCATIONS.contains(char2)) {
                setFeedback("Don't go between scoring locations.");
                return false;
            }
            //Checks for a typo (or people messing around and typing nonsense)
            if (!(Constants.AutoConstants.ALL_POINTS.contains(char2))) {
                setFeedback("You probably made a typo, or you're stupid");
                return false;
            }
        }
        return true;
        // Trajectory traj = new Trajectory(trajectories.get(1).getStates());
    }

    public void drawPaths() {
        PathPlannerTrajectory traj = trajectories.get(1);
        PathPlannerTrajectory.State ppState = traj.getState(1);
        // ppState.ge
    }

    public boolean isStayingInLane(String autoString) {

        Set<Character> lane;
        // sets lane to the lane of the first character
        if (autoString.charAt(0) == '1') lane = Constants.AutoConstants.LANE1;
        if (autoString.charAt(0) == '2') lane = Constants.AutoConstants.LANE2;
        if (autoString.charAt(0) == '3') lane = Constants.AutoConstants.LANE3;
        else return false;
        
        //loops through the string
        for (int i = 0; i < autoString.length(); i++) {

            char character = autoString.charAt(i);
            //Checks if character is in the lane
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

    public State[] convertStatesToStates(PathPlannerTrajectory.State[] ppStates) {
        State[] wpiStates = new State[ppStates.length];
        for (int i = 0; i < ppStates.length; i++) {
            PathPlannerTrajectory.State currentState = ppStates[i];
            // wpiStates[i] = new State(
            //     currentState.timeSeconds,
            //     currentState.velocityMps,
            //     currentState.accelerationMpsSq,
            //     currentState.positionMeters,
                
            // );
        }


        return wpiStates;

    }

    public Command getPathSequence(String autoString) {

    //     SequentialCommandGroup finalPath = new SequentialCommandGroup();
        
    //     for (int i = 0; i < autoString.length()-1; i++) {

    //         char current = autoString.charAt(i);
    //         char next = autoString.charAt(i+1);
    //         try {

    //             PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + next);
    //             finalPath.addCommands(AutoBuilder.followPath(path));
    //             trajectories.add(path.getTrajectory(RobotContainer.drivetrain.getSwerveDrive().getRobotVelocity(), RobotContainer.drivetrain.getRotation()));

    //         } catch(Exception e) {
    //             // TODO: amazing error handling
    //         }
        
    //         // if (Character.isDigit(next)) {
    //         //     if (next == '4') {
    //         //         finalPath.addCommands(Commands555.scoreAmp());
    //         //     } else {
    //         //         finalPath.addCommands(Commands555.scoreSpeaker());
    //         //     }
                
            } else {
                finalPath.addCommands(Commands555.alignToLimelightTarget(RobotContainer.intakeLimelight),Commands555.intake());
            }
        }
        return Commands.sequence(finalPath);
    }

}
