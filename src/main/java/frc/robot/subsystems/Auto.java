package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
// import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands555;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Array555;
import frc.robot.vision.DetectionType;

import static frc.robot.Constants.ArmConstants.INTAKE_ANGLE;
// import static frc.robot.Constants.ArmConstants.INTAKE_SCORE_ANGLE;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;

// ^(?=[^A-H1-4]*[A-H1-4])(?=[^0-9]*[0-9])(?!.*[A-Za-z]{2})(?!.*[0-9]{2})[A-H1-4]+$
public class Auto extends SubsystemBase {

  private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
  private String feedbackValue = "Enter a command!";
  private Command autoCommand = Commands.runOnce(() -> {});
  private Alliance alliance = Alliance.Blue;

  GenericEntry autoStringEntry;
  GenericEntry safetyEntry;

  public Auto() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable("Shuffleboard").getSubTable("Auto");

    autoStringEntry = table.getTopic("Enter Command").getGenericEntry();
    safetyEntry = table.getTopic("Ignore Safety").getGenericEntry();
    // Shuffleboard.getTab("Auto").addBoolean("Is Valid", () -> {
    //   return isValidPathSequence(autoStringEntry.getString(""));
    // });
    // Shuffleboard.getTab("Auto").addBoolean("Is Safe", () -> {
    //   return isStayingInLane(autoStringEntry.getString(""));
    // });
    
  }
  //TODO maybe add handling in case command is invalid and robot is run?

  public Command getAutoCommand() {
    return autoCommand;
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        RobotContainer.drivetrain.getSwerveDrive()::getPose, // Robot pose supplier
        RobotContainer.drivetrain.getSwerveDrive()
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        RobotContainer.drivetrain.getSwerveDrive()
            ::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (ChassisSpeeds x) -> {
          RobotContainer.drivetrain.getSwerveDrive().drive(new ChassisSpeeds(x.vxMetersPerSecond, x.vyMetersPerSecond, x.omegaRadiansPerSecond), true, new Translation2d());
        }, // Method that will drive the robot given ROBOT RELATIVE, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        Constants.AutoConstants.PATH_FOLLOWER_CONFIG,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        RobotContainer.drivetrain);
  }

  // 1A2
  public boolean isValidPathSequence(String autoString) {
    // Checks if the first point in the string is a starting point
    if (autoString.length() == 0) return true;
    if (Array555.indexOf(Constants.AutoConstants.STARTING_POINTS, autoString.charAt(0)) == -1) {
      setFeedback("That's not a real starting spot.");
      return false;
    }
    // Loops through the string.
    for (int i = 0; i < autoString.length() - 1; i++) {
      char char1 = autoString.charAt(i);
      char char2 = autoString.charAt(i + 1);
      // Checks if there are two notes in a row (not good)
      if (Array555.indexOf(Constants.AutoConstants.NOTES, char1) != -1
          && Array555.indexOf(Constants.AutoConstants.NOTES, char2) != -1) {
        setFeedback("Don't go from a note to a note.");
        clearAll();
        return false;
      }
      // Checks if there are two scoring spots in a row (not good)
      if (Array555.indexOf(Constants.AutoConstants.SCORING_LOCATIONS, char1) != -1
          && Array555.indexOf(Constants.AutoConstants.SCORING_LOCATIONS, char2) != -1) {
        setFeedback("Don't go between scoring locations.");
        clearAll();
        return false;
      }
      // Checks for a typo (or people messing around and typing nonsense)
      if (Array555.indexOf(Constants.AutoConstants.ALL_POINTS, char2) == -1) {
        setFeedback("You probably made a typo, or you're stupid");
        clearAll();
        return false;
      }
    }
    return true;
    // Trajectory traj = new Trajectory(trajectories.get(1).getStates());
  }

  public void drawPaths() {
    clearField();
    for (int i = 0; i < trajectories.size(); i++) {
      PathPlannerTrajectory pathTraj = trajectories.get(i);
      List<State> states = convertStatesToStates(pathTraj.getStates());
      Trajectory displayTrajectory = new Trajectory(states);
      
      
      // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        
      //   // displayTrajectory = displayTrajectory.relativeTo(new Pose2d(33, 0, Rotation2d.fromDegrees(180)));
      //   displayTrajectory = displayTrajectory.relativeTo(new Pose2d(33,16, Rotation2d.fromDegrees(180)));
      // }

      RobotContainer.field.getObject("traj" + i).setTrajectory(displayTrajectory);
      // Shuffleboard.getTab("Auto").add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
      // RobotContainer.field.
    }
    // ppState.ge
  }

  public void clearField() {
      for (int i  = 0; i < 100; i++) {
        FieldObject2d obj = RobotContainer.field.getObject("traj" + i);
        // obj.setPose(new Pose2d(-100, -100, Rotation2d.fromDegrees(0)));
        obj.setTrajectory(new Trajectory());
      }
  }

  public void clearAll() {
    trajectories.clear();
    clearField();
  }

  public boolean isStayingInLane(String autoString) {
    char[] lane;
    if (autoString.length() == 0) return true; 
    
    // sets lane to the lane of the first character
    if (autoString.charAt(0) == '1') lane = Constants.AutoConstants.LANE1;
    else if (autoString.charAt(0) == '2') lane = Constants.AutoConstants.LANE2;
    else if (autoString.charAt(0) == '3' || autoString.charAt(0) == '4') lane = Constants.AutoConstants.LANE3;
    else return false;
    
    

    // loops through the string
    for (int i = 0; i < autoString.length(); i++) {

      char character = autoString.charAt(i);
      // Checks if character is in the lane
      if (Array555.indexOf(lane, character) == -1) {
        setFeedback("STAY IN YOUR LANE!!!");
        clearAll();
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

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    // TODO make it the same string that was entered last time? I think i can mark nt key as
    // persistent
    autoTab.add("Enter Command", "").withSize(3, 1).withPosition(0, 0);
    autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
    autoTab.addString("Feedback", () -> feedbackValue).withSize(3,1).withPosition(0,1);
    // autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);

    autoTab
        .add("Ignore Safety", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(2, 1)
        .withPosition(0, 2);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable ntTable = inst.getTable("Shuffleboard").getSubTable("Auto");

    

    ntTable.addListener(
        "Enter Command",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
         validateAndCreatePaths();
        });
       
    ntTable.addListener(
        "Ignore Safety",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
         validateAndCreatePaths();
        });    
    
    //TODO needed In case there is already data in networktables?
    // validateAndCreatePaths();

  }

  public List<State> convertStatesToStates(List<PathPlannerTrajectory.State> ppStates) {
    ArrayList<State> wpiStates = new ArrayList<State>();
    for (int i = 0; i < ppStates.size(); i++) {
      // PathPlannerPath
      PathPlannerTrajectory.State currentState = ppStates.get(i);
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        wpiStates.add(new State(
          currentState.timeSeconds,
          currentState.velocityMps,
          currentState.accelerationMpsSq,
          new Pose2d(16.54175-currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading), //TODO get correct number
          currentState.curvatureRadPerMeter
      ));
      } else {
        wpiStates.add(new State(
          currentState.timeSeconds,
          currentState.velocityMps,
          currentState.accelerationMpsSq,
          new Pose2d(currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
          currentState.curvatureRadPerMeter
      ));
      }
    }

    return wpiStates;
  }

  private void buildPathSequence(String autoString) {


    SequentialCommandGroup finalPath = new SequentialCommandGroup();
    trajectories.clear();

    if (autoString.length() == 0) {
      autoCommand = Commands.sequence(Commands555.setAutoPose(autoString), Commands555.scoreSubwoofer());
      return;
    }
    
    finalPath.addCommands(Commands555.setAutoPose(autoString));

    if (autoString.length() >= 1) {
      char digit = autoString.charAt(0);
      if (digit == '4') {
          finalPath.addCommands(Commands555.scoreAmp());
        } else {
          finalPath.addCommands(Commands555.scoreSubwoofer());
        }
    }

  
    for (int i = 0; i < autoString.length() - 1; i++) {

      char current = autoString.charAt(i);
      char next = autoString.charAt(i + 1);
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + "-" + next);
        
        ParallelCommandGroup segment = new ParallelCommandGroup(AutoBuilder.followPath(path));
        if (!Character.isDigit(next)) {
          segment.addCommands(Commands555.setSprocketAngle(INTAKE_ANGLE));

        }
        finalPath.addCommands(segment);
        trajectories.add(
            path.getTrajectory(
              new ChassisSpeeds(),
              path.getPreviewStartingHolonomicPose().getRotation()
            ));

        // if (firstPath) {
        //   RobotContainer.drivetrain.getSwerveDrive().resetOdometry(path.getPreviewStartingHolonomicPose());
        //   firstPath = false;
        // } //TODO make this a command, this won't work if they move the robot after
      } catch (Exception e) {
        // TODO: amazing error handling
        setFeedback("Path File Not Found");
        autoCommand = Commands.runOnce(() -> {}); //TODO this right?
      }

      if (Character.isDigit(next)) {
        if (next == '4') {
          finalPath.addCommands(Commands555.scoreAmp());
        } else {
          finalPath.addCommands(Commands555.scoreSubwoofer());
        }

      } else {
        
        finalPath.addCommands(Commands555.autonomousIntake());
      }
    }
    setFeedback("Successfully Created Auto Sequence!");
    autoCommand = finalPath;
  }

  public void validateAndCreatePaths() {
    String autoString = autoStringEntry.getString("");
    // previousString = autoString;

    boolean ignoreSafety = safetyEntry.getBoolean(false);


    boolean isSafePath = false;
    if (ignoreSafety) {
      isSafePath = true;
    } else {
      isSafePath = isStayingInLane(autoString); // :)
    }
    
    boolean isValidPath = isValidPathSequence(autoString);
    // setFeedback("boo!");
    if (isValidPath) {
      if (!ignoreSafety && isSafePath) {
        buildPathSequenceOdometry(autoString);
        drawPaths();
      } else if (ignoreSafety) {
        buildPathSequenceOdometry(autoString);
        drawPaths();
      }
    }
  }

  @Override
  public void periodic() {
   if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() != alliance) {
    alliance = DriverStation.getAlliance().get();
    validateAndCreatePaths();
   }
    
  }

  public void buildPathSequenceOdometry(String autoString) {

    
    SequentialCommandGroup finalPath = new SequentialCommandGroup();
    
    trajectories.clear();

    if (autoString.length() == 0) {
      autoCommand = Commands.sequence(Commands555.setAutoPose(autoString), Commands555.scoreSubwoofer());
      return;
    }

    finalPath.addCommands(Commands555.setAutoPose(autoString));
    finalPath.addCommands(Commands.runOnce(() -> {
      RobotContainer.shooter.shootActually(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
    }));
    finalPath.addCommands(Commands555.waitForTime(0.75)); //Wait for shooter to ramp up initially

    if (autoString.length() >= 1) {
      char digit = autoString.charAt(0);
      if (digit == '4') {
          finalPath.addCommands(Commands555.scoreAmp());
        } else {
          finalPath.addCommands(Commands555.scoreSubwoofer());
        }
    }

    
    ParallelCommandGroup segment = new ParallelCommandGroup();
    for (int i = 0; i < autoString.length() - 1; i++) {

      char current = autoString.charAt(i);
      char next = autoString.charAt(i+1);
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + "-" + next);
        trajectories.add(
            path.getTrajectory(
              new ChassisSpeeds(),
              path.getPreviewStartingHolonomicPose().getRotation()
            ));
        segment = new ParallelCommandGroup(AutoBuilder.followPath(path));
        
      } catch (Exception e) {
        setFeedback("Path File Not Found");
        autoCommand = Commands.runOnce(() -> {});
        
      }

      if (Array555.indexOf(AutoConstants.NOTES, next) != -1) {
        segment.addCommands(Commands555.loadNoteAuto()); //A version of loadNote that ramps the shooter back up to speaker speed after
      } 

      
      finalPath.addCommands(segment);

      if (!Character.isAlphabetic(next)) {
        if (next == '4') {
          finalPath.addCommands(Commands555.scoreAmpAuto());
        } else { // speaker
          finalPath.addCommands(Commands555.scoreSubwoofer());
        }
      }
      
    }
    finalPath.addCommands(Commands.runOnce(() -> {
      RobotContainer.shooter.stop();
    }));

    setFeedback("Successfully Created Auto Sequence!");
    autoCommand = finalPath;
  }


  public void buildPathSequenceWeird(String autoString) {
    SequentialCommandGroup finalPath = new SequentialCommandGroup();

    trajectories.clear();

    if (autoString.length() == 0) {
      autoCommand = Commands.sequence(Commands555.setAutoPose(autoString), Commands555.scoreSubwoofer());
      return;
    }

    finalPath.addCommands(Commands555.setAutoPose(autoString));

    for (int i = 0; i < autoString.length()-1; i++) {
      char current = autoString.charAt(i);
      char next = autoString.charAt(i+1);
       try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + "-" + next);
        trajectories.add(
            path.getTrajectory(
              new ChassisSpeeds(),
              path.getPreviewStartingHolonomicPose().getRotation()
            ));
        finalPath.addCommands(AutoBuilder.followPath(path));
        
        } catch (Exception e) {
          setFeedback("Path File Not Found");
          autoCommand = Commands.runOnce(() -> {}); 
        }

        if (Array555.indexOf(AutoConstants.NOTES, next) != -1) {
          finalPath.addCommands(Commands555.loadNoteAuto()); 
        }

        if (next == '4') {
          finalPath.addCommands(Commands555.scoreAmp());
        } else if (next == '1' || next == '2' || next == '3') {
          finalPath.addCommands(Commands555.scoreSubwoofer());
        }
        // if (autoString.length() >= 1) {
        //   char digit = autoString.charAt(0);
        // if (digit == '4') {
        //   finalPath.addCommands(Commands555.scoreAmp());
        // } else {
        //   finalPath.addCommands(Commands555.scoreSubwoofer());
        // }
    }

    
    autoCommand = finalPath;

  }

}
  

