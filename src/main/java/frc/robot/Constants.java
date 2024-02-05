// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveConstants {
    public static final int MAX_SPEED = 4;

    public static final double MAX_ROT_SPEED = 2 * Math.PI;
    // Max Acceleration in M/s^2
    public static final double MAX_ACCELERATION = 2.0;
    // Max angular acceleration in Rad/S^2
    public static final double MAX_ANGULAR_ACCELERATION = 1.5;

  }
  
    
  
  public static class VisionConstants {
    public static final double TARGET_DEBOUNCE_TIME = 0.2;
  }
  public static class Ports {
    public static final int LED_PWM_PORT = 100;

    public static final int FLIPTOP_MOTOR_PORT = 101; 

    public static final int INTAKE_MOTOR_1_PORT = 102;
    public static final int INTAKE_MOTOR_2_PORT = 103;

    public static final int SHOOTER_MOTOR_1_PORT = 104;
    public static final int SHOOTER_MOTOR_2_PORT = 105;

    public static final int ANGLE_MOTOR_PORT = 106;
  }
  
  public static class SubsystemConstants {
    public static final double FLIPTOP_SPEED = 0.5;
    public static final double INTAKE_SPEED = 0.5;
    public static final double SPEAKER_EJECT_SPEED = 0.5;
    public static final double AMP_EJECT_SPEED = 0.1;
    public static final double ANGLE_MOVE_SPEED = 0.5;

    //TODO: Angle PID Constants: (!!!!!!)
    public static final double angleKP = 0.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    //TODO: All of these constants are basically guessed!!
    public static final double ANGLE_SPEED = 0.5; //Speed of the angle changing
    public static final double SPROCKET_ROTATIONS_PER_DEGREE = 1.26983333;
    public static final double ENCODER_MIN_ANGLE = 0.0;
    public static final double ENCODER_MAX_ANGLE = 75.787;
    public static final double SPEAKER_SCORE_ANGLE = -1;
    public static final double AMP_SCORE_ANGLE = -1;


    //TODO: needs to be set
    public static final int BOTTOM_LIMIT_SWITCH = 0;
    public static final int TOP_LIMIT_SWITCH = 1;

    public static final double FF_VOLTAGE = 0.6; //TODO: stolen from ChargedUp elevator feedforward  }
}

public static class AutoConstants {

    public static final Set<Character> notes = Arrays.stream(new Character[] {'A','B','C','D','E','F','G','H'}).collect(Collectors.toSet());
    public static final Set<Character> scoringLocations = Arrays.stream(new Character[] {'1','2','3','4'}).collect(Collectors.toSet());
    public static final Set<Character> lane3 = Arrays.stream(new Character[] {'A','3','D', 'E'}).collect(Collectors.toSet());
    public static final Set<Character> lane2 = Arrays.stream(new Character[] {'B', '2','F','G','E'}).collect(Collectors.toSet());
    public static final Set<Character> lane1 = Arrays.stream(new Character[] {'C','1','4', 'G', 'H'}).collect(Collectors.toSet());
    public static final Set<Character> ALL_POINTS = Arrays.stream(new Character[] {'A','B','C','D','E','F','G','H', '1','2','3','4'}).collect(Collectors.toSet());

    public static final PIDConstants ANGULAR_PID_CONSTANTS = new PIDConstants(3.0,0.0,0.0);
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3.0,0.0,0.0);

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
      TRANSLATION_PID_CONSTANTS,
      ANGULAR_PID_CONSTANTS,
      Constants.DriveConstants.MAX_SPEED, 
      0.43, 
      new ReplanningConfig()
    );
    
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
      DriveConstants.MAX_SPEED, DriveConstants.MAX_ACCELERATION,
      DriveConstants.MAX_ROT_SPEED, DriveConstants.MAX_ANGULAR_ACCELERATION
    );
    
    public static final double GOAL_END_VELOCITY = 0.0;
    public static final double ROTATION_DELAY_DISTANCE = 0.0;
    /*    |X|(AMP)
     * 
     * \ 3    A        |       D
     *  \              |       
     SPKR| 2  B        |       E
     *  /           ╱| |       
     * / 1    C     ╲| |       F
     *                 |
     *                 |       G
     *                 |
     *                 |       H
     * (human player)
     */
  }
  
  public static class LEDConstants{
    public static final int LED_PWM_PORT = 0;
  }

}

