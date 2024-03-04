// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import animation2.AnimationReel;
import animation2.CircusAnimation;
import animation2.MagicAnimation;
import animation2.QuickSlowFlash;
import animation2.RaceAnimation;
import animation2.RainbowAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Tunable;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static Tunable<Double> kp = Tunable.of(0.4, "heading.kp");
    public static Tunable<Double> kd = Tunable.of(0, "heading.kd");
    public static Tunable<Double> ki = Tunable.of(0.01, "heading.ki");

    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); // TOOD get real width

    public static final boolean IS_OPEN_LOOP = true;

    public static final double MAX_VOLTAGE_V = 12.0;
    public static final int MAX_SPEED = 4;

    public static final double MAX_ROT_SPEED = 2 * Math.PI;
    // Max Acceleration in M/s^2
    public static final double MAX_ACCELERATION = 3.0;
    // Max angular acceleration in Rad/S^2
    public static final double MAX_ANGULAR_ACCELERATION = 1.5;

    public static final double DRIVE_BASE_RADIUS = 0.43;

    public static final Pose2d EDGE_OF_DRIVEBASE =
        new Pose2d(0, DRIVE_BASE_RADIUS + BUMPER_WIDTH, new Rotation2d());
    // TODO: correct angle deadband
    public static final double ANGLE_DEADBAND = 3;
  }

  public static class VisionConstants {
    public static final double TARGET_DEBOUNCE_TIME = 0.2;
    public static final double SHOOTER_LIMELIGHT_HEIGHT = 10.375;
    public static final double INTAKE_LIMELIGHT_HEIGHT = 10.227995;
    public static final double SPEAKER_APRILTAG_HEIGHT = 57.875;
    public static final double SHOOTER_LIMELIGHT_ANGLE_DEGREES = 27.5;
    public static final double INTAKE_LIMELIGHT_ANGLE_DEGREES = 0;
  }

  public static class Ports { // TODO: add correct ports
    public static final int LED_PWM = 5;

    public static final int INTAKE_TOP_MOTOR = 21;
    public static final int INTAKE_BOTTOM_MOTOR = 20;

    
    public static final int SHOOTER_BOTTOM_MOTOR = 40;
    public static final int SHOOTER_TOP_MOTOR = 41;
    public static final int SHOOTER_MOTOR_TRANSPORT = 42;

    // Sprocket motors
    public static final int LEFT_ANGLE_MOTOR = 30;
    public static final int RIGHT_ANGLE_MOTOR = 31;

    public static final int TRANSPORT_BEAM_BREAK = 7;

    // Climber ports
    public static final int CLIMBER_LEFT_MOTOR = 48;
    public static final int CLIMBER_RIGHT_MOTOR = 49;

    

    public static final int SPROCKET_ABS_ENCODER = 9;
  }

  public static class ClimberConstants {
    // Climbers
    public static final double CLIMBER_SPEED = 0.5;
    public static final double MAX_HEIGHT = 18; // Inches
    public static final double ROTATIONS_PER_INCH = 12.0672 * Math.PI; // TODO:
  }

  public static class IntakeConstants {
    // Intake
    public static final double INTAKE_SPEED = 0.5;
    public static final boolean INTAKE_BEAM_INVERT = true;
  }

  public static class ArmConstants {

    public static final double MAX_VOLTAGE_V = 12.0;
    // TODO: needs to be set
    
    public static final boolean SPROCKET_BEAM_INVERT = false;
    


    // public static final Tunable<Double> kS = Tunable.of(0, "arm.feedforward.ks");
    // public static final Tunable<Double> kG = Tunable.of(0.1, "arm.feedforward.kG");
    // public static final Tunable<Double> kV = Tunable.of(8.91, "arm.feedforward.kV");
    // public static final Tunable<Double> kA = Tunable.of(0.01, "arm.feedforward.ka");

    // TODO: Angle PID Constants: (!!!!!!)
    public static final Tunable<Double> angleKP = Tunable.of(0.8, "arm.kp");
    public static final Tunable<Double> angleKI = Tunable.of(0, "arm.ki");
    public static final Tunable<Double> angleKD = Tunable.of(0, "arm.kd");

    
    public static final double SPROCKET_ROTATIONS_PER_DEGREE = 1.26984126984;
    public static final double ENCODER_MIN_ANGLE = 32;
    public static final double ENCODER_MAX_ANGLE = 75.787;

    public static final double SPEAKER_SCORE_ANGLE = 62.7;
    public static final double AMP_SCORE_ANGLE = 50;


    public static final Tunable<Boolean> LEFT_INVERT = Tunable.of(true, "arm.invert.left");
    public static final Tunable<Boolean> RIGHT_INVERT = Tunable.of(true, "arm.invert.right");

    // Sprocket
    // TODO: Validate values
    public static final double SPROCKET_ANGLE_MOVE_SPEED = 0.5;
    public static final double SPROCKET_ANGLE_DEADBAND = 2;
    public static final double SPROCKET_ANGLE_LIMIT_DEADBAND = 2.5;

    public static final double INTAKE_SCORE_ANGLE = 45;

    
  }

  public static class ShooterConstants {
    // Shooter
    public static final double SPEAKER_EJECT_SPEED = 3500;
    public static final double AMP_EJECT_SPEED = 1000;

    public static Tunable<Double> kp = Tunable.of(0.0005, "shooter.kp");
    public static Tunable<Double> ki = Tunable.of(0, "shooter.ki");
    public static Tunable<Double> kd = Tunable.of(0, "shooter.kd");
    public static Tunable<Double> ff = Tunable.of(0.0, "shooter.ff");

    public static final double MAX_RPM = 5700;

    public static final int VELOCITY_DEADBAND = 150;

    public static final double TRANSPORT_SPEED = 0.6;
    
    public static final boolean BEAM_BRAKE_INVERT = true;
    
    public static final double TRANSPORT_FERRY_SPEED = 0.3;
    // From SysId
    // public static final double TOP_SHOOTER_FF_KS = 0.20823;
    // public static final double TOP_SHOOTER_FF_KV = 0.0021625;
    // public static final double TOP_SHOOTER_FF_KA = 0.00024932;
    // public static final double TOP_SHOOTER_PID_KP = 3.8326e-07;
    // public static final double TOP_SHOOTER_PID_KI = 0;
    // public static final double TOP_SHOOTER_PID_KD = 0;
    public static final double TOP_SHOOTER_FF_KS = 0.18454;
    public static final double TOP_SHOOTER_FF_KV = 0.0021629;
    public static final double TOP_SHOOTER_FF_KA = 0.00026348;
    public static final double TOP_SHOOTER_PID_KP = 4.4848e-07;
    public static final double TOP_SHOOTER_PID_KI = 0;
    public static final double TOP_SHOOTER_PID_KD = 0;

    // public static final double BOTTOM_SHOOTER_FF_KS = 0.22812;
    // public static final double BOTTOM_SHOOTER_FF_KV = 0.0022851;
    // public static final double BOTTOM_SHOOTER_FF_KA = 0.00026402;
    // public static final double BOTTOM_SHOOTER_PID_KP = 4.1584e-07;
    // public static final double BOTTOM_SHOOTER_PID_KI = 0;
    // public static final double BOTTOM_SHOOTER_PID_KD = 0;
    public static final double BOTTOM_SHOOTER_FF_KS = 0.19665;
    public static final double BOTTOM_SHOOTER_FF_KV = 0.0022763;
    public static final double BOTTOM_SHOOTER_FF_KA = 0.00033793;
    public static final double BOTTOM_SHOOTER_PID_KP = 7.1995e-07;
    public static final double BOTTOM_SHOOTER_PID_KI = 0;
    public static final double BOTTOM_SHOOTER_PID_KD = 0;

    public static final double TRANSPORT_FF_KS = 0.15883;
    public static final double TRANSPORT_FF_KV = 0.0020936;
    public static final double TRANSPORT_FF_KA = 0.00017895;
    public static final double TRANSPORT_PID_KP = 1.6005e-07;
    public static final double TRANSPORT_PID_KI = 0;
    public static final double TRANSPORT_PID_KD = 0;
  }

  public static class SprocketConstants {
    public static final double MAX_SPEED = 0.5;
    public static final double INTAKE_ANGLE = 52;
    // First number in meters, second in degrees
    public static final Point2D.Double[] ANGLE_POINTS = {
      new Point2D.Double(0, 0) // example
    };

    // First number in meters, second in RPM
    public static final Point2D.Double[] SPEED_POINTS = {};
  }

  public static class AutoConstants {
    public static final char[] NOTES = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
    public static final char[] SCORING_LOCATIONS = {'1', '2', '3', '4'};
    public static final char[] LANE3 = {'3', 'C', 'H', 'G', '4'};
    public static final char[] LANE2 = {'2', 'B', 'E', 'F'};
    public static final char[] LANE1 = {'1', 'A', 'D', 'E'};
    public static final char[] ALL_POINTS = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', '1', '2', '3', '4'};
    public static final char[] STARTING_POINTS = {'1', '2', '3', '4'};    
        


    public static final PIDConstants ANGULAR_PID_CONSTANTS = new PIDConstants(3.0, 0.0, 0.0);
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3.0, 0.0, 0.0);

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
        new HolonomicPathFollowerConfig(
            TRANSLATION_PID_CONSTANTS,
            ANGULAR_PID_CONSTANTS,
            DriveConstants.MAX_SPEED,
            DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig());

    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            DriveConstants.MAX_SPEED, DriveConstants.MAX_ACCELERATION,
            DriveConstants.MAX_ROT_SPEED, DriveConstants.MAX_ANGULAR_ACCELERATION);

    public static final double GOAL_END_VELOCITY = 0.0;
    public static final double ROTATION_DELAY_DISTANCE = 0.0;
    public static final double INTAKING_TIMEOUT = 3; // in seconds
    public static final double INTAKING_MOVE_SPEED = 0.04; // in M/s.
 


    /*
    |4| (AMP)
    *
    * \ 3        A      D
    *  #  2      B      G
    * / 1        C      F
    * 
    * |4| (AMP)
    * */
  }

  public static class LEDConstants {

    public static final double TRANSITION_TIME = 1;
    public static final double REEL_TIME = 10;

    public static final Animation DEMO_REEL =
        new AnimationReel(
            REEL_TIME,
            TRANSITION_TIME,
            new WipeTransition(),
            MagicAnimation.fire(),
            new CircusAnimation().randomized(),
            new RainbowAnimation().randomized(),
            MagicAnimation.galaxy(),
            new QuickSlowFlash(Color.kAquamarine),
            new RaceAnimation(Color.kIndigo).randomized());
  }
}
