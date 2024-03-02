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

    public static final double BUMPER_WIDTH = Units.inchesToMeters(2.75); // TOOD get real width

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
    public static final double ANGLE_DEADBAND = 2;
  }

  public static class VisionConstants {
    public static final double TARGET_DEBOUNCE_TIME = 0.2;
    public static final double SHOOTER_LIMELIGHT_HEIGHT = 0;
    public static final double INTAKE_LIMELIGHT_HEIGHT = 0;
    public static final double SPEAKER_APRILTAG_HEIGHT = 0;
    public static final double SHOOTER_LIMELIGHT_ANGLE_DEGREES = 0;
    public static final double INTAKE_LIMELIGHT_ANGLE_DEGREES = 0;
  }

  public static class Ports { // TODO: add correct ports
    public static final int LED_PWM_PORT = 9;

    public static final int FLIPTOP_MOTOR_PORT = 101;

    public static final int INTAKE_TOP_MOTOR = 21;
    public static final int INTAKE_BOTTOM_MOTOR = 20;

    // TODO get actual ports from other code
    public static final int SHOOTER_MOTOR_BOTTOM_PORT = 40;
    public static final int SHOOTER_MOTOR_TOP_PORT = 41;
    public static final int SHOOTER_MOTOR_TRANSPORT_PORT = 42;

    // Sprocket motors
    public static final int LEFT_ANGLE_MOTOR_PORT = 30;
    public static final int RIGHT_ANGLE_MOTOR_PORT = 31;

    // TODO get ports
    public static final int INTAKE_BEAM_BREAK_CHANNEL = 5;

    public static final int SPROCKET_BEAM_BREAK_CHANNEL = 1;

    // Climber ports
    public static final int CLIMBER_LEFT_MOTOR_PORT = 48;
    public static final int CLIMBER_RIGHT_MOTOR_PORT = 49;

    public static final int CLIMBER_TOP_LIMIT_SWITCH_PORT = 8;
    public static final int CLIMBER_BOTTOM_LIMIT_SWITCH_PORT = 9;
  }

  public static class ClimberConstants {
    // Climbers
    public static final double CLIMBER_SPEED = 1;
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
    public static final int BOTTOM_LIMIT_SWITCH = 0;
    public static final boolean SPROCKET_BEAM_INVERT = true;
    public static final int TOP_LIMIT_SWITCH = 1;

    public static final Tunable<Double> FF_VOLTAGE =
        Tunable.of(0.6, "arm.feedforwardvoltage"); // TODO: stolen from
    // ChargedUp elevator
    // feedforward }

    // public static final Tunable<Double> kS = Tunable.of(0, "arm.feedforward.ks");
    // public static final Tunable<Double> kG = Tunable.of(0.1, "arm.feedforward.kG");
    // public static final Tunable<Double> kV = Tunable.of(8.91, "arm.feedforward.kV");
    // public static final Tunable<Double> kA = Tunable.of(0.01, "arm.feedforward.ka");

    // TODO: Angle PID Constants: (!!!!!!)
    public static final Tunable<Double> angleKP = Tunable.of(0.8, "arm.kp");
    public static final Tunable<Double> angleKI = Tunable.of(0, "arm.ki");
    public static final Tunable<Double> angleKD = Tunable.of(0, "arm.kd");

    // TODO: All of these constants are basically guessed!!
    public static final double ANGLE_SPEED = 0.5; // Speed of the angle changing
    public static final double SPROCKET_ROTATIONS_PER_DEGREE = 1.26984126984;
    public static final double ENCODER_MIN_ANGLE = 32;
    public static final double ENCODER_MAX_ANGLE = 75.787;

    public static final double SPEAKER_SCORE_ANGLE = -1;
    public static final double AMP_SCORE_ANGLE = -1;

    public static final double MAX_SPEED = 20; // TODO ???

    public static final Tunable<Boolean> LEFT_INVERT = Tunable.of(true, "arm.invert.left");
    public static final Tunable<Boolean> RIGHT_INVERT = Tunable.of(true, "arm.invert.right");

    public static final double ENCODER_DIFFERENCE =
        2; // the acceptable difference between encoder values

    // Sprocket
    // TODO: Validate values
    public static final double SPROCKET_ANGLE_MOVE_SPEED = 0.5;
    public static final double SPROCKET_ANGLE_DEADBAND = 2;
    public static final double SPROCKET_ANGLE_LIMIT_DEADBAND = 2.5;
  }

  public static class ShooterConstants {
    // Shooter
    public static final double SPEAKER_EJECT_SPEED = 0.5;
    public static final double AMP_EJECT_SPEED = 0.1;

    public static Tunable<Double> kp = Tunable.of(0.0005, "shooter.kp");
    public static Tunable<Double> ki = Tunable.of(0, "shooter.ki");
    public static Tunable<Double> kd = Tunable.of(0, "shooter.kd");
    public static Tunable<Double> ff = Tunable.of(0.0, "shooter.ff");

    public static final double MAX_RPM = 5700;

    public static final int VELOCITY_DEADBAND = 10;

    // Transport
    public static final double TRANSPORT_SPEED = 0.6;

    // // Shooter
    // public static final double SPEAKER_EJECT_SPEED = 0.5;
    // public static final double AMP_EJECT_SPEED = 0.1;

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

    // First number in meters, second in degrees
    public static final Point2D.Double[] ANGLE_POINTS = {
      new Point2D.Double(0, 0) // example
    };

    // First number in meters, second in RPM
    public static final Point2D.Double[] SPEED_POINTS = {};
  }

  public static class PidConstants {
    // TODO: Accurate PID constants
    public static final double angleKP = 0.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    // Pathplanner PID constants
    public static final PIDConstants ANGULAR_PID_CONSTANTS = new PIDConstants(3.0, 0.0, 0.0);
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3.0, 0.0, 0.0);
  }

  public static class AutoConstants {
    public static final Set<Character> NOTES =
        Arrays.stream(new Character[] {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'})
            .collect(Collectors.toSet());
    public static final Set<Character> SCORING_LOCATIONS =
        Arrays.stream(new Character[] {'1', '2', '3', '4'}).collect(Collectors.toSet());
    public static final Set<Character> LANE3 =
        Arrays.stream(new Character[] {'A', '3', 'D', 'E'}).collect(Collectors.toSet());
    public static final Set<Character> LANE2 =
        Arrays.stream(new Character[] {'B', '2', 'F', 'G', 'E'}).collect(Collectors.toSet());
    public static final Set<Character> LANE1 =
        Arrays.stream(new Character[] {'C', '1', '4', 'G', 'H'}).collect(Collectors.toSet());
    public static final Set<Character> ALL_POINTS =
        Arrays.stream(new Character[] {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', '1', '2', '3', '4'})
            .collect(Collectors.toSet());
    public static final Set<Character> STARTING_POINTS =
        Arrays.stream(new Character[] {'1', '2', '3'}).collect(Collectors.toSet());

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
        new HolonomicPathFollowerConfig(
            PidConstants.TRANSLATION_PID_CONSTANTS,
            PidConstants.ANGULAR_PID_CONSTANTS,
            Constants.DriveConstants.MAX_SPEED,
            Constants.DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig());

    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            DriveConstants.MAX_SPEED, DriveConstants.MAX_ACCELERATION,
            DriveConstants.MAX_ROT_SPEED, DriveConstants.MAX_ANGULAR_ACCELERATION);

    public static final double GOAL_END_VELOCITY = 0.0;
    public static final double ROTATION_DELAY_DISTANCE = 0.0;
    /*
     * |X|(AMP)
     *
     * \ 3 A | D
     * \ |
     * SPKR| 2 B | E
     * / /| |
     * / 1 C \| | F
     * |
     * | G
     * |
     * | H
     * (human player)
     */
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
