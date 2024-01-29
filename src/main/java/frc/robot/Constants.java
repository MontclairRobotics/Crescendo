// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
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
  public static class PathPlannerConstants {
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
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
    public static final double EJECT_SPEED = 0.5;
    public static final double ANGLE_MOVE_SPEED = 0.5;
  }
}
