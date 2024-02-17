package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class AngleLookUp {
    public static double interpolate(Pose2d first, Pose2d second, double currentDistance) {
        return (first.getY() * (second.getX() - currentDistance) + second.getY() * (currentDistance - first.getX()))/(second.getX() - first.getX());
    }
}