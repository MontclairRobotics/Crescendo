package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class AngleLookUp {
  public static Pose2d[] table = new Pose2d[10];

  public static double interpolate(
      Pose2d first, Pose2d second, double currentDistance) { // distance, angle
    return (first.getY() * (second.getX() - currentDistance)
            + second.getY() * (currentDistance - first.getX()))
        / (second.getX() - first.getX());
  }

  public static double findAngle(double currentDistance) {
    int mid = 0;
    int left = 0;
    int right = table.length - 1;
    while (left <= right) {
      mid = (left + right);
      if (table[mid].getX() == currentDistance) {
        return table[mid].getY();
      } else if (table[mid].getX() < currentDistance) {
        left = mid + 1;
      } else {
        right = mid - 1;
      }
    }
    return interpolate(table[mid - 1], table[mid + 1], currentDistance);
  }
}
