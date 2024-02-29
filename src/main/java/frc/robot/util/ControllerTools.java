package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerTools {
  /**
   * Represents one of the D-Pad inputs of a controller. Can either be {@code UP}, {@code DOWN},
   * {@code LEFT}, or {@code RIGHT}
   */
  public static enum DPad {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    private DPad(int angle) {
      this.angle = angle;
    }

    public final int angle;

    public static boolean get(DPad type, int pov) {
      switch (type) {
        case UP:
          return (0 <= pov && pov <= 45) || (360 - 45 <= pov && pov <= 360);
        case RIGHT:
          return (90 - 45 <= pov && pov <= 90 + 45);
        case DOWN:
          return (180 - 45 <= pov && pov <= 180 + 45);
        case LEFT:
          return (270 - 45 <= pov && pov <= 270 + 45);
      }

      throw new RuntimeException("Unknown dpad type " + type);
    }
  }

  public static Trigger getDPad(DPad type, CommandPS5Controller controller) {
    return new Trigger(() -> DPad.get(type, controller.getHID().getPOV()));
  }
}
