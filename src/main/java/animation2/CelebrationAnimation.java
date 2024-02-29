package animation2;

import animation2.api.TimedAnimationBase;
import frc.robot.math.Math555;

public class CelebrationAnimation extends TimedAnimationBase {
  public static final double CYCLE_TIME = 0.5;

  @Override
  public void render() {
    // Calculate strip offset
    final int offset = (int) (getBuffer().getLength() * (getTimeElapsed() / CYCLE_TIME % 1));

    // For every pixel
    for (int i = 0; i < getBuffer().getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = ((offset + i) * 180 / getBuffer().getLength()) % 180;
      // Set the value
      getBuffer().setHSV(i, hue, 255, Math555.lerp(255, 0, Math.pow(getPercentComplete(), 1.5)));
    }
  }
}
