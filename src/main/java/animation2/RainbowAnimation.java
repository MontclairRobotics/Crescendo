package animation2;

import animation2.api.AnimationBase;

/**
 * An animation which creates a cycling rainbow over the entirety of the LED strip. Uses HSV Math in
 * order to ensure that the
 */
public class RainbowAnimation extends AnimationBase {
  public static final double CYCLE_TIME = 1.0;

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
      getBuffer().setHSV(i, hue, 255, 255);
    }
  }
}
