package animation2;

import animation2.api.AnimationBase;
import edu.wpi.first.wpilibj.util.Color;

public class CircusAnimation extends AnimationBase {
  public static final double SHIFT_SPEED = 0.1;
  private int offset = 0;

  public void render() {
    offset = (int) (getTimeElapsed() / SHIFT_SPEED);

    for (int i = 0; i < getBuffer().getLength(); i++) {
      if ((i + offset) % 3 == 0) {
        getBuffer().setLED(i, Color.kBlack);
      } else {
        getBuffer().setLED(i, Color.kCoral);
      }
    }
  }
}
