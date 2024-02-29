package animation2;

import animation2.api.AnimationBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.Math555;

public class CycleAnimation extends AnimationBase {
  private Color[] colors;

  public CycleAnimation(Color... colors) {
    this.colors = colors;
  }

  @Override
  public void render() {
    for (int i = 0; i < getBuffer().getLength(); i++) {
      int variable = (int) (i + getTimeElapsed() * 10) % getBuffer().getLength();
      int n =
          Math555.repeatingCycle(
              variable,
              0,
              getBuffer().getLength(),
              getBuffer().getLength() / colors.length,
              colors.length);
      getBuffer().setLED(i, colors[n]);
    }
  }
}
