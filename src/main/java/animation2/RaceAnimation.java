package animation2;

import animation2.api.TimedAnimationBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.Math555;

public class RaceAnimation extends TimedAnimationBase {
  final Color bgCol;

  public RaceAnimation(Color color) {
    this.bgCol = color;
  }

  @Override
  public void render() {
    int n = Math555.repeatingCycle(getPercentComplete(), 0, 1, 10, getBuffer().getLength());

    for (int i = 0; i < getBuffer().getLength(); i++) {
      Color color;

      if (i >= n - 1 && i <= n + 1) color = Color.kAqua;
      else color = bgCol;

      getBuffer().setLED(i, color);
    }
  }
}
