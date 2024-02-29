package animation2;

import animation2.api.TransitionBase;
import frc.robot.math.Math555;

/**
 * An animation which fades the entire LED strip from one color to another. Used to transition from
 * two solid colors.
 */
public class FadeTransition extends TransitionBase {
  @Override
  public void render() {
    super.render();

    for (int i = 0; i < getBuffer().getLength(); i++) {
      getBuffer()
          .setLED(
              i,
              Math555.lerp(
                  getOut().getBuffer().getLED(i),
                  getIn().getBuffer().getLED(i),
                  getPercentComplete()));
    }
  }
}
