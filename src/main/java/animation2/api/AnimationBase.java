package animation2.api;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED;

/** A class which implements a simple animation that contains its own buffer. */
public abstract class AnimationBase extends SimpleAnimationBase {
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED.LED_COUNT);

  @Override
  public AddressableLEDBuffer getBuffer() {
    return buffer;
  }
}
