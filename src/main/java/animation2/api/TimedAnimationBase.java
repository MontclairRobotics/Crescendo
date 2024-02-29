package animation2.api;

/**
 * Implements an animation which depends on its run-length and can only end when its run-length has
 * been elapsed.
 */
public abstract class TimedAnimationBase extends AnimationBase {
  private double length;

  public double getLength() {
    return length;
  }

  public void setLength(double length) {
    this.length = length;
  }

  public TimedAnimationBase length(double length) {
    setLength(length);
    return this;
  }

  public double getPercentComplete() {
    return getTimeElapsed() / getLength();
  }

  @Override
  public final boolean isFinished() {
    return getTimeElapsed() >= length;
  }
}
