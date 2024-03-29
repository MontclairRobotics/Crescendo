package animation2;

import animation2.api.TimedAnimationBase;


public class ShootAnimation extends TimedAnimationBase{
    public static final double SHOOT_TIME = 1;
    @Override
    public void render() {
        final int offset = (int) (getBuffer().getLength() * (getTimeElapsed() / SHOOT_TIME % 1));
        
    }
}
