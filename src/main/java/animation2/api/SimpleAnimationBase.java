package animation2.api;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class which implements a simple animation base with a timer to keep track of time.
 */
public abstract class SimpleAnimationBase implements Animation
{
    private Timer timer = new Timer();

    @Override
    public void start() 
    {
        timer.restart();
    }

    protected double getTimeElapsed() {return timer.get();}
    
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}