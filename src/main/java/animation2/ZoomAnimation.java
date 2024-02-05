package animation2;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.Math555;
import animation2.api.AnimationBase;

public class ZoomAnimation extends AnimationBase
{
    public static final double WRAP_TIME = 0.5;
    private final Color col;

    public ZoomAnimation(Color col) 
    {
        this.col = col;
    }

    @Override
    public void render()
    {
        final int offset = (int)(getBuffer().getLength() * ((getTimeElapsed() / WRAP_TIME) % 1.0));

        for(int i = 0; i < getBuffer().getLength(); i++)
        {
            final int x = (offset + i) % getBuffer().getLength();
            final double t = i * 1.0 / getBuffer().getLength();

            getBuffer().setLED(x, Math555.lerp(Color.kBlack, col, t));
        }
    }
}
