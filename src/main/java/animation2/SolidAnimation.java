package animation2;

import animation2.api.AnimationBase;
import animation2.api.LEDBuffer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An animation which simply fills the led buffer with a provided color for some amount of time.
 */
public class SolidAnimation extends AnimationBase
{
    private Color color;
    
    public SolidAnimation(Color color)
    {
        this.color = color;
    }

    @Override
    public void render()
    {
        LEDBuffer.fill(getBuffer(), color);
    }
}
