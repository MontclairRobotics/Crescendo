package animation2;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.Math555;
import animation2.api.AnimationBase;
import animation2.api.LEDBuffer;
import animation2.api.TimedAnimationBase;

public class QuickSlowFlash extends TimedAnimationBase
{
    private Color flashColor;
    private double proportionQuick;

    public QuickSlowFlash(Color color)
    {
        flashColor = color;
        proportionQuick = 0.33;
    }

    @Override
    public void render()
    {
        double percentFinished = getPercentComplete();
        Color color = flashColor;

        if (percentFinished < proportionQuick)
        {
            int state = Math555.repeatingCycle(percentFinished, 0, proportionQuick, 2, 200);
            color = Math555.lerp(Color.kBlack, flashColor, state / 200.0);
        }
        else 
        {
            int state = Math555.repeatingCycle(percentFinished, proportionQuick, 1, 2, 200);
            color = Math555.lerp(Color.kBlack, flashColor, state / 200.0);
        }

        LEDBuffer.fill(getBuffer(), color);
    }
    
}