package animation2;

import edu.wpi.first.wpilibj.util.Color;

import animation2.api.AnimationBase;
import frc.robot.math.PerlinNoiseRing;
import frc.robot.math.Math555;

public class MagicAnimation extends AnimationBase
{
    public MagicAnimation(Color base, Color magic) 
    {
        noiseRing = new PerlinNoiseRing();

        this.base = base;
        this.magic = magic;
    }
    private Color base;
    private Color magic;

    private PerlinNoiseRing noiseRing;

    @Override
    public void render()
    {
        // Render each pixel
        for(int i = 0; i < getBuffer().getLength(); i++)
        {
            // Remap this value
            double heatRemap = noiseRing.get(getTimeElapsed(), i, getBuffer().getLength());

            // Perform the linear interpolations needed to get the hue and value,
            // using (0 -> 10) as the hue range and (0 -> 255) for the value range.
            // To change the color of the flames, modify these values.
            Color color = Math555.lerp(base, magic,  Math.pow(heatRemap, 4.0));

            // Push the pixel color to the led buffer
            getBuffer().setLED(i, color);
        }   
    } 

    public static MagicAnimation fire()
    {
        return new MagicAnimation(Color.kBlack, Color.kOrangeRed);
    }
    public static MagicAnimation galaxy()
    {
        return new MagicAnimation(new Color(25, 0, 150), Color.kDeepPink);
    }
}
