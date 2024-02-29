package animation2;

import animation2.api.AnimationBase;
import animation2.api.LEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;

/**
 * The default animation for the LEDs. Takes the current alliance color from the driver station and
 * sets the LEDs to the correct color.
 */
public class AllianceAnimation extends AnimationBase {
  @Override
  public void render() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Color color = Color.kDimGray;

    if (alliance.get() == Alliance.Blue) color = Color.kBlue;
    else if (alliance.get() == Alliance.Red) color = Color.kRed;

    LEDBuffer.fill(getBuffer(), color);
  }
}
