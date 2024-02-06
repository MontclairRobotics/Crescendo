package frc.robot.animation3;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotContainer;

public class Animations {

    public static Runnable allianceAnimation() {
        return () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
        
            Color color = Color.kDimGray;

            if (alliance.get() == Alliance.Blue) color = Color.kBlue; 
            else if (alliance.get() == Alliance.Red) color = Color.kRed;

            RobotContainer.led.fill(color);
        };
    }

    public static Runnable rainbowAnimation() {
        return () -> {
            final int offset = (int)(LedConstants.BUFFER_SIZE * Timer.getFPGATimestamp());
            for (int i = 0; i < LedConstants.BUFFER_SIZE; i++) {
                int hue = ((offset + i) * 180 / LedConstants.BUFFER_SIZE) % 180;
                RobotContainer.led.setHSV(i, hue, 255, 255);
            }
        };
    }
}
