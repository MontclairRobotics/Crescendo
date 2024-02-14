package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.math.Tunable;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

    public final CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TOP_PORT, MotorType.kBrushless);
    public final CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_BOTTOM_PORT, MotorType.kBrushless);
    private final CANSparkMax transportMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT_PORT, MotorType.kBrushless);

    private Tunable<Double> transportSpeed = Tunable.of(.5, "transport speed");
    private Tunable<Double> topMotorSpeed = Tunable.of(.5, "top motor speed");
    private Tunable<Double> bottomMotorSpeed = Tunable.of(1, "bottom motor speed");

    
    public Shooter() {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();
        transportMotor.restoreFactoryDefaults();
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        transportMotor.setInverted(true);
    }
    
    
    /**
     * Shoots (used for speaker)
     */
    public void shootSpeaker() {
        topMotor.set(topMotorSpeed.get());
        bottomMotor.set(bottomMotorSpeed.get());
    }
    public void transport() {
        
        System.out.println(transportSpeed.get());
        transportMotor.set(transportSpeed.get());
    }
    public void transportSet(double x) {
        //System.out.println("transportSet: " + x);
        transportMotor.set(x);
    }
    public void stopTransport() {
        transportMotor.set(0);
    }
    /**
     * Shoots (used for amp)
     */
    public void shootAmp() {
        topMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
        bottomMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
    }

    /**
     * Stops the motors
     */
    public void stopShooter(){
        topMotor.set(0);
        bottomMotor.set(0);
    }
    /**
     * reverse shooter, in case shooter jams, etc.
     */ 
    public void reverseShooter() {
        topMotor.set(-SubsystemConstants.SPEAKER_EJECT_SPEED);
        bottomMotor.set(-SubsystemConstants.SPEAKER_EJECT_SPEED);
    }
}