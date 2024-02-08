package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Tunable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

    private final CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TOP_PORT, MotorType.kBrushless);
    private final CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_BOTTOM_PORT, MotorType.kBrushless);
    private final CANSparkMax transportMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT_PORT, MotorType.kBrushless);

    // private Tunable<Double> topMotorAmpEjectSpeed = Tunable.of(SubsystemConstants.SPEAKER_EJECT_SPEED, "Shooter/Top Motor/Amp Eject Speed");
    private Tunable<Double> topMotorSpeakerEjectSpeed = Tunable.of(SubsystemConstants.SPEAKER_EJECT_SPEED, "Shooter/Top Motor/Speaker Eject Speed");
    // private Tunable<Double> bottomMotorAmpEjectSpeed = Tunable.of(SubsystemConstants.SPEAKER_EJECT_SPEED, "Shooter/Bottom Motor/Amp Eject Speed");
    private Tunable<Double> bottomMotorSpeakerEjectSpeed = Tunable.of(SubsystemConstants.SPEAKER_EJECT_SPEED, "Shooter/Bottom Motor/Speaker Eject Speed");
    private Tunable<Double> transportMotorTransportSpeed = Tunable.of(SubsystemConstants.SPEAKER_EJECT_SPEED, "Shooter/Transport Motor/Transport Speed");

    public Shooter() {
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        transportMotor.setInverted(true);
    }
    
    /**
     * Shoots (used for speaker)
     */
    public void shootSpeaker() {
        topMotor.set(topMotorSpeakerEjectSpeed.get());
        bottomMotor.set(bottomMotorSpeakerEjectSpeed.get());
        // transportMotor.set(SubsystemConstants.TRANSPORT_SPEED);
    }
    /**
     * Shoots (used for amp)
     */
    public void shootAmp() {
        topMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
        bottomMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
    }

    public void transportWithSpeed(double desiredSpeed) {
        transportMotor.set(desiredSpeed);
    }
    public void transport() {
        transportMotor.set(0.9);
    }
    public void stopTransport() {
        transportMotor.set(0);
    }
    /**
     * Stops the motors
     */
    public void stop(){
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