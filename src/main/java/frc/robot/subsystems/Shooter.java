package frc.robot.subsystems;

import frc.robot.Constants.*;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

    private final CANSparkMax motor1 = new CANSparkMax(Ports.SHOOTER_MOTOR_1_PORT, MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(Ports.SHOOTER_MOTOR_2_PORT, MotorType.kBrushless);
    
    public Shooter() {
        
        motor1.setInverted(false);
        motor2.setInverted(false);
    }
    
    // shoot
    public void shoot() {
        motor1.set(SubsystemConstants.EJECT_SPEED);
        motor2.set(SubsystemConstants.EJECT_SPEED);
    }

    // stop shooting
    public void stop(){
        motor1.set(0);
        motor2.set(0);
    }
    // reverse shooter, in case shooter jams, etc.
    public void reverseShooter() {
        motor1.set(-SubsystemConstants.EJECT_SPEED);
        motor2.set(-SubsystemConstants.EJECT_SPEED);
    } 
}