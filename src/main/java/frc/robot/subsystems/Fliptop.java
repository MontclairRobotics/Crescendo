package frc.robot.subsystems;

import frc.robot.Constants.*;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Fliptop extends SubsystemBase {
    
    private final CANSparkMax motor = new CANSparkMax(Ports.FLIPTOP_MOTOR_PORT, MotorType.kBrushless);

    public Fliptop() {
        motor.setInverted(false);
    }
    
    // Move fliptop forwards
    public void forward() {
        motor.set(SubsystemConstants.FLIPTOP_SPEED);
    }
    
    // Move fliptop backwards
    public void backward() {
        motor.set(-SubsystemConstants.FLIPTOP_SPEED);
    }
    
    
}