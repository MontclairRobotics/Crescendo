package frc.robot.subsystems;

import frc.robot.Constants.*;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax motor = new CANSparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);

    public Sprocket() {
        motor.setInverted(false);
    }
    // Move sprocket up
    public void goUp() {
        motor.set(SubsystemConstants.ANGLE_MOVE_SPEED);
    }
    // Move sprocket down
    public void goDown() {
        motor.set(-SubsystemConstants.ANGLE_MOVE_SPEED);
    }
    // Stop sprocket
    public void stop() {
        motor.set(0);
    }
}