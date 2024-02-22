package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimitSwitch;
import frc.robot.Constants.*;
import frc.robot.Constants.Ports;

public class Climbers extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final LimitSwitch topLimit;
    private final LimitSwitch bottomLimit;

    public Climbers() {
        leftMotor = new CANSparkMax(Ports.CLIMBER_LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.CLIMBER_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        topLimit = new LimitSwitch(Ports.CLIMBER_TOP_LIMIT_SWITCH_PORT, false);
        bottomLimit = new LimitSwitch(Ports.CLIMBER_BOTTOM_LIMIT_SWITCH_PORT, false);
    }

    public void up() {
        leftMotor.set(SubsystemConstants.CLIMBER_SPEED);
        rightMotor.set(SubsystemConstants.CLIMBER_SPEED);
    }

    public void down() {
        leftMotor.set(-SubsystemConstants.CLIMBER_SPEED);
        rightMotor.set(-SubsystemConstants.CLIMBER_SPEED);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
    
    public void periodic() {
        
        if (bottomLimit.get()) {
            stop();
        }

        else if (topLimit.get()) {
            stop();
        }

    }

}
