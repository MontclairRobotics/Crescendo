package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    public Climbers() {
        leftMotor = new CANSparkMax(Ports.CLIMBER_LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.CLIMBER_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        topLimit = new LimitSwitch(Ports.CLIMBER_TOP_LIMIT_SWITCH_PORT, false);
        bottomLimit = new LimitSwitch(Ports.CLIMBER_BOTTOM_LIMIT_SWITCH_PORT, false);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftEncoder.setPositionConversionFactor(1/ClimberConstants.ROTATIONS_PER_INCH); //Converts to inches per rotation
        rightEncoder.setPositionConversionFactor(1/ClimberConstants.ROTATIONS_PER_INCH);

        leftEncoder.setVelocityConversionFactor(1/ClimberConstants.ROTATIONS_PER_INCH);
        rightEncoder.setVelocityConversionFactor(1/ClimberConstants.ROTATIONS_PER_INCH);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void up() {
        leftMotor.set(ClimberConstants.CLIMBER_SPEED);
        rightMotor.set(ClimberConstants.CLIMBER_SPEED);
    }

    public void down() {
        leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
        rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
    
    public void periodic() {
        
        if (bottomLimit.get()) {
            stop();
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
        }

        else if (topLimit.get()) {
            stop();
            leftEncoder.setPosition(ClimberConstants.MAX_HEIGHT);
            rightEncoder.setPosition(ClimberConstants.MAX_HEIGHT);
        }
    }
}
