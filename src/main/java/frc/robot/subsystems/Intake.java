package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor1 = new CANSparkMax(Ports.INTAKE_MOTOR_1_PORT, MotorType.kBrushless); 
    private final CANSparkMax intakeMotor2 = new CANSparkMax(Ports.INTAKE_MOTOR_2_PORT, MotorType.kBrushless);

    /**
     * Accelerates motors to intake something
     */
    public void in() {
        intakeMotor1.set(SubsystemConstants.INTAKE_SPEED); 
        intakeMotor2.set(SubsystemConstants.INTAKE_SPEED);
    }
    /**
     * Reverse intake if gamepiece gets stuck
     */
    public void out() {
        intakeMotor1.set(-SubsystemConstants.INTAKE_SPEED);
        intakeMotor2.set(-SubsystemConstants.INTAKE_SPEED);
    }
    
    /**
     * Stop intaking
     */
    public void stop() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }

}