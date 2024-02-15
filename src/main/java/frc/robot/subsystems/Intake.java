package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Tunable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor1 = new CANSparkMax(Ports.INTAKE_MOTOR_1_PORT, MotorType.kBrushless); 
    private final CANSparkMax intakeMotor2 = new CANSparkMax(Ports.INTAKE_MOTOR_2_PORT, MotorType.kBrushless);
    //private final DigitalInput beamBreak = new DigitalInput(Ports.BEAM_BREAK_CHANNEL);
    private Tunable<Double> intakeSpeed = Tunable.of(SubsystemConstants.INTAKE_SPEED, "Shooter/Top Motor/Speaker Eject Speed");
    /**
     * Accelerates motors to intake something
     */
    public void in() {
        intakeMotor1.set(intakeSpeed.get()); 
        intakeMotor2.set(intakeSpeed.get());
    }

    public void shootWithSpeed(double speed) {
        intakeMotor1.set(speed);
        intakeMotor2.set(speed);
    }
    /**
     * Reverse intake if gamepiece gets stuck
     */
    public void out() {
        intakeMotor1.set(-intakeSpeed.get());
        intakeMotor2.set(-intakeSpeed.get());
    }
    
    /**
     * Stop intaking
     */
    public void stop() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }

    /**
     * gets the state of the beam break sensor in the intake
     * @return true if the sensor is broken (gamepiece intaked), false if unbroken
     */
    // public boolean getSensor() {
    //     return !beamBreak.get();
    // }

}