package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.PIDMechanism;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax motor = new CANSparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);
    public final PIDController pidController = new PIDController(PidConstants.angleKP, PidConstants.angleKI, PidConstants.angleKD);
    public final PIDMechanism pid;
    private final double speed = SubsystemConstants.ANGLE_SPEED;
    RelativeEncoder encoder;

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(SubsystemConstants.BOTTOM_LIMIT_SWITCH, false);
    public LimitSwitch topLimitSwitch = new LimitSwitch(SubsystemConstants.BOTTOM_LIMIT_SWITCH, false);

    public Sprocket() {

        motor.setInverted(false);

        pid = new PIDMechanism(pidController);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(SubsystemConstants.SPROCKET_ROTATIONS_PER_DEGREE);
        encoder.setPosition(SubsystemConstants.ENCODER_MIN_ANGLE);
    }
    /**
     * Move sprocket up
     */
    public void goUp() {
        pid.setSpeed(speed);
    }
    /**
     * Move sprocket down
     */
    public void goDown() {
        pid.setSpeed(-speed);
    }
    /**
     * Stop sprocket
     */
    public void stop() {
        pid.setSpeed(0.0);
    }

    /**
     * Go to angle! Yay!
     */
    public Command goToAngle(double angle) {
        return pid.goToSetpoint(() -> angle, RobotContainer.sprocket);
    }
    /**
     * Stops PID from PIDDING
     */
    public void stopPID() {
        pid.cancel();
    }
    /**
     * Returns if PID is active
     */
    public boolean isPIDActive() {
        return pid.active();
    }
    @AutoLogOutput
    /**
     * If the angle is less than 0 then 0.0 is returned, if the angle is greater than 90 it return 90.0, else it will return the actual angle
     */
    public double getAngle() {
        if(encoder.getPosition() < 0) {
            return 0.0;
        }
        else if(encoder.getPosition() > 90) {
            return 90.0;
        }
        return encoder.getPosition();
    }

    public boolean isAtAngle(double angle) { //TODO make this a constant
        return Math.abs(getAngle() - angle) < 3;
    }

    @Override
    /**
     * will this work if getAngle returns degrees? I do not know
     */
    public void periodic() {
        pid.setMeasurement(getAngle());
        pid.update();

        if (bottomLimitSwitch.get()) {
            stop();
            pid.cancel();
            encoder.setPosition(SubsystemConstants.ENCODER_MIN_ANGLE);
        }
        else if (topLimitSwitch.get()) {
            stop();
            pid.cancel();
            encoder.setPosition(SubsystemConstants.ENCODER_MAX_ANGLE);
        }
        
        motor.set(pid.getSpeed() + SubsystemConstants.FF_VOLTAGE);
    }
}

