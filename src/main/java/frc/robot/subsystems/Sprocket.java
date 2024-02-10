package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.PIDMechanism;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.math.Math555;

import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax leftMotor = new CANSparkMax(Ports.LEFT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
     private final CANSparkMax rightMotor = new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    public final PIDController pidController = new PIDController(ArmConstants.angleKP.get(), ArmConstants.angleKI.get(), ArmConstants.angleKD.get());
    public final PIDMechanism pid;
    private ArmFeedforward angleFeedForward;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(BOTTOM_LIMIT_SWITCH, false);
    public LimitSwitch topLimitSwitch = new LimitSwitch(TOP_LIMIT_SWITCH, false);

    public Sprocket() {

        leftMotor.setInverted(LEFT_INVERT.get());
        rightMotor.setInverted(RIGHT_INVERT.get());

        ArmConstants.LEFT_INVERT.whenUpdate(leftMotor::setInverted);
        ArmConstants.RIGHT_INVERT.whenUpdate(rightMotor::setInverted);

        pid = new PIDMechanism(pidController);
        
        //TODO do we want to normalize this PID Controller?
        pid.disableOutputClamping();
        angleFeedForward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        Consumer<Double> whenUpdate = (p) -> angleFeedForward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        kS.whenUpdate(whenUpdate);
        kG.whenUpdate(whenUpdate);
        kV.whenUpdate(whenUpdate);
        kA.whenUpdate(whenUpdate);

        angleKD.whenUpdate(pidController::setD);
        angleKI.whenUpdate(pidController::setI);
        angleKP.whenUpdate(pidController::setP);



        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setPosition(ENCODER_MIN_ANGLE);

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(SPROCKET_ROTATIONS_PER_DEGREE);
        rightEncoder.setPosition(ENCODER_MIN_ANGLE);
    }
    /**
     * Move sprocket up
     */
    public void goUp() {
        pid.setSpeed(ANGLE_SPEED);
    }
    /**
     * Move sprocket down
     */
    public void goDown() {
        pid.setSpeed(-ANGLE_SPEED);
    }


    public void setSpeed(double speed) {
        pid.setSpeed(speed);
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
    // public Command goToAngle(double angle) {
    //     return pid.goToSetpoint(() -> angle, RobotContainer.sprocket);
    // }
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
        if(getEncoderPosition() < ENCODER_MIN_ANGLE) {
            return ENCODER_MIN_ANGLE;
        }
        else if(getEncoderPosition() > ENCODER_MAX_ANGLE) {
            return ENCODER_MAX_ANGLE;
        }
        return getEncoderPosition();
    }

    public double getEncoderPosition() {
        if (Math.abs(leftEncoder.getPosition() - rightEncoder.getPosition()) > ENCODER_DIFFERENCE) { //Should the arm auto-reset if this is not equal?
            System.out.println("Encoders misaligned!");
        }
        
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public boolean isAtAngle(double angle) { //TODO make this a constant
        return Math.abs(getAngle() - angle) < 3;
    }

    @Override
    /**
     * will this work if getAngle returns degrees? I do not know
     */
    public void periodic() {
        //will this work if getAngle returns degrees? I do not know - yes if its consistent with the units
        pid.setMeasurement(Math555.invlerp(getAngle(), ENCODER_MIN_ANGLE, ENCODER_MAX_ANGLE));
        pid.update();

        if (bottomLimitSwitch.get()) {
            stop();
            pid.cancel();
            leftEncoder.setPosition(ENCODER_MIN_ANGLE);
            rightEncoder.setPosition(ENCODER_MIN_ANGLE);
        }
        else if (topLimitSwitch.get()) {
            stop();
            pid.cancel();
            leftEncoder.setPosition(ENCODER_MAX_ANGLE);
            rightEncoder.setPosition(ENCODER_MAX_ANGLE);
        }
        
        //Calculate voltage from PID and Feedforward, then use .setVoltage since the voltages are significant
        //TODO is the output from the controller normalized?
        double voltage = Math555.clamp(pid.getSpeed() * MAX_VOLTAGE_V + FF_VOLTAGE.get(), -MAX_VOLTAGE_V, MAX_VOLTAGE_V); //TODO set clamping
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }
}

