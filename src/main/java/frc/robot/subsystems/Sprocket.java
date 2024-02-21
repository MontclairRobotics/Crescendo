package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.Constants.*;

import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static frc.robot.Constants.ArmConstants.*;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax leftMotor = new CANSparkMax(Ports.LEFT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR_PORT, MotorType.kBrushless);

    private ArmFeedforward angleFeedForward;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    private final SparkPIDController leftController = leftMotor.getPIDController();
    private final SparkPIDController rightController = rightMotor.getPIDController();

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(BOTTOM_LIMIT_SWITCH, false);
    public LimitSwitch topLimitSwitch = new LimitSwitch(TOP_LIMIT_SWITCH, false);

    public Sprocket() {

        leftMotor.setInverted(LEFT_INVERT.get());
        rightMotor.setInverted(RIGHT_INVERT.get());

        ArmConstants.LEFT_INVERT.whenUpdate(leftMotor::setInverted);
        ArmConstants.RIGHT_INVERT.whenUpdate(rightMotor::setInverted);

        leftController.setD(angleKD.get(), 1);
        leftController.setP(angleKP.get(), 1);
        leftController.setI(angleKI.get(), 1);

        rightController.setD(angleKD.get(), 1);
        rightController.setP(angleKP.get(), 1);
        rightController.setI(angleKI.get(), 1);


        
        //TODO do we want to normalize this PID Controller?
        angleFeedForward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        Consumer<Double> whenUpdate = (p) -> angleFeedForward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        kS.whenUpdate(whenUpdate);
        kG.whenUpdate(whenUpdate);
        kV.whenUpdate(whenUpdate);
        kA.whenUpdate(whenUpdate);

        angleKD.whenUpdate((k) -> {leftController.setD(k, 1);});
        angleKI.whenUpdate((k) -> {leftController.setI(k, 1);});
        angleKP.whenUpdate((k) -> {leftController.setP(k, 1);});

        angleKD.whenUpdate((k) -> {rightController.setD(k, 1);});
        angleKI.whenUpdate((k) -> {rightController.setI(k, 1);});
        angleKP.whenUpdate((k) -> {rightController.setP(k, 1);});



        //TODO check conversion factors
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setPosition(ENCODER_MIN_ANGLE);

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        rightEncoder.setPosition(ENCODER_MIN_ANGLE);
    }
    /**
     * Move sprocket up
     */
    public void goUp() {
        leftMotor.set(ANGLE_SPEED);
        rightMotor.set(ANGLE_SPEED);
    }
    /**
     * Move sprocket down
     */
    public void goDown() {
        leftMotor.set(-ANGLE_SPEED);
        rightMotor.set(-ANGLE_SPEED);
    }


    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void setPosition(double angle) {
        double feedForward = angleFeedForward.calculate(angle, 0);

        leftController.setReference(angle * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition, 1, feedForward);
        rightController.setReference(angle * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition,1,  feedForward);
    }

    public SysIdRoutine getSysId() {
        MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
        MutableMeasure<Angle> degrees = MutableMeasure.mutable(Units.Degrees.of(0));
        MutableMeasure<Velocity<Angle>> motorVelocity = MutableMeasure.mutable(Units.DegreesPerSecond.of(0));
        

        return new SysIdRoutine(
            new Config(), new Mechanism(
                (Measure<Voltage> volts) -> {
                    leftMotor.setVoltage(volts.in(Units.Volts));
                    rightMotor.setVoltage(volts.in(Units.Volts));
                },
                (SysIdRoutineLog log) -> {
                    log.motor("Left")
                    .voltage(appliedVoltage.mut_replace(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), Units.Volts))
                    .angularPosition(degrees.mut_replace(getAngle(), Units.Degrees))
                    .angularVelocity(motorVelocity.mut_replace(leftMotor.getEncoder().getVelocity(), Units.DegreesPerSecond));

                    log.motor("Right")
                    .voltage(appliedVoltage.mut_replace(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(), Units.Volts))
                    .angularPosition(degrees.mut_replace(getAngle(), Units.Degrees))
                    .angularVelocity(motorVelocity.mut_replace(rightMotor.getEncoder().getVelocity(), Units.DegreesPerSecond));
                },
                this,
                "Sprocket"
            ));
    }

    /**
     * Stop sprocket
     */
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    /**
     * Go to angle! Yay!
     */
    public void goToAngle(double angle) {
        leftController.setReference(angle * SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition);
        rightController.setReference(angle * SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition);
    }

    public boolean isSprocketSafe() {
        return !(getAngle() <= ENCODER_MIN_ANGLE || getAngle() >= ENCODER_MAX_ANGLE);
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

    public boolean isAtAngle(double angle) { 
        return Math.abs(getAngle() - angle) < SubsystemConstants.SPROCKET_ANGLE_DEADBAND;
    }

    @Override
    /**
     * will this work if getAngle returns degrees? I do not know
     */
    public void periodic() {
        //will this work if getAngle returns degrees? I do not know - yes if its consistent with the units


        if (bottomLimitSwitch.get()) {
            stop();
            leftController.setReference(ArmConstants.BOTTOM_LIMIT_SWITCH * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition); //TODO this should be the limit switch position
            rightController.setReference(ArmConstants.BOTTOM_LIMIT_SWITCH * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition);
            leftEncoder.setPosition(ENCODER_MIN_ANGLE);
            rightEncoder.setPosition(ENCODER_MIN_ANGLE);
        }
        else if (topLimitSwitch.get()) {
            stop();
            leftController.setReference(ArmConstants.TOP_LIMIT_SWITCH * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition);
            rightController.setReference(ArmConstants.TOP_LIMIT_SWITCH * ArmConstants.SPROCKET_ROTATIONS_PER_DEGREE, ControlType.kPosition);
            leftEncoder.setPosition(ENCODER_MAX_ANGLE);
            rightEncoder.setPosition(ENCODER_MAX_ANGLE);
        }
        
        //Calculate voltage from PID and Feedforward, then use .setVoltage since the voltages are significant
        //TODO is the output from the controller normalized?
        // double voltage = Math555.clamp(pid.getSpeed() * MAX_VOLTAGE_V + FF_VOLTAGE.get(), -MAX_VOLTAGE_V, MAX_VOLTAGE_V); //TODO set clamping
        // leftMotor.setVoltage(voltage);
        // rightMotor.setVoltage(voltage);
    }
}

