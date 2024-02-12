package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

    public final CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TOP_PORT, MotorType.kBrushless);
    public final CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_BOTTOM_PORT, MotorType.kBrushless);
    public final CANSparkMax transportMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT_PORT, MotorType.kBrushless);

    private RelativeEncoder topEncoder = topMotor.getEncoder();
    private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    private SparkPIDController topController = topMotor.getPIDController();
    private SparkPIDController bottomController = bottomMotor.getPIDController();

    // private SimpleMotorFeedforward topMotorFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.TOP_SHOOTER_FF_KS, Constants.ShooterConstants.TOP_SHOOTER_FF_KV, Constants.ShooterConstants.TOP_SHOOTER_FF_KA);
    // private SimpleMotorFeedforward bottomMotorFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KS, Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KV, Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KA);


    public Shooter() {

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        transportMotor.setInverted(true);

        // topController.setP(Constants.ShooterConstants.TOP_SHOOTER_PID_KP);
        // // topController.setP(0.0);
        // topController.setI(Constants.ShooterConstants.TOP_SHOOTER_PID_KI);
        // topController.setD(Constants.ShooterConstants.TOP_SHOOTER_PID_KD);
        // topController.setOutputRange(-1, 1);

        // bottomController.setP(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KP);
        // // bottomController.setP(0.0);
        // bottomController.setI(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KI);
        // bottomController.setD(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KD);
        // bottomController.setOutputRange(-1, 1);

        // topEncoder.setPositionConversionFactor(getBottomVelocity())
        
        // topController.setI(ShooterConstants.ki.get());
        // topController.setP(ShooterConstants.kp.get());
        // topController.setD(ShooterConstants.kd.get());
        // topController.setFF(ShooterConstants.ff.get());

        // bottomController.setI(ShooterConstants.ki.get());
        // bottomController.setP(ShooterConstants.kp.get());
        // bottomController.setD(ShooterConstants.kd.get());
        // bottomController.setFF(ShooterConstants.ff.get());

        // ShooterConstants.ki.whenUpdate((ki) -> {
        //     topController.setI(ki);
        //     bottomController.setI(ki);
        // });

        // ShooterConstants.kp.whenUpdate((kp) -> {
        //     topController.setP(kp);
        //     bottomController.setP(kp);
        // });

        // ShooterConstants.kd.whenUpdate((kd) -> {
        //     topController.setD(kd);
        //     bottomController.setD(kd);
        // });

        // ShooterConstants.ff.whenUpdate((ff) -> {
        //     topController.setFF(ff);
        //     bottomController.setFF(ff);
        // });

        // topController.setOutputRange(-1, 1);
        // bottomController.setOutputRange(-1, 1);
    }
    
    /**
     * Shoots (used for speaker)
     */
    public void shootSpeaker() {
        topMotor.set(ShooterConstants.SPEAKER_EJECT_SPEED);
        bottomMotor.set(ShooterConstants.SPEAKER_EJECT_SPEED);
        transportMotor.set(SubsystemConstants.TRANSPORT_SPEED);
    }

    public void shootVelocity(double velocity) {
        System.out.println("shootVelocity: " + velocity);

        // double topFeedForward = topMotorFeedforward.calculate(velocity) / (12 * 6000);
        // double bottomFeedForward = bottomMotorFeedforward.calculate(velocity) / (12 * 6000);
        // topController.setFF(topFeedForward);
        // bottomController.setFF(bottomFeedForward);
        // System.out.println("TopFF: " + topFeedForward + " BottomFF: " + bottomFeedForward);

        topController.setReference(velocity, ControlType.kVelocity);
        bottomController.setReference(velocity, ControlType.kVelocity);

        System.out.println("Top: " + topEncoder.getVelocity() + " Bottom: " + bottomEncoder.getVelocity());
        System.out.println("TopV: " + topMotor.getAppliedOutput() + " BottomV: " + bottomMotor.getAppliedOutput());
    }

    public boolean isAtSetpoint(double setpoint) {
        return Math.abs(topEncoder.getVelocity() - setpoint) < ShooterConstants.VELOCITY_DEADBAND &&
        Math.abs(bottomEncoder.getVelocity() - setpoint) < ShooterConstants.VELOCITY_DEADBAND;
    }

    public void transportStart() {
        transportMotor.set(SubsystemConstants.TRANSPORT_SPEED);
    }

    public double getTopVelocity() {
        return topEncoder.getVelocity();
    }

    public double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }
    /**
     * Shoots (used for amp)
     */
    public void shootAmp() {
        topMotor.set(ShooterConstants.AMP_EJECT_SPEED);
        bottomMotor.set(ShooterConstants.AMP_EJECT_SPEED);
    }

    /**
     * Stops the motors
     */
    public void stop(){
        topMotor.set(0);
        bottomMotor.set(0);
    }
    /**
     * reverse shooter, in case shooter jams, etc.
     */ 
    public void reverseShooter() {
        topMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
        bottomMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
    }



    public Command shooterSysId(String type, SysIdRoutine.Direction direction) {
        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
        // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
        final MutableMeasure<Angle> rotations = mutable(Revolutions.of(0));
        // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
        final MutableMeasure<Velocity<Angle>> velocity = mutable(RevolutionsPerSecond.of(0));

        // Create a new SysId routine for characterizing the drive.
        final SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        topMotor.setVoltage(volts.in(Volts));
                        bottomMotor.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record data of each motor
                    log -> {

                        System.out.println("pos: " + topEncoder.getPosition() + " vel: " + topEncoder.getVelocity());
                        // Record data for top motor
                        log.motor("shooter-top")
                            .voltage(appliedVoltage.mut_replace(topMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(rotations.mut_replace(topEncoder.getPosition(), Rotations))
                            .angularVelocity(velocity.mut_replace(topEncoder.getVelocity(), RotationsPerSecond));

                        // Record data for bottom motor
                        log.motor("shooter-bottom")
                            .voltage(appliedVoltage.mut_replace(bottomMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(rotations.mut_replace(bottomEncoder.getPosition(), Rotations))
                            .angularVelocity(velocity.mut_replace(bottomEncoder.getVelocity(), RotationsPerSecond));
                    },
                // We are doing SysId on "this" subsystem
                this));

        if (type == "quasistatic") {
            return sysIdRoutine.quasistatic(direction);
        } else {
            return sysIdRoutine.dynamic(direction);
        }
    }
}