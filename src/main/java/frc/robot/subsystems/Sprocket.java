package frc.robot.subsystems;

import frc.robot.Constants.*;
<<<<<<< Updated upstream

=======
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Tunable;
>>>>>>> Stashed changes

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sprocket extends SubsystemBase {
    
<<<<<<< Updated upstream
    private final CANSparkMax motor = new CANSparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);
=======
    private final CANSparkMax leftMotor = new CANSparkMax(Ports.LEFT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    private Tunable<Double> speed = Tunable.of(0.1, "Sprocket Speed");
    private ArmFeedforward angleFeedForward;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    private final SparkPIDController leftController = leftMotor.getPIDController();
    private final SparkPIDController rightController = rightMotor.getPIDController();

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(BOTTOM_LIMIT_SWITCH, false);
    public LimitSwitch topLimitSwitch = new LimitSwitch(TOP_LIMIT_SWITCH, false);
>>>>>>> Stashed changes

    public Sprocket() {
        motor.setInverted(false);
    }
    // Move sprocket up
    public void goUp() {
<<<<<<< Updated upstream
        motor.set(SubsystemConstants.ANGLE_MOVE_SPEED);
=======
        leftMotor.set(speed.get());
        rightMotor.set(speed.get());
>>>>>>> Stashed changes
    }
    // Move sprocket down
    public void goDown() {
<<<<<<< Updated upstream
        motor.set(-SubsystemConstants.ANGLE_MOVE_SPEED);
=======
        leftMotor.set(-speed.get());
        rightMotor.set(-speed.get());
>>>>>>> Stashed changes
    }
    // Stop sprocket
    public void stop() {
        motor.set(0);
    }
}