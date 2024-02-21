package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimitSwitch;
import frc.robot.Constants.ClimberConstants;

public class Climbers extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final LimitSwitch topLimit;
    private final LimitSwitch bottomLimit;

    public Climbers() {
        leftMotor = new CANSparkMax(ClimberConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

        topLimit = new LimitSwitch(ClimberConstants.TOP_LIMIT_SWITCH_PORT, false);
        bottomLimit = new LimitSwitch(ClimberConstants.BOTTOM_LIMIT_SWITCH_PORT, false);
    }

    
}
