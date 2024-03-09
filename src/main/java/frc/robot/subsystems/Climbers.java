package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.Ports;
import frc.robot.util.LimitSwitch;

public class Climbers extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final LimitSwitch leftLimit;
  private final LimitSwitch rightLimit;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  
  //public boolean inMotion = false; 
  public boolean wentUpLast = false;
  public boolean atBottom = true;
  public boolean atTop = false;

  /** Creates objects for the motors, limits, and encoders Then sets the encoders */
  public Climbers() {
    leftMotor = new CANSparkMax(Ports.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Ports.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftLimit = new LimitSwitch(Ports.CLIMBER_LEFT_LIMIT_SWITCH_PORT, false);
    rightLimit = new LimitSwitch(Ports.CLIMBER_RIGHT_LIMIT_SWITCH_PORT, false);
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(
        ClimberConstants.ROTATIONS_PER_INCH); // Converts to inches per rotation
    rightEncoder.setPositionConversionFactor(ClimberConstants.ROTATIONS_PER_INCH);

    // leftEncoder.setVelocityConversionFactor(1 / ClimberConstants.ROTATIONS_PER_INCH);
    // rightEncoder.setVelocityConversionFactor(1 / ClimberConstants.ROTATIONS_PER_INCH);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    Shuffleboard.getTab("Debug").addDouble("Climber Height", () -> getHeight());
    Shuffleboard.getTab("Debug").addBoolean("At Top", () -> atTop());
    Shuffleboard.getTab("Debug").addBoolean("At Bottom", () -> atBottom());
  }

  public boolean atTop() {
    return getLimits() && getHeight() > 15;
  }

  public boolean atBottom() {
    return getLimits() && getHeight() < 5;
  }

  /** Makes the climber go up */
  public void up() {
    //inMotion = true;
    // if(!(getLimits() && wentUpLast)) {
    //   leftMotor.set(ClimberConstants.CLIMBER_SPEED);
    //   rightMotor.set(ClimberConstants.CLIMBER_SPEED);
    // } else {
    //   //inMotion = false;
    //   wentUpLast = true;
    //   atTop = true;
    //   atBottom = false;
    //   stop();
    // }

    if (atTop()) {
      stop();
    } else {
      leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
      rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }
  }

  public double getHeight() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }
  /** Climber arm goes down */
  public void down() {
    
    //inMotion = true;
    // if(!(getLimits() && !wentUpLast)) {
    //   leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
    //   rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
    // } else {
    //   //inMotion = false;
    //   wentUpLast = false;
    //   atBottom = true;
    //   atTop = false;
    //   stop();
    // }

    if (atBottom()) {
      stop();
    } else {
      leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
      rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }
  }

  /** Stops The Climbers */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
    //inMotion = false;
  }

  /** gets the limits */
  public boolean getLimits() {
    if(rightLimit.get() || leftLimit.get() ) {
      return true;
    }
    return false;
  }



  /** If the arm reaches the bottom limit, it will stop Same for top */
  public void periodic() {

  }
}
