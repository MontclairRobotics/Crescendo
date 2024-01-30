package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class PIDMechanism implements Sendable
{
    private PIDController pidController;
    private boolean usingPID;
    private double measurement;
    private double speed;
    private double directSpeed;
    
    private boolean clampOutput = true;

    public PIDMechanism(PIDController pidController)
    {
        this.pidController = pidController;
    }

    /**
     * Get the PIDController for this mechanism
     * 
     * @return PIDController for this mechanism
     */
    public PIDController getPIDController()
    {
        return pidController;
    }

    public void disableOutputClamping()
    {
        clampOutput = false;
    }

    public double getTarget()
    {
        return pidController.getSetpoint();
    }

    /**
     * Set the target value (setPoint) that the mechanism moves to
     * 
     * @param target
     */
    public void setTarget(double target)
    {
        pidController.reset();
        updateTarget(target);
    }

    public void updateTarget(double target)
    {
        pidController.setSetpoint(target);
        usingPID = !pidController.atSetpoint();
    }

    /**
     * Stop using PID to determine speed
     */
    public void cancel()
    {
        usingPID = false;
    }

    /**
     * Set the measurement of where it currently is
     * @param measurement
     */
    public void setMeasurement(double measurement)
    {
        this.measurement = measurement;
    }

    public double getMeasurement()
    {
        return measurement;
    }

    /**
     * Set the speed manually (without PID)
     * @param speed
     */
    public void setSpeed(double speed) //duck you fylan
    {
        this.directSpeed = speed;
    }
    /**
     * Updates the calculated speed
     * <p>
     * <ul>
     * <li>if we are currently using PID, calculate using PID
     * <li>otherwise, use {@link PIDMechanism#setSpeed(double) setSpeed()} to set the speed manually
     * </ul>
     */
    public void update()
    {
        if(pidController.atSetpoint())
        {
            cancel();
        }

        if(usingPID)
        {
            // Logging.info("USIND PID BITCHES");
            double calulation = pidController.calculate(measurement);

            if(clampOutput)
            {
                //speed = Math555.clamp1(calulation);
                //We are now clamping manually because fun fun fun
                if(speed < -1) {
                    speed = -1.0;
                }
                else if(speed > 1) {
                    speed = 1.0;
                }
            }
            else 
            {
                speed = calulation;
            }
        }
        else 
        {
            // Logging.info("UNOT SIND PID BITCHES");
            speed = directSpeed;
        }

        if(pidController.atSetpoint() && usingPID)
        {
            cancel();
        }
    }
    /**
     * get the speed determined by the {@link PIDMechanism#update() update method}
     * @return speed
     */
    public double getSpeed() 
    {
        return speed;
    }

    /**
     * Get whether or not PID is used to calculate
     * <ul>
     *  <li> <code>true</code> if using PID
     *  <li> <code>false</code> if <b>not</b> using PID
     * </ul>
     * @return boolean
     */
    public boolean active()
    {
        return usingPID;
    }
    public boolean free()
    {
        return !usingPID;
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.addDoubleProperty ("P", pidController::getP, pidController::setP);
        builder.addDoubleProperty ("I", pidController::getI, pidController::setI);
        builder.addDoubleProperty ("D", pidController::getD, pidController::setD);
        builder.addDoubleProperty ("Setpoint", pidController::getSetpoint, pidController::setSetpoint);
        builder.addBooleanProperty("At Setpoint", pidController::atSetpoint, x -> usingPID = !x);
        builder.addDoubleProperty ("Speed", () -> speed, x -> speed = x);

        builder.setSmartDashboardType("ShuffleboardLayout");
    }

    public Command goToSetpoint(double target, Subsystem... requirements)
    {
        return goToSetpoint(() -> target, requirements);
    }

    public Command goToSetpoint(DoubleSupplier target, Subsystem... requirements)
    {
        /* Create an Anonymous Command cuz Command is an interface */
        return new Command()
        {
            // How commands work approximately (not actually)
            // initialize();
            // while(!isFinished)
            // {
            //    execute();
            // }
            // end();

            @Override
            public Set<Subsystem> getRequirements() 
            {
                return Set.of(requirements);
            }

            @Override
            public void initialize()
            {
                setTarget(target.getAsDouble());
            }

            @Override
            public void execute()
            {
                updateTarget(target.getAsDouble());
            }

            @Override
            public boolean isFinished()
            {
                return pidController.atSetpoint();
            }

            @Override
            public void end(boolean inter)
            {
                PIDMechanism.this.cancel();
            }
        };
    }
}


/* 
class MultiPIDMechanism
{
    PIDMechanism(Map<String, PIDController>) - constructor
    PIDController controller(String name)    - get the controller with the specified name

    void add(String name, PIDController) - add a new controller to this mechanism

    void target(String name, double) - target measurement for controller
    void cancel()                    - cancel all pidding

    void setMeasurement(String name, double) - set measurement for the named pid controller
    void setSpeed(double)                    - set speed directly. ignore if pidding

    void update() - update speed

    double get()     - get resultant speed
    boolean active() - get if any pidding is occurring

    String current() - get the name of the current controller, or 'null' if no pidding is ocurring
}
*/

























// hi