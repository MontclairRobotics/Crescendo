package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.math.MathUtils;
import frc.robot.math.Tunable;
import frc.robot.subsystems.Shooter;

/**
 * Command that shoots the motor and maintances velocity using a PID controller for a certain amount of time
 */
public class ShootPIDCommand extends Command {
    // The shooter
    private Shooter shooter;

    // Speeds
    private DoubleSupplier topSpeed;
    private DoubleSupplier bottomSpeed;
    
    // PID controllers
    private PIDController topPIDController;
    private PIDController bottomPIDController;

    // PID settings
    private Tunable<Double> kP = Tunable.of(Constants.ShootCommandConstants.PID.P, "ShootCommand/PID/p");
    private Tunable<Double> kI = Tunable.of(Constants.ShootCommandConstants.PID.I, "ShootCommand/PID/d");
    private Tunable<Double> kD = Tunable.of(Constants.ShootCommandConstants.PID.D, "ShootCommand/PID/i");
    private Tunable<Double> kIz = Tunable.of(Constants.ShootCommandConstants.PID.D, "ShootCommand/PID/iz");
    
    // How long to run the shooter
    private Tunable<Double> time = Tunable.of(Constants.ShootCommandConstants.TIME, "ShootCommand/Time");
    private Timer timer;

    // Should shooter use feed forward
    private Tunable<Boolean> useFeedForward = Tunable.of(Constants.ShootCommandConstants.USE_FEED_FORWARD, "ShootCommand/PID/i");    
    


    /**
     * Creates a new shoot command
     * 
     * @param topSpeed Top speed in RPMs
     * @param bottomSpeed Bottom speed in RPMs
     * @param shooter The shooter
     */
    public ShootPIDCommand(DoubleSupplier topSpeed, DoubleSupplier bottomSpeed, Shooter shooter) {
        this.shooter = shooter;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;

        timer = new Timer();

        topPIDController = new PIDController(kP.get(), kI.get(), kD.get());
        topPIDController.setIZone(kIz.get());
        kP.whenUpdate(topPIDController::setP);
        kI.whenUpdate(topPIDController::setI);
        kD.whenUpdate(topPIDController::setD);
        kIz.whenUpdate(topPIDController::setIZone);

        bottomPIDController = new PIDController(kP.get(), kI.get(), kD.get());
        bottomPIDController.setIZone(kIz.get());
        kP.whenUpdate(bottomPIDController::setP);
        kI.whenUpdate(bottomPIDController::setI);
        kD.whenUpdate(bottomPIDController::setD);
        kIz.whenUpdate(bottomPIDController::setIZone);

        addRequirements(shooter);
    }
    /**
     * Creates a new shoot command
     * 
     * @param topSpeed Top speed in RPMs
     * @param bottomSpeed Bottom speed in RPMs
     * @param shooter The shooter
     */
    public ShootPIDCommand(Double topSpeed, Double bottomSpeed, Shooter shooter) {
        this(() -> topSpeed, () -> bottomSpeed, shooter);
    }


    /**
     * Calculcates feed forward
     * Feed forward is just the speed we want to go
     */
    protected double getTopFeedForward() {
        return useFeedForward.get() ? topSpeed.getAsDouble() : 0;
    }
    protected double getBottomFeedForward() {
        return useFeedForward.get() ? bottomSpeed.getAsDouble() : 0;
    }


    /**
     * Called when command starts
     * Initializes PIDs
     * Starts timer
     */
    @Override
    public void initialize() {
        topPIDController.reset();
        bottomPIDController.reset();

        topPIDController.setSetpoint(this.topSpeed.getAsDouble());
        bottomPIDController.setSetpoint(this.bottomSpeed.getAsDouble());

        timer.start();
    }

    /**
     * Update speeds periodically
     */
    @Override
    public void execute() {
        double newTopRPMs = getTopFeedForward() + topPIDController.calculate(shooter.getTopEncoder().getVelocity(), topSpeed.getAsDouble() / Constants.ShootCommandConstants.MAX_RPMS);
        double newBottomRPMs = getBottomFeedForward() + bottomPIDController.calculate(shooter.getBottomEncoder().getVelocity(), bottomSpeed.getAsDouble() / Constants.ShootCommandConstants.MAX_RPMS);

        shooter.setTopSpeed(MathUtils.clamp(newTopRPMs / Constants.ShootCommandConstants.MAX_RPMS, -1, 1));
        shooter.setBottomSpeed(MathUtils.clamp(newBottomRPMs / Constants.ShootCommandConstants.MAX_RPMS, -1, 1));
    }

    /**
     * Turn off motors when command ends
     */
    @Override
    public void end(boolean interrupted) {
        shooter.setTopSpeed(0.0);
        shooter.setBottomSpeed(0.0);
        
    }

    /**
     * Is command done?
     * For shooter, shooter runs for a certain period of time
     */
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time.get());
    }
    
}
