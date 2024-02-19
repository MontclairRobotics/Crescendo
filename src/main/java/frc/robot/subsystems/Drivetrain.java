package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
    
    public final SwerveDrive swerveDrive;

    @AutoLogOutput
    private boolean isFieldRelative;
    // private AHRS navX;
    
    public Drivetrain(File directory) {

        this.isFieldRelative = true;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }

        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
        

    }
    /**
     * It drives a certain distance with a certain rotation
     */
    public void drive(Translation2d translation, double rotation) {

        swerveDrive.drive(translation, rotation, this.isFieldRelative, DriveConstants.IS_OPEN_LOOP);

    }
    /**
     * Moves chassis
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
    /**
     * logs data: module positions, gyro rotation, and pose
     */
    @Override
    public void periodic() {
        Logger.recordOutput("Drivetrain/Module-Positions",getSwerveDrive().getModulePositions());
        Logger.recordOutput("Drivetrain/Gyro-Rotation",getSwerveDrive().getGyroRotation3d());
        Logger.recordOutput("Drivetrain/Pose",getSwerveDrive().getPose());        
        //RobotContainer.field.setRobotPose(swerveDrive.getPose());
    }
    /**
     * sets isFieldRelative to either true or false, used for getIsFieldRelative
     */
    public void setIsFieldRelative(boolean relative) {
        this.isFieldRelative = relative;
    }
    /**
     * Returns if the paramter 
     */
    public boolean getIsFieldRelative(boolean relative) {
        return this.isFieldRelative;
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }
    /**
     * Resets the odometer 
     */
    public void resetOdometry() {
        this.swerveDrive.resetOdometry(new Pose2d(0.0,0.0, new Rotation2d(0.0)));
    }
    @AutoLogOutput
    /**
     * returns direction
     */
    public Rotation2d getRotation() {
        
        return this.swerveDrive.getOdometryHeading();
    }

    public void enableFieldRelative() {
        System.out.println("Enabled field relative!");
        isFieldRelative = true;
    }

    public void disableFieldRelative() {
        System.out.println("Disabled field relative!");
        isFieldRelative = false;
    }

    /**
     * Returns angle of the robot between 0 and 360
     */
    public Rotation2d getWrappedRotation() {
        return Rotation2d.fromDegrees(getRotation().getDegrees() % 360);
    }
    

  
    public void setInputFromController(CommandPS5Controller controller) {
      
        double thetaSpeed = -MathUtil.applyDeadband(controller.getRightX(), 0.05) * DriveConstants.MAX_ROT_SPEED;

        double xSpeed = -MathUtil.applyDeadband(controller.getLeftX(), 0.05) * DriveConstants.MAX_SPEED;
        double ySpeed = -MathUtil.applyDeadband(controller.getLeftY(), 0.05) * DriveConstants.MAX_SPEED;
        
        Translation2d targetTranslation = new Translation2d(ySpeed,xSpeed);
        Logger.recordOutput("Drivetrain/Controller-Translation", targetTranslation);
        Logger.recordOutput("Drivetrain/Controller-Theta", thetaSpeed);

        this.drive(targetTranslation, thetaSpeed);
        
    }
    

}




