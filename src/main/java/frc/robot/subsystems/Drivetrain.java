package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.frc.PIDMechanism;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative;
    
    public PIDMechanism xPID;
    public PIDMechanism yPID;
    public PIDMechanism thetaPID;
    
    
    public Drivetrain(File directory) {
            
        PIDController xController = new PIDController(
            Constants.DriveConstants.PosPID.kP.get(), 
            Constants.DriveConstants.PosPID.kI.get(), 
            Constants.DriveConstants.PosPID.kD.get()
        );

        PIDController thetaController = new PIDController(
            Constants.DriveConstants.ThetaPID.kP.get(),
            Constants.DriveConstants.ThetaPID.kI.get(),
            Constants.DriveConstants.ThetaPID.kD.get()
        );

        PIDController yController = new PIDController(
            Constants.DriveConstants.PosPID.kP.get(), 
            Constants.DriveConstants.PosPID.kI.get(), 
            Constants.DriveConstants.PosPID.kD.get()
        );

        DriveConstants.PosPID.kP.whenUpdate(xController::setP);
        DriveConstants.PosPID.kI.whenUpdate(xController::setI);
        DriveConstants.PosPID.kD.whenUpdate(xController::setD);

        DriveConstants.PosPID.kP.whenUpdate(yController::setP);
        DriveConstants.PosPID.kI.whenUpdate(yController::setI);
        DriveConstants.PosPID.kD.whenUpdate(yController::setD);

        DriveConstants.ThetaPID.kP.whenUpdate(thetaController::setP);
        DriveConstants.ThetaPID.kI.whenUpdate(thetaController::setI);
        DriveConstants.ThetaPID.kD.whenUpdate(thetaController::setD);


       
        thetaController.setTolerance(Math.toRadians(1.5), Math.toRadians(0.5));
        thetaController.enableContinuousInput(0, Math.PI * 2);

        xPID = new PIDMechanism(xController);
        yPID = new PIDMechanism(yController);
        thetaPID = new PIDMechanism(thetaController);
        
        
        

        this.isFieldRelative = true;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    public void driveFromInputs(Translation2d translation, double rotation) {
        double xSpeed = translation.getX();
        double ySpeed = translation.getY();
        double thetaSpeed = rotation;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        if (isFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, swerveDrive.getOdometryHeading());
        }

        xPID.setSpeed(chassisSpeeds.vxMetersPerSecond / Constants.DriveConstants.MAX_SPEED);
        yPID.setSpeed(chassisSpeeds.vyMetersPerSecond / Constants.DriveConstants.MAX_SPEED);
        thetaPID.setSpeed(chassisSpeeds.omegaRadiansPerSecond / Constants.DriveConstants.MAX_ROT_SPEED);


    }

    // public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    //     swerveDrive.setChassisSpeeds(chassisSpeeds);
    // }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void setIsFieldRelative(boolean relative) {
        this.isFieldRelative = relative;
    }
    public boolean getIsFieldRelative(boolean relative) {
        return this.isFieldRelative;
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    // public boolean isUsingPID() {

    // }

    @Override
    public void periodic() {

        xPID.setMeasurement(swerveDrive.getPose().getX());
        yPID.setMeasurement(swerveDrive.getPose().getY());
        thetaPID.setMeasurement(swerveDrive.getOdometryHeading().getRadians());
         
        xPID.update();
        yPID.update();
        thetaPID.update();
        
        swerveDrive.drive(new ChassisSpeeds(
            xPID.getSpeed() * Constants.DriveConstants.MAX_SPEED, 
            yPID.getSpeed() * Constants.DriveConstants.MAX_SPEED, 
            thetaPID.getSpeed() * Constants.DriveConstants.MAX_ROT_SPEED
            ));
        
    }
}
