package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative;
    
    public Drivetrain(File directory) {

        this.isFieldRelative = true;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
            // swerveDrive.pushOffsetsToControllers();
        } catch(Exception e) {
            throw new RuntimeException(e);
        }

    }

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, this.isFieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
    }
    
    public void newDrive(Translation2d translation, double rotation) {
        swerveDrive.drive(new Translation2d(
            -Math.pow(translation.getX(), 3) * swerveDrive.getMaximumVelocity(),
            -Math.pow(translation.getY(), 3) * swerveDrive.getMaximumVelocity()),
            -Math.pow(rotation, 3) * swerveDrive.getMaximumAngularVelocity(),
            this.isFieldRelative,
            false
        );
    }
    
    public void setOneModule() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        SmartDashboard.putNumber("Desired State1", states[0].angle.getDegrees());
        SwerveModuleState[] states2 = swerveDrive.kinematics.toSwerveModuleStates(swerveDrive.kinematics.toChassisSpeeds(states));
        SmartDashboard.putNumber("Desired State2", states2[0].angle.getDegrees());

        SmartDashboard.putNumber("Absolution Position", Rotation2d.fromDegrees(swerveDrive.getModules()[0].getAbsolutePosition()).getDegrees());
        SwerveModuleState state0 = SwerveModuleState.optimize(states[0], Rotation2d.fromDegrees(swerveDrive.getModules()[0].getAbsolutePosition()));
        SmartDashboard.putNumber("Desired State3", state0.angle.getDegrees());
        

        swerveDrive.setModuleStates(states, false);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
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
}
