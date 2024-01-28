package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Commands555 {
    

    //field relative
    public static Command driveToSetpoint(Supplier<Translation2d> target) {
        return Commands.parallel(
            Robot.drivetrain.xPID.goToSetpoint(target.get().getX()),
            Robot.drivetrain.yPID.goToSetpoint(target.get().getY())
        );
    }

    public static Command driveToSetpoint(Translation2d target) {
        return driveToSetpoint(() -> target);
    }

    public static Command driveToSetpointRelative(Supplier<Translation2d> target) {
        return driveToSetpoint(() -> 
            // robotxy + targetxy (rotate) robotrotation
            Robot.drivetrain.getSwerveDrive().getPose().getTranslation()
                .plus(target.get().rotateBy(Robot.drivetrain.getSwerveDrive().getOdometryHeading()))
        );
    }

    public static Command driveToSetpointRelative(Translation2d target) {
        return driveToSetpointRelative(() -> target);
    }

    public static Command goToAngle(Rotation2d angle) {
        return Robot.drivetrain.thetaPID.goToSetpoint(angle.getRadians());
    }
}
