package frc.robot.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {
    private double lastTx = 0;

    private String cameraName;
    private Debouncer targetDebouncer = new Debouncer(VisionConstants.TARGET_DEBOUNCE_TIME, DebounceType.kFalling);
    
    public Limelight(String cameraName) {
 
        double[] camerapose_robotspace = new double[] {-1, -1, -1, 0.0, 0.0, 0.0};
        LimelightHelpers.setLimelightNTDoubleArray(cameraName,"camerapose_robotspace", camerapose_robotspace);
        // TODO: Networktables boilerplate?

        this.cameraName = cameraName;
    }
    @AutoLogOutput
    public double getTimestampSeconds() {
        double latency = (LimelightHelpers.getLimelightNTDouble(cameraName, "cl") + LimelightHelpers.getLimelightNTDouble(cameraName, "tl")) / 1000.0;
        
        return Timer.getFPGATimestamp() - latency;
    }
    @AutoLogOutput
    public boolean hasValidTarget() {
        boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
        return targetDebouncer.calculate(hasMatch);
    }
    
    public boolean currentPipelineMatches(DetectionType type) {

        int pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
        return type.getPipe() == pipeline;

    }

    public double getObjectXSafe() {
        if (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") != 1) {
            return lastTx;
        } else {
            lastTx = getObjectTX();
            return getObjectTX();
        }
    }
    
    public void setPipelineTo(DetectionType type) {
        if (type == DetectionType.DRIVER) {
            LimelightHelpers.setCameraMode_Driver(cameraName);
        } else {
            LimelightHelpers.setCameraMode_Processor(cameraName);
        }
        
        LimelightHelpers.setPipelineIndex(cameraName, type.getPipe());
    }
    @AutoLogOutput
    public double getObjectTX() {
        return LimelightHelpers.getLimelightNTDouble(cameraName, "tx");
    }
    @AutoLogOutput
    public double getObjectTY() {
        return LimelightHelpers.getLimelightNTDouble(cameraName, "ty");
    }
    

}