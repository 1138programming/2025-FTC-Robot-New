package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drivebase;


import java.util.List;


public class Limelight {


    private final Limelight3A limelight;
    private final double kAcceptableDegreeError = 0.1;


    public Limelight(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }


    private LLResult getResult() {
        return limelight.getLatestResult();
    }


    public boolean hasTarget() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }


    public double getTx() {
        LLResult result = getResult();
        if (result == null) return 0;
        return result.getTx();
    }


    public double getTy() {
        LLResult result = getResult();
        if (result == null) return 0;
        return result.getTy();
    }


    public int getTagId() {
        LLResult result = getResult();
        if (result == null) return -1;


        List<DetectorResult> results = result.getDetectorResults();
        if (results.isEmpty()) return -1;


        return results.get(0).getClassId();
    }


    public Pose3D getBotPose() {
        LLResult result = getResult();
        if (result == null) return null;
        return result.getBotpose();
    }


    public void updateRobotYaw(double yawDegrees) {
        limelight.updateRobotOrientation(yawDegrees);
    }


    // Rotate robot to center target
    public void alignToTarget(Drivebase drivebase) {
        double degreesToRotate = (-getTx()) - 180;
        while (Math.abs(getTx()) > kAcceptableDegreeError) {
            drivebase.rotateDegrees(degreesToRotate);
        }
    }
}


